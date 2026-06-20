/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "feature_tracker.h"

#include <algorithm>

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

static double imageMeanIntensity(const cv::Mat &img)
{
    if (img.empty())
        return 0.0;
    return cv::mean(img)[0];
}

static double imageBlurScore(const cv::Mat &img)
{
    if (img.empty())
        return 0.0;
    cv::Mat lap;
    cv::Laplacian(img, lap, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev.val[0] * stddev.val[0];
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
    enable_recovery_bridge = false;
    recovery_max_cnt = 30;
    recovery_time_tolerance = 0.01;
    recovery_prev_match_radius = 4.0;
    recovery_min_dist_ratio = 0.5;
    recovery_request_min_tracks = 80;
    recovery_request_lost_ratio = 0.35;
    recovery_request_timeout_ms = 0.0;
    recovery_request_max_mean_flow = 0.0;
    recovery_request_min_blur_score = 0.0;
    recovery_request_brightness_delta = 0.0;
    recovery_request_cooldown_ms = 0.0;
    recovery_request_min_signal_count = 1;
    recovery_request_require_degradation_signal = false;
    last_recovery_request_time = -1.0;
    recovery_max_flow_error = 0.0;
    recovery_min_flow_tracks = 20;
    prev_mean_intensity = 0.0;
    prev_blur_score = 0.0;
    has_prev_image_quality = false;
    recovery_candidate_time = -1.0;
}

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    double cur_mean_intensity = imageMeanIntensity(cur_img);
    double cur_blur_score = imageBlurScore(cur_img);
    cur_pts.clear();

    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        vector<cv::Point2f> raw_prev_pts = prev_pts;
        vector<int> raw_ids = ids;
        vector<int> raw_track_cnt = track_cnt;
        if(hasPrediction)
        {
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        if (enable_recovery_bridge && recovery_request_callback && recovery_request_timeout_ms > 0.0)
        {
            int active_num = 0;
            vector<cv::Point2f> lost_prev_pts;
            vector<int> lost_ids;
            vector<int> lost_track_cnt;
            vector<cv::Point2f> active_prev_pts;
            vector<cv::Point2f> active_flows;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                {
                    active_num++;
                    if (i < raw_prev_pts.size() && i < cur_pts.size())
                    {
                        active_prev_pts.push_back(raw_prev_pts[i]);
                        active_flows.push_back(cur_pts[i] - raw_prev_pts[i]);
                    }
                }
                else if (i < raw_prev_pts.size())
                {
                    lost_prev_pts.push_back(raw_prev_pts[i]);
                    lost_ids.push_back(raw_ids[i]);
                    lost_track_cnt.push_back(raw_track_cnt[i]);
                }
            }
            double flow_sum = 0.0;
            int flow_count = 0;
            for (size_t i = 0; i < active_flows.size(); i++)
            {
                flow_sum += sqrt(active_flows[i].x * active_flows[i].x + active_flows[i].y * active_flows[i].y);
                flow_count++;
            }
            double mean_flow = flow_count > 0 ? flow_sum / static_cast<double>(flow_count) : 0.0;
            double brightness_delta = has_prev_image_quality ? fabs(cur_mean_intensity - prev_mean_intensity) : 0.0;
            double lost_ratio = status.empty() ? 0.0 : static_cast<double>(lost_prev_pts.size()) / static_cast<double>(status.size());
            bool track_count_bad = active_num < recovery_request_min_tracks;
            bool lost_ratio_bad = lost_ratio >= recovery_request_lost_ratio;
            bool high_speed_bad = recovery_request_max_mean_flow > 0.0 && mean_flow >= recovery_request_max_mean_flow;
            bool blur_bad = recovery_request_min_blur_score > 0.0 && cur_blur_score <= recovery_request_min_blur_score;
            bool brightness_bad = recovery_request_brightness_delta > 0.0 && brightness_delta >= recovery_request_brightness_delta;
            int tracking_signal_count = (track_count_bad ? 1 : 0) + (lost_ratio_bad ? 1 : 0);
            int degradation_signal_count = (high_speed_bad ? 1 : 0) + (blur_bad ? 1 : 0) + (brightness_bad ? 1 : 0);
            int signal_count = tracking_signal_count + degradation_signal_count;
            bool signal_count_ok = signal_count >= recovery_request_min_signal_count;
            bool degradation_policy_ok = !recovery_request_require_degradation_signal ||
                                         (tracking_signal_count > 0 && degradation_signal_count > 0);
            bool cooldown_ok = recovery_request_cooldown_ms <= 0.0 ||
                               last_recovery_request_time < 0.0 ||
                               (cur_time - last_recovery_request_time) * 1000.0 >= recovery_request_cooldown_ms;
            bool should_request = !lost_prev_pts.empty() &&
                                  signal_count_ok &&
                                  degradation_policy_ok &&
                                  cooldown_ok;
            if (should_request)
            {
                last_recovery_request_time = cur_time;
                {
                    std::lock_guard<std::mutex> lock(recovery_mutex);
                    recovery_candidate_time = -1.0;
                    recovery_candidates.clear();
                }
                recovery_request_callback(cur_time, lost_prev_pts, lost_ids, lost_track_cnt);
                std::unique_lock<std::mutex> lock(recovery_mutex);
                recovery_condition.wait_for(lock,
                                             std::chrono::milliseconds(static_cast<int>(recovery_request_timeout_ms)),
                                             [&]() {
                                                 return recovery_candidate_time >= 0 &&
                                                        fabs(recovery_candidate_time - cur_time) <= recovery_time_tolerance;
                                             });
                ROS_WARN("recovery request health active=%d lost=%lu ratio=%.3f flow=%.2f blur=%.1f brightness_delta=%.1f reasons=%s%s%s%s%s",
                         active_num,
                         lost_prev_pts.size(),
                         lost_ratio,
                         mean_flow,
                         cur_blur_score,
                         brightness_delta,
                         track_count_bad ? "track_count " : "",
                         lost_ratio_bad ? "lost_ratio " : "",
                         high_speed_bad ? "high_speed " : "",
                         blur_bad ? "blur " : "",
                         brightness_bad ? "brightness " : "");
            }
        }
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        vector<cv::Point2f> active_prev_pts;
        vector<cv::Point2f> active_flows;
        size_t active_index = 0;
        for (size_t i = 0; i < status.size() && i < raw_prev_pts.size(); i++)
        {
            if (status[i])
            {
                active_prev_pts.push_back(raw_prev_pts[i]);
                if (active_index < cur_pts.size())
                    active_flows.push_back(cur_pts[active_index] - raw_prev_pts[i]);
                active_index++;
            }
        }
        int recovered_num = applyRecoveryCandidates(raw_prev_pts, raw_ids, raw_track_cnt, status, active_prev_pts, active_flows);
        if (recovered_num > 0)
            ROS_WARN("LightGlue recovery bridge accepted %d tracks; active tracks now %lu", recovered_num, cur_pts.size());
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    prev_mean_intensity = cur_mean_intensity;
    prev_blur_score = cur_blur_score;
    has_prev_image_quality = true;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}

void FeatureTracker::configureRecoveryBridge(bool enable, int max_recoveries, double time_tolerance,
                                             double prev_match_radius, double min_dist_ratio)
{
    enable_recovery_bridge = enable;
    recovery_max_cnt = std::max(0, max_recoveries);
    recovery_time_tolerance = std::max(0.0, time_tolerance);
    recovery_prev_match_radius = std::max(0.0, prev_match_radius);
    recovery_min_dist_ratio = std::max(0.0, min_dist_ratio);
    ROS_WARN("LightGlue recovery bridge %s: max=%d time_tol=%.4f prev_radius=%.2f min_dist_ratio=%.2f",
             enable_recovery_bridge ? "enabled" : "disabled",
             recovery_max_cnt,
             recovery_time_tolerance,
             recovery_prev_match_radius,
             recovery_min_dist_ratio);
}

void FeatureTracker::configureRecoveryRequest(int min_tracks, double lost_ratio, double timeout_ms,
                                              double cooldown_ms, int min_signal_count,
                                              bool require_degradation_signal)
{
    recovery_request_min_tracks = std::max(0, min_tracks);
    recovery_request_lost_ratio = std::max(0.0, lost_ratio);
    recovery_request_timeout_ms = std::max(0.0, timeout_ms);
    recovery_request_cooldown_ms = std::max(0.0, cooldown_ms);
    recovery_request_min_signal_count = std::max(1, min_signal_count);
    recovery_request_require_degradation_signal = require_degradation_signal;
    ROS_WARN("LightGlue recovery request: min_tracks=%d lost_ratio=%.3f timeout=%.1fms cooldown=%.1fms min_signals=%d require_degradation=%d",
             recovery_request_min_tracks,
             recovery_request_lost_ratio,
             recovery_request_timeout_ms,
             recovery_request_cooldown_ms,
             recovery_request_min_signal_count,
             recovery_request_require_degradation_signal ? 1 : 0);
}

void FeatureTracker::configureRecoveryDegradation(double max_mean_flow, double min_blur_score, double brightness_delta)
{
    recovery_request_max_mean_flow = std::max(0.0, max_mean_flow);
    recovery_request_min_blur_score = std::max(0.0, min_blur_score);
    recovery_request_brightness_delta = std::max(0.0, brightness_delta);
    ROS_WARN("LightGlue recovery degradation triggers: max_mean_flow=%.2f min_blur=%.1f brightness_delta=%.1f",
             recovery_request_max_mean_flow,
             recovery_request_min_blur_score,
             recovery_request_brightness_delta);
}

void FeatureTracker::configureRecoveryGeometry(double max_flow_error, int min_flow_tracks)
{
    recovery_max_flow_error = std::max(0.0, max_flow_error);
    recovery_min_flow_tracks = std::max(0, min_flow_tracks);
    ROS_WARN("LightGlue recovery geometry gate: max_flow_error=%.2f min_flow_tracks=%d",
             recovery_max_flow_error,
             recovery_min_flow_tracks);
}

void FeatureTracker::setRecoveryRequestCallback(std::function<void(double,
                                                                   const vector<cv::Point2f> &,
                                                                   const vector<int> &,
                                                                   const vector<int> &)> callback)
{
    recovery_request_callback = callback;
}

void FeatureTracker::setRecoveryCandidates(double timestamp, const vector<RecoveryCandidate> &candidates)
{
    if (!enable_recovery_bridge)
        return;
    std::lock_guard<std::mutex> lock(recovery_mutex);
    recovery_candidate_time = timestamp;
    recovery_candidates = candidates;
    recovery_condition.notify_all();
}

int FeatureTracker::applyRecoveryCandidates(const vector<cv::Point2f> &raw_prev_pts,
                                            const vector<int> &raw_ids,
                                            const vector<int> &raw_track_cnt,
                                            const vector<uchar> &status,
                                            const vector<cv::Point2f> &active_prev_pts,
                                            const vector<cv::Point2f> &active_flows)
{
    if (!enable_recovery_bridge || recovery_max_cnt <= 0 || raw_prev_pts.empty())
        return 0;

    vector<RecoveryCandidate> candidates;
    {
        std::lock_guard<std::mutex> lock(recovery_mutex);
        if (recovery_candidate_time < 0 || fabs(recovery_candidate_time - cur_time) > recovery_time_tolerance)
            return 0;
        candidates = recovery_candidates;
        recovery_candidate_time = -1.0;
        recovery_candidates.clear();
    }
    if (candidates.empty())
        return 0;

    std::sort(candidates.begin(), candidates.end(), [](const RecoveryCandidate &a, const RecoveryCandidate &b) {
        return a.score > b.score;
    });

    int accepted = 0;
    for (const auto &candidate : candidates)
    {
        if (accepted >= recovery_max_cnt)
            break;
        if (!inBorder(candidate.cur_pt))
            continue;

        int best_index = -1;
        double best_distance = recovery_prev_match_radius;
        for (size_t i = 0; i < raw_prev_pts.size(); i++)
        {
            if (i < status.size() && status[i])
                continue;
            double d = ::distance(raw_prev_pts[i], candidate.prev_pt);
            if (d < best_distance)
            {
                best_distance = d;
                best_index = static_cast<int>(i);
            }
        }
        if (best_index < 0)
            continue;

        if (recovery_max_flow_error > 0.0)
        {
            if (active_prev_pts.size() != active_flows.size())
                continue;
            cv::Point2f candidate_flow = candidate.cur_pt - candidate.prev_pt;
            double local_radius = std::max(recovery_prev_match_radius * 4.0, static_cast<double>(MIN_DIST) * 2.0);
            vector<double> flow_errors;
            flow_errors.reserve(active_flows.size());
            for (size_t i = 0; i < active_prev_pts.size(); i++)
            {
                if (::distance(active_prev_pts[i], candidate.prev_pt) > local_radius)
                    continue;
                cv::Point2f flow_delta = active_flows[i] - candidate_flow;
                flow_errors.push_back(sqrt(flow_delta.x * flow_delta.x + flow_delta.y * flow_delta.y));
            }
            if (static_cast<int>(flow_errors.size()) < recovery_min_flow_tracks)
                continue;
            std::nth_element(flow_errors.begin(),
                             flow_errors.begin() + flow_errors.size() / 2,
                             flow_errors.end());
            double median_flow_error = flow_errors[flow_errors.size() / 2];
            if (median_flow_error > recovery_max_flow_error)
                continue;
        }

        bool too_close = false;
        double min_dist = MIN_DIST * recovery_min_dist_ratio;
        for (const auto &pt : cur_pts)
        {
            if (::distance(pt, candidate.cur_pt) < min_dist)
            {
                too_close = true;
                break;
            }
        }
        if (too_close)
            continue;

        cur_pts.push_back(candidate.cur_pt);
        ids.push_back(raw_ids[best_index]);
        track_cnt.push_back(raw_track_cnt[best_index] + 1);
        accepted++;
    }
    return accepted;
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}
