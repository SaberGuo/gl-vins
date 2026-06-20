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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;
double recovery_image_delay_ms = 0.0;
ros::Publisher pub_recovery_request;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty() && recovery_image_delay_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(recovery_image_delay_ms)));
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty() && recovery_image_delay_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(recovery_image_delay_ms)));
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 8)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void recovery_candidate_callback(const sensor_msgs::PointCloudConstPtr &candidate_msg)
{
    vector<RecoveryCandidate> candidates;
    if(candidate_msg->channels.size() < 2)
    {
        ROS_WARN_THROTTLE(2.0, "ignore recovery candidates: expected channels prev_u, prev_v, optional score");
        return;
    }

    candidates.reserve(candidate_msg->points.size());
    for (unsigned int i = 0; i < candidate_msg->points.size(); i++)
    {
        RecoveryCandidate candidate;
        candidate.prev_pt = cv::Point2f(candidate_msg->channels[0].values[i],
                                        candidate_msg->channels[1].values[i]);
        candidate.cur_pt = cv::Point2f(candidate_msg->points[i].x,
                                       candidate_msg->points[i].y);
        candidate.score = candidate_msg->channels.size() > 2 ? candidate_msg->channels[2].values[i] : 1.0f;
        candidates.push_back(candidate);
    }
    estimator.inputRecoveryCandidates(candidate_msg->header.stamp.toSec(), candidates);
}

void publish_recovery_request(double t,
                              const vector<cv::Point2f> &lost_prev_pts,
                              const vector<int> &lost_ids,
                              const vector<int> &lost_track_cnt)
{
    if (!pub_recovery_request)
        return;

    sensor_msgs::PointCloud request_msg;
    request_msg.header.stamp = ros::Time(t);
    request_msg.header.frame_id = "camera";

    sensor_msgs::ChannelFloat32 id_channel;
    sensor_msgs::ChannelFloat32 track_cnt_channel;
    id_channel.name = "id";
    track_cnt_channel.name = "track_cnt";

    for (size_t i = 0; i < lost_prev_pts.size(); i++)
    {
        geometry_msgs::Point32 p;
        p.x = lost_prev_pts[i].x;
        p.y = lost_prev_pts[i].y;
        p.z = 0.0f;
        request_msg.points.push_back(p);
        id_channel.values.push_back(i < lost_ids.size() ? lost_ids[i] : -1);
        track_cnt_channel.values.push_back(i < lost_track_cnt.size() ? lost_track_cnt[i] : 0);
    }
    request_msg.channels.push_back(id_channel);
    request_msg.channels.push_back(track_cnt_channel);
    pub_recovery_request.publish(request_msg);
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();
    bool external_feature_only = false;
    bool enable_lightglue_recovery_bridge = false;
    int recovery_max_cnt = 30;
    double recovery_time_tolerance = 0.01;
    double recovery_prev_match_radius = 4.0;
    double recovery_min_dist_ratio = 0.5;
    int recovery_request_min_tracks = 80;
    double recovery_request_lost_ratio = 0.35;
    double recovery_request_timeout_ms = 0.0;
    double recovery_request_max_mean_flow = 0.0;
    double recovery_request_min_blur_score = 0.0;
    double recovery_request_brightness_delta = 0.0;
    double recovery_max_flow_error = 0.0;
    int recovery_min_flow_tracks = 20;
    n.param("external_feature_only", external_feature_only, false);
    n.param("enable_lightglue_recovery_bridge", enable_lightglue_recovery_bridge, false);
    n.param("recovery_max_cnt", recovery_max_cnt, 30);
    n.param("recovery_time_tolerance", recovery_time_tolerance, 0.01);
    n.param("recovery_prev_match_radius", recovery_prev_match_radius, 4.0);
    n.param("recovery_min_dist_ratio", recovery_min_dist_ratio, 0.5);
    n.param("recovery_request_min_tracks", recovery_request_min_tracks, 80);
    n.param("recovery_request_lost_ratio", recovery_request_lost_ratio, 0.35);
    n.param("recovery_request_timeout_ms", recovery_request_timeout_ms, 0.0);
    n.param("recovery_request_max_mean_flow", recovery_request_max_mean_flow, 0.0);
    n.param("recovery_request_min_blur_score", recovery_request_min_blur_score, 0.0);
    n.param("recovery_request_brightness_delta", recovery_request_brightness_delta, 0.0);
    n.param("recovery_max_flow_error", recovery_max_flow_error, 0.0);
    n.param("recovery_min_flow_tracks", recovery_min_flow_tracks, 20);
    n.param("recovery_image_delay_ms", recovery_image_delay_ms, 0.0);
    estimator.configureRecoveryBridge(enable_lightglue_recovery_bridge,
                                      recovery_max_cnt,
                                      recovery_time_tolerance,
                                      recovery_prev_match_radius,
                                      recovery_min_dist_ratio);
    estimator.configureRecoveryRequest(recovery_request_min_tracks,
                                       recovery_request_lost_ratio,
                                       recovery_request_timeout_ms);
    estimator.configureRecoveryDegradation(recovery_request_max_mean_flow,
                                           recovery_request_min_blur_score,
                                           recovery_request_brightness_delta);
    estimator.configureRecoveryGeometry(recovery_max_flow_error,
                                        recovery_min_flow_tracks);
    if (external_feature_only)
        ROS_WARN("external_feature_only enabled: estimator consumes /feature_tracker/feature and does not run internal image frontend");
    if (recovery_image_delay_ms > 0)
        ROS_WARN("recovery image delay enabled: %.1f ms", recovery_image_delay_ms);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    if(enable_lightglue_recovery_bridge)
    {
        pub_recovery_request = n.advertise<sensor_msgs::PointCloud>("/feature_tracker/recovery_request", 2000);
        estimator.setRecoveryRequestCallback(publish_recovery_request);
    }

    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_recovery_candidate;
    if(enable_lightglue_recovery_bridge)
        sub_recovery_candidate = n.subscribe("/feature_tracker/recovery_candidates", 2000, recovery_candidate_callback);
    ros::Subscriber sub_img0;
    ros::Subscriber sub_img1;
    if(!external_feature_only)
    {
        sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
        if(STEREO)
        {
            sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
        }
    }
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread;
    if(!external_feature_only)
        sync_thread = std::thread{sync_process};
    ros::spin();

    if(sync_thread.joinable())
        sync_thread.join();

    return 0;
}
