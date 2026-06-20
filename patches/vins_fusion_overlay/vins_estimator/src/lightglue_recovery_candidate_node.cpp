/*******************************************************
 * LightGlue recovery candidate publisher for VINS-Fusion.
 *
 * This node intentionally does not publish VINS feature tracks. It only
 * publishes raw-pixel recovery candidates that the original C++ FeatureTracker
 * may accept for KLT-lost tracks.
 *******************************************************/

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <map>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#ifdef ENABLE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace
{

class LightGlueRecoveryCandidateNode
{
public:
    LightGlueRecoveryCandidateNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param<std::string>("image_topic", image_topic_, "/cam0/image_raw");
        pnh.param<std::string>("output_topic", output_topic_, "/feature_tracker/recovery_candidates");
        pnh.param<std::string>("backend", backend_, "fake_grid");
        pnh.param<int>("publish_every_n", publish_every_n_, 1);
        pnh.param<int>("fake_grid_step", fake_grid_step_, 5);
        pnh.param<int>("max_candidates", max_candidates_, 20000);
        pnh.param<double>("fake_motion_u", fake_motion_u_, 0.0);
        pnh.param<double>("fake_motion_v", fake_motion_v_, 0.0);
        pnh.param<std::string>("superpoint_onnx", superpoint_onnx_, "");
        pnh.param<std::string>("lightglue_onnx", lightglue_onnx_, "");
        pnh.param<std::string>("pipeline_onnx", pipeline_onnx_, "");
        pnh.param<int>("model_width", model_width_, 0);
        pnh.param<int>("model_height", model_height_, 0);
        pnh.param<double>("min_score", min_score_, 0.0);
        int use_cuda_int = 0;
        pnh.param<int>("use_cuda", use_cuda_int, 0);
        use_cuda_ = use_cuda_int != 0;
        pnh.param<int>("cuda_device_id", cuda_device_id_, 0);
        int request_only_int = 1;
        pnh.param<int>("request_only", request_only_int, 1);
        request_only_ = request_only_int != 0;
        pnh.param<double>("request_time_tolerance", request_time_tolerance_, 0.02);
        pnh.param<double>("request_prev_radius", request_prev_radius_, 12.0);
        pnh.param<double>("request_distance_penalty", request_distance_penalty_, 0.03);
        pnh.param<double>("image_cache_sec", image_cache_sec_, 5.0);

        publish_every_n_ = std::max(1, publish_every_n_);
        fake_grid_step_ = std::max(2, fake_grid_step_);
        max_candidates_ = std::max(0, max_candidates_);
        model_width_ = std::max(0, model_width_);
        model_height_ = std::max(0, model_height_);

        pub_ = nh.advertise<sensor_msgs::PointCloud>(output_topic_, 50);
        sub_ = nh.subscribe(image_topic_, 50, &LightGlueRecoveryCandidateNode::imageCallback, this);
        sub_request_ = nh.subscribe("/feature_tracker/recovery_request", 50,
                                    &LightGlueRecoveryCandidateNode::requestCallback, this);

        ROS_WARN("lightglue_recovery_candidate_node backend=%s image_topic=%s output_topic=%s publish_every_n=%d",
                 backend_.c_str(), image_topic_.c_str(), output_topic_.c_str(), publish_every_n_);
        ROS_WARN("lightglue_recovery_candidate_node use_cuda=%d model_size=%dx%d min_score=%.3f max_candidates=%d",
                 use_cuda_ ? 1 : 0, model_width_, model_height_, min_score_, max_candidates_);
        ROS_WARN("lightglue_recovery_candidate_node request_only=%d request_prev_radius=%.1f request_time_tol=%.3f",
                 request_only_ ? 1 : 0, request_prev_radius_, request_time_tolerance_);

        if (backend_ == "onnx")
        {
#ifdef ENABLE_ONNXRUNTIME
            if (!initOnnx())
            {
                ROS_ERROR("failed to initialize ONNX backend; falling back to fake_grid");
                backend_ = "fake_grid";
            }
#else
            ROS_ERROR("ONNX backend requested but this build has no ONNX Runtime runner linked. "
                      "Install ONNX Runtime C++ and wire SuperPoint/LightGlue inference before using backend:=onnx. "
                      "superpoint_onnx=%s lightglue_onnx=%s",
                      superpoint_onnx_.c_str(), lightglue_onnx_.c_str());
            backend_ = "fake_grid";
#endif
        }
        else if (backend_ != "fake_grid")
        {
            ROS_WARN("Unknown backend '%s'; falling back to fake_grid dry-run publisher.", backend_.c_str());
            backend_ = "fake_grid";
        }
    }

private:
    cv::Mat imageFromMsg(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        if (msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = msg->header;
            img.height = msg->height;
            img.width = msg->width;
            img.is_bigendian = msg->is_bigendian;
            img.step = msg->step;
            img.data = msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
        {
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        return ptr->image.clone();
    }

    static float deterministicScore(int x, int y, int frame_index)
    {
        unsigned int h = static_cast<unsigned int>(x * 73856093) ^
                         static_cast<unsigned int>(y * 19349663) ^
                         static_cast<unsigned int>(frame_index * 83492791);
        return static_cast<float>(h % 1000003) / 1000003.0f;
    }

    void publishFakeGrid(const std_msgs::Header &header, const cv::Mat &cur_img)
    {
        sensor_msgs::PointCloud msg;
        msg.header = header;

        sensor_msgs::ChannelFloat32 prev_u;
        sensor_msgs::ChannelFloat32 prev_v;
        sensor_msgs::ChannelFloat32 score;
        prev_u.name = "prev_u";
        prev_v.name = "prev_v";
        score.name = "score";

        const int border = std::max(8, fake_grid_step_);
        for (int y = border; y < cur_img.rows - border; y += fake_grid_step_)
        {
            for (int x = border; x < cur_img.cols - border; x += fake_grid_step_)
            {
                if (max_candidates_ > 0 && static_cast<int>(msg.points.size()) >= max_candidates_)
                    break;

                geometry_msgs::Point32 p;
                p.x = static_cast<float>(x + fake_motion_u_);
                p.y = static_cast<float>(y + fake_motion_v_);
                p.z = 0.0f;
                if (p.x <= 1.0f || p.y <= 1.0f || p.x >= cur_img.cols - 1 || p.y >= cur_img.rows - 1)
                    continue;

                msg.points.push_back(p);
                prev_u.values.push_back(static_cast<float>(x));
                prev_v.values.push_back(static_cast<float>(y));
                score.values.push_back(deterministicScore(x, y, frame_index_));
            }
            if (max_candidates_ > 0 && static_cast<int>(msg.points.size()) >= max_candidates_)
                break;
        }

        msg.channels.push_back(prev_u);
        msg.channels.push_back(prev_v);
        msg.channels.push_back(score);
        pub_.publish(msg);

        ROS_INFO_THROTTLE(2.0, "published %lu fake recovery candidates at %.6f",
                          msg.points.size(), header.stamp.toSec());
    }

#ifdef ENABLE_ONNXRUNTIME
    bool initOnnx()
    {
        if (pipeline_onnx_.empty())
        {
            ROS_ERROR("backend:=onnx requires _pipeline_onnx for superpoint_lightglue_pipeline.onnx");
            return false;
        }

        try
        {
            ort_env_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "vins_lightglue_recovery"));
            Ort::SessionOptions options;
            options.SetIntraOpNumThreads(1);
            options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            if (use_cuda_)
            {
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = cuda_device_id_;
                options.AppendExecutionProvider_CUDA(cuda_options);
                ROS_WARN("ONNX Runtime CUDA provider requested on device %d", cuda_device_id_);
            }
            ort_session_.reset(new Ort::Session(*ort_env_, pipeline_onnx_.c_str(), options));
            input_names_.clear();
            output_names_.clear();
            input_name_ptrs_.clear();
            output_name_ptrs_.clear();
            Ort::AllocatorWithDefaultOptions allocator;
            for (size_t i = 0; i < ort_session_->GetInputCount(); i++)
            {
                auto name = ort_session_->GetInputNameAllocated(i, allocator);
                input_names_.push_back(name.get());
            }
            for (size_t i = 0; i < ort_session_->GetOutputCount(); i++)
            {
                auto name = ort_session_->GetOutputNameAllocated(i, allocator);
                output_names_.push_back(name.get());
            }
            for (const auto &name : input_names_)
                input_name_ptrs_.push_back(name.c_str());
            for (const auto &name : output_names_)
                output_name_ptrs_.push_back(name.c_str());
            if (input_name_ptrs_.empty() || output_name_ptrs_.size() < 3)
            {
                ROS_ERROR("unexpected ONNX pipeline IO: inputs=%lu outputs=%lu",
                          input_name_ptrs_.size(), output_name_ptrs_.size());
                return false;
            }
            ROS_WARN("ONNX pipeline initialized: %s input=%s outputs=%lu",
                     pipeline_onnx_.c_str(), input_name_ptrs_[0], output_name_ptrs_.size());
        }
        catch (const Ort::Exception &e)
        {
            ROS_ERROR("ONNX Runtime init failed: %s", e.what());
            return false;
        }
        return true;
    }

    cv::Mat prepareImage(const cv::Mat &img, int &net_w, int &net_h, double &sx, double &sy)
    {
        cv::Mat gray = img;
        if (gray.channels() != 1)
            cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

        net_w = model_width_ > 0 ? model_width_ : gray.cols;
        net_h = model_height_ > 0 ? model_height_ : gray.rows;
        cv::Mat resized;
        if (net_w != gray.cols || net_h != gray.rows)
            cv::resize(gray, resized, cv::Size(net_w, net_h), 0, 0, cv::INTER_AREA);
        else
            resized = gray;

        sx = static_cast<double>(gray.cols) / static_cast<double>(net_w);
        sy = static_cast<double>(gray.rows) / static_cast<double>(net_h);
        cv::Mat f;
        resized.convertTo(f, CV_32FC1, 1.0 / 255.0);
        return f;
    }

    struct CandidateRow
    {
        cv::Point2f prev_pt;
        cv::Point2f cur_pt;
        float score = 0.0f;
        double rank = 0.0;
    };

    void publishOnnxMatches(const std_msgs::Header &header,
                            const cv::Mat &prev_img,
                            const cv::Mat &cur_img,
                            const std::vector<cv::Point2f> &lost_prev_pts = std::vector<cv::Point2f>())
    {
        if (!ort_session_)
            return;

        try
        {
            int net_w0 = 0, net_h0 = 0, net_w1 = 0, net_h1 = 0;
            double sx0 = 1.0, sy0 = 1.0, sx1 = 1.0, sy1 = 1.0;
            cv::Mat prev_f = prepareImage(prev_img, net_w0, net_h0, sx0, sy0);
            cv::Mat cur_f = prepareImage(cur_img, net_w1, net_h1, sx1, sy1);
            if (net_w0 != net_w1 || net_h0 != net_h1)
            {
                ROS_WARN("skip ONNX inference: previous/current network sizes differ");
                return;
            }

            const size_t image_size = static_cast<size_t>(net_w0) * static_cast<size_t>(net_h0);
            std::vector<float> input(2 * image_size);
            std::memcpy(input.data(), prev_f.ptr<float>(), image_size * sizeof(float));
            std::memcpy(input.data() + image_size, cur_f.ptr<float>(), image_size * sizeof(float));

            std::array<int64_t, 4> input_shape{{2, 1, net_h0, net_w0}};
            Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, input.data(), input.size(), input_shape.data(), input_shape.size());

            const double t0 = ros::Time::now().toSec();
            ROS_WARN_THROTTLE(5.0, "running ONNX inference at %.6f", header.stamp.toSec());
            auto outputs = ort_session_->Run(Ort::RunOptions{nullptr},
                                             input_name_ptrs_.data(), &input_tensor, 1,
                                             output_name_ptrs_.data(), output_name_ptrs_.size());
            const double dt_ms = (ros::Time::now().toSec() - t0) * 1000.0;
            if (outputs.size() < 3)
            {
                ROS_WARN("ONNX output count too small: %lu", outputs.size());
                return;
            }

            auto kpt_info = outputs[0].GetTensorTypeAndShapeInfo();
            auto match_info = outputs[1].GetTensorTypeAndShapeInfo();
            auto score_info = outputs[2].GetTensorTypeAndShapeInfo();
            std::vector<int64_t> kpt_shape = kpt_info.GetShape();
            std::vector<int64_t> match_shape = match_info.GetShape();
            std::vector<int64_t> score_shape = score_info.GetShape();
            if (kpt_shape.size() != 3 || kpt_shape[0] < 2 || kpt_shape[2] != 2 || match_shape.size() != 2)
            {
                ROS_WARN("unexpected ONNX shapes: keypoints dims=%lu matches dims=%lu scores dims=%lu",
                         kpt_shape.size(), match_shape.size(), score_shape.size());
                return;
            }

            const int64_t k_count = kpt_shape[1];
            const int64_t m_count = match_shape[0];
            const int64_t m_width = match_shape[1];
            const int64_t *keypoints = outputs[0].GetTensorData<int64_t>();
            const int64_t *matches = outputs[1].GetTensorData<int64_t>();
            const float *scores = outputs[2].GetTensorData<float>();

            std::vector<CandidateRow> rows;
            rows.reserve(static_cast<size_t>(m_count));

            for (int64_t i = 0; i < m_count; i++)
            {
                if (m_width < 2)
                    continue;

                int64_t a = matches[i * m_width + 0];
                int64_t b = matches[i * m_width + 1];
                int64_t c = m_width >= 3 ? matches[i * m_width + 2] : -1;
                int64_t idx0 = -1;
                int64_t idx1 = -1;
                if (m_width >= 3 && a == 0 && b >= 0 && b < k_count && c >= 0 && c < k_count)
                {
                    idx0 = b;
                    idx1 = c;
                }
                else if (a >= 0 && a < k_count && b >= 0 && b < k_count)
                {
                    idx0 = a;
                    idx1 = b;
                }
                else if (m_width >= 3 && b >= 0 && b < k_count && c >= 0 && c < k_count)
                {
                    idx0 = b;
                    idx1 = c;
                }
                if (idx0 < 0 || idx1 < 0)
                    continue;

                float s = scores && i < static_cast<int64_t>(score_info.GetElementCount()) ? scores[i] : 1.0f;
                if (s < min_score_)
                    continue;

                const int64_t prev_base = (0 * k_count + idx0) * 2;
                const int64_t cur_base = (1 * k_count + idx1) * 2;
                float pu = static_cast<float>(keypoints[prev_base + 0] * sx0);
                float pv = static_cast<float>(keypoints[prev_base + 1] * sy0);
                float cu = static_cast<float>(keypoints[cur_base + 0] * sx1);
                float cv_y = static_cast<float>(keypoints[cur_base + 1] * sy1);
                if (cu <= 1.0f || cv_y <= 1.0f || cu >= cur_img.cols - 1 || cv_y >= cur_img.rows - 1)
                    continue;
                if (pu <= 1.0f || pv <= 1.0f || pu >= prev_img.cols - 1 || pv >= prev_img.rows - 1)
                    continue;

                double rank = s;
                if (!lost_prev_pts.empty())
                {
                    double best_d = 1e9;
                    for (const auto &lost_pt : lost_prev_pts)
                    {
                        double dx = pu - lost_pt.x;
                        double dy = pv - lost_pt.y;
                        double d = std::sqrt(dx * dx + dy * dy);
                        if (d < best_d)
                            best_d = d;
                    }
                    if (best_d > request_prev_radius_)
                        continue;
                    rank = static_cast<double>(s) - request_distance_penalty_ * best_d;
                }

                CandidateRow row;
                row.prev_pt = cv::Point2f(pu, pv);
                row.cur_pt = cv::Point2f(cu, cv_y);
                row.score = s;
                row.rank = rank;
                rows.push_back(row);
            }

            std::sort(rows.begin(), rows.end(), [](const CandidateRow &a, const CandidateRow &b) {
                return a.rank > b.rank;
            });

            sensor_msgs::PointCloud msg;
            msg.header = header;
            sensor_msgs::ChannelFloat32 prev_u, prev_v, score;
            prev_u.name = "prev_u";
            prev_v.name = "prev_v";
            score.name = "score";

            for (const auto &row : rows)
            {
                if (max_candidates_ > 0 && static_cast<int>(msg.points.size()) >= max_candidates_)
                    break;
                geometry_msgs::Point32 p;
                p.x = row.cur_pt.x;
                p.y = row.cur_pt.y;
                p.z = 0.0f;
                msg.points.push_back(p);
                prev_u.values.push_back(row.prev_pt.x);
                prev_v.values.push_back(row.prev_pt.y);
                score.values.push_back(row.score);
            }

            msg.channels.push_back(prev_u);
            msg.channels.push_back(prev_v);
            msg.channels.push_back(score);
            pub_.publish(msg);
            ROS_WARN_THROTTLE(2.0, "published %lu ONNX recovery candidates from %lld raw matches at %.6f, inference %.1f ms, request_lost=%lu",
                              msg.points.size(), static_cast<long long>(m_count), header.stamp.toSec(), dt_ms, lost_prev_pts.size());
        }
        catch (const Ort::Exception &e)
        {
            ROS_WARN("ONNX inference failed: %s", e.what());
        }
    }
#endif

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        frame_index_++;

        cv::Mat cur_img;
        try
        {
            cur_img = imageFromMsg(msg);
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_WARN("failed to decode image: %s", e.what());
            return;
        }

        if (cur_img.empty())
            return;

        cacheImage(msg->header.stamp.toSec(), cur_img);

        if (backend_ == "fake_grid")
        {
            if (frame_index_ % publish_every_n_ != 0)
                return;
            publishFakeGrid(msg->header, cur_img);
        }
#ifdef ENABLE_ONNXRUNTIME
        else if (backend_ == "onnx")
        {
            if (request_only_)
                return;
            if (!prev_img_.empty() && frame_index_ % publish_every_n_ == 0)
                publishOnnxMatches(msg->header, prev_img_, cur_img);
            prev_img_ = cur_img.clone();
        }
#endif
    }

    void cacheImage(double t, const cv::Mat &img)
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        image_cache_[t] = img.clone();
        while (!image_cache_.empty() && image_cache_.begin()->first < t - image_cache_sec_)
            image_cache_.erase(image_cache_.begin());
    }

    bool findFramePair(double t, cv::Mat &prev_img, cv::Mat &cur_img)
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (image_cache_.size() < 2)
            return false;
        auto cur_it = image_cache_.lower_bound(t);
        if (cur_it == image_cache_.end())
            cur_it = std::prev(image_cache_.end());
        if (cur_it != image_cache_.begin())
        {
            auto prev_candidate = std::prev(cur_it);
            if (std::fabs(prev_candidate->first - t) < std::fabs(cur_it->first - t))
                cur_it = prev_candidate;
        }
        if (std::fabs(cur_it->first - t) > request_time_tolerance_)
            return false;
        if (cur_it == image_cache_.begin())
            return false;
        auto prev_it = std::prev(cur_it);
        prev_img = prev_it->second.clone();
        cur_img = cur_it->second.clone();
        return true;
    }

    void requestCallback(const sensor_msgs::PointCloudConstPtr &request_msg)
    {
        if (backend_ != "onnx")
            return;
#ifdef ENABLE_ONNXRUNTIME
        bool expected = false;
        if (!inference_busy_.compare_exchange_strong(expected, true))
            return;

        std::vector<cv::Point2f> lost_prev_pts;
        lost_prev_pts.reserve(request_msg->points.size());
        for (const auto &p : request_msg->points)
            lost_prev_pts.push_back(cv::Point2f(p.x, p.y));
        if (lost_prev_pts.empty())
        {
            inference_busy_ = false;
            return;
        }

        cv::Mat prev_img, cur_img;
        double t = request_msg->header.stamp.toSec();
        if (!findFramePair(t, prev_img, cur_img))
        {
            ROS_WARN_THROTTLE(1.0, "skip recovery request %.6f: matching image pair not in cache", t);
            inference_busy_ = false;
            return;
        }
        publishOnnxMatches(request_msg->header, prev_img, cur_img, lost_prev_pts);
        inference_busy_ = false;
#endif
    }

    ros::Subscriber sub_;
    ros::Subscriber sub_request_;
    ros::Publisher pub_;
    std::string image_topic_;
    std::string output_topic_;
    std::string backend_;
    std::string superpoint_onnx_;
    std::string lightglue_onnx_;
    std::string pipeline_onnx_;
    int publish_every_n_ = 1;
    int fake_grid_step_ = 5;
    int max_candidates_ = 20000;
    int frame_index_ = 0;
    int model_width_ = 0;
    int model_height_ = 0;
    double fake_motion_u_ = 0.0;
    double fake_motion_v_ = 0.0;
    double min_score_ = 0.0;
    bool use_cuda_ = false;
    bool request_only_ = true;
    int cuda_device_id_ = 0;
    double request_time_tolerance_ = 0.02;
    double request_prev_radius_ = 12.0;
    double request_distance_penalty_ = 0.03;
    double image_cache_sec_ = 5.0;
    cv::Mat prev_img_;
    std::mutex cache_mutex_;
    std::map<double, cv::Mat> image_cache_;
    std::atomic<bool> inference_busy_{false};

#ifdef ENABLE_ONNXRUNTIME
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char *> input_name_ptrs_;
    std::vector<const char *> output_name_ptrs_;
#endif
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lightglue_recovery_candidate_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LightGlueRecoveryCandidateNode node(nh, pnh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
