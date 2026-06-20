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

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <execinfo.h>
#include <csignal>
#include <functional>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

struct RecoveryCandidate
{
    cv::Point2f prev_pt;
    cv::Point2f cur_pt;
    float score;
};

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    void configureRecoveryBridge(bool enable, int max_recoveries, double time_tolerance,
                                 double prev_match_radius, double min_dist_ratio);
    void configureRecoveryRequest(int min_tracks, double lost_ratio, double timeout_ms,
                                  double cooldown_ms, int min_signal_count,
                                  bool require_degradation_signal);
    void configureRecoveryDegradation(double max_mean_flow, double min_blur_score, double brightness_delta);
    void configureRecoveryGeometry(double max_flow_error, int min_flow_tracks);
    void setRecoveryRequestCallback(std::function<void(double,
                                                       const vector<cv::Point2f> &,
                                                       const vector<int> &,
                                                       const vector<int> &,
                                                       const vector<cv::Point2f> &)> callback);
    void setRecoveryCandidates(double timestamp, const vector<RecoveryCandidate> &candidates);
    int applyRecoveryCandidates(const vector<cv::Point2f> &raw_prev_pts,
                                const vector<int> &raw_ids,
                                const vector<int> &raw_track_cnt,
                                const vector<uchar> &status,
                                const vector<cv::Point2f> &active_prev_pts,
                                const vector<cv::Point2f> &active_flows);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;
    bool enable_recovery_bridge;
    int recovery_max_cnt;
    double recovery_time_tolerance;
    double recovery_prev_match_radius;
    double recovery_min_dist_ratio;
    int recovery_request_min_tracks;
    double recovery_request_lost_ratio;
    double recovery_request_timeout_ms;
    double recovery_request_max_mean_flow;
    double recovery_request_min_blur_score;
    double recovery_request_brightness_delta;
    double recovery_request_cooldown_ms;
    int recovery_request_min_signal_count;
    bool recovery_request_require_degradation_signal;
    double last_recovery_request_time;
    double recovery_max_flow_error;
    int recovery_min_flow_tracks;
    double prev_mean_intensity;
    double prev_blur_score;
    bool has_prev_image_quality;
    double recovery_candidate_time;
    vector<RecoveryCandidate> recovery_candidates;
    std::mutex recovery_mutex;
    std::condition_variable recovery_condition;
    std::function<void(double,
                       const vector<cv::Point2f> &,
                       const vector<int> &,
                       const vector<int> &,
                       const vector<cv::Point2f> &)> recovery_request_callback;
};
