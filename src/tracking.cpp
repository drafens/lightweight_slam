//
// Created by drafens on 1/18/20.
//
#include "myslam/tracking.h"
#include "myslam/optimizing.h"
#include "myslam/mapper.h"
#include "myslam/mappoint.h"
#include "myslam/cv_ext.h"
#include <opencv2/opencv.hpp>

Tracking::Tracking(size_t max_frame){
    running_.store(true);
    int num_features = Config::get<int>("FeaturePoint.num_features");
    detector_ = cv::GFTTDetector::create(num_features, 0.01, 10);
    num_feature_point_init_ = Config::get<int>("Tracking.min_features_init");
    max_stereo_match_tan_ = Config::get<double>("Tracking.max_stereo_match_tan");
    num_tracking_lost_ = Config::get<int>("Tracking.num_tracking_lost");
    pose_relative_ = SE3d();
    max_frame_ = max_frame;
}

void Tracking::close() {
    LOG(WARNING) << "exit with " << frame_->id_+1 << " frames, " << map_->getAllKeyframes().size() << " keyframes, " << map_->getAllMappoints().size() << " mappoint.";
    running_.store(false);
}

bool Tracking::track(const Frame::Ptr& frame) {
    frame_ = frame;
    if (frame_->id_ > max_frame_) return false;
    frame_->setCamera(camera_);
    switch (state_) {
        case INITIAL:
            if(stereoInit()){
                LOG(WARNING) << "init success.";
                state_ = OK;
            }
            break;
        case OK:
            trackLastFrameLK();
            estimatePose();
            insertKeyFrame(); // contain map_, viewer_, optimizer_ updateMap
            break;
        case LOST:
            LOG(ERROR) << "lost...";
            return false;
    }
    // update norm frame
    map_->addFrame(frame_);
    viewer_->addFrame(frame_);
    frame_last_ = frame_;
    return running_.load();
}

// update keyframe
void Tracking::insertKeyFrame(){
    if(state_==LOST) return;
    if(state_!=INITIAL) {
        if (!frame_->insertKeyFrame(keyframe_)) return;
        detectFeaturePoint();
        findFeaturePointRight();
        map_->insertKeyframe(frame_);
    }
    mapper_->updateMap(frame_);
    optimizer_->updateMap(frame_);
    keyframe_ = frame_;
}

void Tracking::trackLastFrameLK(){
    vector<cv::Point2f> pts_last, pts;
    {
        unique_lock<mutex> lck(map_->mtx_active_point_and_pose_);
        if(frame_last_){
            frame_->pose_->setPose(pose_relative_ * frame_last_->pose_->T());
        }
        for (const auto &fp : frame_last_->feature_points_) {
            pts_last.push_back(fp->keypoint_.pt);
            auto mp = fp->map_point_.lock();
            if (mp) {
                auto px = camera_->world2pixel(mp->pos_, frame_->pose_->T());
                pts.emplace_back(px[0], px[1]);
            } else {
                pts.push_back(fp->keypoint_.pt);
            }
        }
        lck.unlock();
    }

    vector<uchar> lk_status, ransac_status;
    Mat error;
    int level = 5;
    auto size = cv::Size(11, 11);
    auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01);
    cv::calcOpticalFlowPyrLK(frame_last_->image_, frame_->image_, pts_last, pts, lk_status, error, size, level, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

    // check by ransac
    cv::findFundamentalMat(pts_last, pts, ransac_status, cv::FM_RANSAC);

    vector<int> outlier_lk, outlier_ransac;
    for (int i = 0; i < lk_status.size(); ++i) {
        if(!lk_status[i]) { // not matched by lk
            outlier_lk.push_back(i);
            continue;
        }
        if(!ransac_status[i]) { // matched error or dynamic
            outlier_ransac.push_back(i);
            continue;
        }
        cv::KeyPoint kp(pts[i], 7);
        FeaturePoint::Ptr fp(new FeaturePoint(frame_, kp));
        frame_->feature_points_.push_back(fp);
        auto mp = frame_last_->feature_points_[i]->map_point_.lock();
        if(!mp) continue;
        fp->map_point_ = mp;
    }
    LOG(INFO) << "frame " << frame_->id_ << " outlier " << outlier_lk.size() << " " << outlier_ransac.size() << " in LK.";
    if(frame_->feature_points_.size() < num_tracking_lost_) state_ = LOST;
    frame_->N_ = frame_->feature_points_.size();
    LOG(INFO) << "frame " << frame_->id_ << " track " << frame_->N_ << " points by LK.";
//    if(outlier_lk.size()+outlier_ransac.size()>100) cv::drawLK(frame_last_->image_, frame_->image_, pts_last, pts, outlier_lk+outlier_ransac);
}

void Tracking::estimatePose(){
    if(state_==LOST) return;

    Optimizing::optimizeOnlyPose(frame_, map_, camera_);
    if(frame_->N_ < num_tracking_lost_) state_ = LOST;
    LOG(INFO) << "frame " << frame_->id_ << " track " << frame_->N_ << " points after g2o.";
    pose_relative_ = relativeSE3(frame_last_->pose_->T(), frame_->pose_->T()); // to last frame
    frame_->relative_pose_.insert(make_pair(keyframe_->id_keyframe_, relativeSE3(keyframe_->pose_->T(), frame_->pose_->T()))); // to last keyframe
}

void Tracking::detectFeaturePoint(){
    vector<cv::KeyPoint> keypoints;
    frame_->feature_points_.clear();
    frame_->feature_points_right_.clear();
    detector_->detect(frame_->image_, keypoints);
    for (const auto& kp : keypoints) {
        frame_->feature_points_.push_back(make_shared<FeaturePoint>(frame_, kp));
    }
    frame_->N_ = frame_->feature_points_.size();
}

void Tracking::findFeaturePointRight(){
    vector<cv::Point2f> pts_l, pts_r;
    {
        unique_lock<mutex> lck(map_->mtx_active_point_and_pose_);
        for (const auto &fp : frame_->feature_points_) {
            pts_l.push_back(fp->keypoint_.pt);
            auto mp = fp->map_point_.lock();
            if (mp) {
                auto px = camera_->world2pixel(mp->pos_, camera_->right_*frame_->pose_->T());
                pts_r.emplace_back(Point2d(px[0], px[1]));
            } else {
                pts_r.push_back(fp->keypoint_.pt);
            }
        }
        lck.unlock();
    }

    vector<uchar> status;
    Mat error;
    int level = 5;
    auto size = cv::Size(11, 11);
    auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01);
    cv::calcOpticalFlowPyrLK(frame_->image_, frame_->image_right_, pts_l, pts_r, status, error, size, level, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

    // find feature with disparity only nearly horizontal, tan theta is small
    double tan=0;
    vector<double> tans;
    int N = 0;
    for (int i = 0; i < status.size(); ++i) {
        if(!status[i]) {
            tans.push_back(_max_value);
            continue;
        }
        Point2d d = pts_l[i]-pts_r[i];
        if(d.x==0) {
            tans.push_back(_max_value);
            continue;
        }
        double t = d.y/d.x;
        if(1<t || t<-1) {
            tans.push_back(t);
            continue;
        }
        tans.push_back(t);
        tan += t;
        N ++;
    }
    if(N<=0) {
        frame_->N_ = N;
        return;
    }
    tan = tan / N;
    if (1<tan || tan<-1) {
        frame_->N_ = 0;
        return;
    }
    LOG(INFO) << "mean tan is " << tan;

    N = 0;
    for (size_t i = 0; i < status.size(); i++) {
        bool good = true;
        if (!status[i]) good = false;
        double t = tans[i] - tan;
        if(max_stereo_match_tan_<t || t<-max_stereo_match_tan_) good = false;

        if (good){
            cv::KeyPoint kp(pts_r[i], 7);
            FeaturePoint::Ptr fp(new FeaturePoint(frame_, kp));
            fp->is_on_right_ = true;
            frame_->feature_points_right_.push_back(fp);
            N ++;
        } else{
            frame_->feature_points_right_.push_back(nullptr);
        }
    }

//    drawLK(keyframe_->image_, keyframe_->image_right_, pts_l, pts_r, keyframe_->feature_points_right_);
    frame_->N_ = N;
    LOG(INFO) << "keyframe " << frame_->id_ << " find "<< frame_->N_ << " stereo feature point.";
}

bool Tracking::stereoInit() {
    frame_->pose_->setPose();
    detectFeaturePoint();
    findFeaturePointRight();
    if(frame_->N_ < num_feature_point_init_) return false;
    frame_->setKeyFrame();
    map_->initMap(frame_);
    insertKeyFrame();
    return true;
}

