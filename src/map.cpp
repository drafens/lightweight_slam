//
// Created by drafens on 1/18/20.
//

#include "myslam/map.h"
#include "myslam/config.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"
#include <opencv2/opencv.hpp>

Map::Map() {
    num_active_keyframe_ = Config::get<int>("Mapping.num_keyframe");
    max_disparity_ = Config::get<double>("Tracking.max_disparity");
    min_disparity_ = Config::get<double>("Tracking.min_disparity");
    min_frame_distance_ = Config::get<double>("Mapping.min_frame_distance");
    keyframe_ = nullptr;
}

void Map::initMap(const Frame::Ptr& frame){
    keyframe_ = frame;
    keyframe_list_.insert(make_pair(frame->id_keyframe_, keyframe_));
    keyframe_list_active_.insert(make_pair(frame->id_keyframe_, keyframe_));
    insertAllMappoint(keyframe_);
    LOG(INFO) << "init map by " << getAllMappoints().size() << " map points.";
}

Map::MappointsType Map::getAllMappoints() {
    unique_lock<mutex> lck(data_mutex_);
    return mappoint_list_;
}

Map::MappointsType Map::getActiveMappoints() {
    unique_lock<mutex> lck(data_mutex_);
    return mappoint_list_active_;
}

Map::KeyframesType Map::getAllKeyframes() {
    unique_lock<mutex> lck(data_mutex_);
    return keyframe_list_;
}

Map::KeyframesType Map::getActiveKeyframes() {
    unique_lock<mutex> lck(data_mutex_);
    return keyframe_list_active_;
}

Map::KeyframesType Map::getActiveFrames() {
    unique_lock<mutex> lck(data_mutex_);
    return frame_list_active_;
}

vector<pair<size_t, SE3d>> Map::getAllFramePoses() {
    unique_lock<mutex> lck(data_mutex_);
    return frame_pose_list_;
}

void Map::checkMappoints() {
    for (auto it=mappoint_list_active_.begin();it!=mappoint_list_active_.end();) {
        if(it->second->observation_times_ == 0){
            it = mappoint_list_active_.erase(it);
        } else{
            it ++;
        }
    }
}

void Map::checkKeyframes() {
    if (keyframe_ == nullptr) return;
    if (keyframe_list_active_.size() <= num_active_keyframe_) return;
    auto Twc = keyframe_->pose_->T().inverse();
    double max_dis = 0, min_dis = _max_value;
    size_t max_dis_id=keyframe_list_active_.size()-1, min_dis_id=0;
    for (auto& f : keyframe_list_active_) {
        if (f.second == keyframe_) continue;
        auto distance = (f.second->pose_->T() * Twc).log().norm();
        if (distance > max_dis){
            max_dis = distance;
            max_dis_id = f.first;
        }
        if(distance < min_dis){
            min_dis = distance;
            min_dis_id = f.first;
        }
    }

    Frame::Ptr frame_remove = nullptr;
    if (min_dis < min_frame_distance_){
        frame_remove = keyframe_list_.at(min_dis_id);
    }else{
        frame_remove = keyframe_list_.at(max_dis_id);
    }

    frame_remove->active_ = false;
    keyframe_list_active_.erase(frame_remove->id_keyframe_);
    for (const auto& f : frame_remove->feature_points_+frame_remove->feature_points_right_) {
        if(f == nullptr) continue;
        auto mp = f->map_point_.lock();
        if(mp){
            mp->removeObs(f);
        }
    }
    checkMappoints();
}

void Map::insertAllMappoint(const Frame::Ptr& frame){
    if (frame->feature_points_.size() != frame->feature_points_right_.size()) return;
    for (size_t i = 0; i < frame->feature_points_.size(); ++i) {
        auto f1 = frame->feature_points_[i];
        auto f2 = frame->feature_points_right_[i];
        if (nullptr == f1 || nullptr == f2) continue;
        if (!f1->map_point_.expired()) continue;

        double disparity = cv::norm(f1->keypoint_.pt - f2->keypoint_.pt);
        if(disparity<min_disparity_ || disparity>max_disparity_) continue;
        double depth = camera_->computeDepth(disparity);
        Vector3d pw = camera_->pixel2world(Point2d(f1->keypoint_.pt), frame->pose_->T(), depth);

        double score = disparity*disparity-0.25; // the uncertainty fb/score(m)

        MapPoint::Ptr mp = MapPoint::CreatMappoint(pw);
        mp->id_keyframe_ = frame->id_keyframe_;
        mp->score_ = score;
        mp->addObs(frame->feature_points_[i]);
        mp->addObs(frame->feature_points_right_[i]);
        frame->feature_points_[i]->map_point_ = mp;
        frame->feature_points_right_[i]->map_point_ = mp;
        insertMappoint(mp);
    }
}

void Map::insertKeyframe(const Frame::Ptr& frame) {
    keyframe_ = frame;
    if(keyframe_list_.find(frame->id_keyframe_) == keyframe_list_.end()){
        keyframe_list_.insert(make_pair(keyframe_->id_keyframe_, keyframe_));
        keyframe_list_active_.insert(make_pair(keyframe_->id_keyframe_, keyframe_));
    } else{
        keyframe_list_[frame->id_keyframe_] = keyframe_;
        keyframe_list_active_[frame->id_keyframe_] = keyframe_;
    }

    if(keyframe_list_active_.size() > num_active_keyframe_){
        checkKeyframes();
    }
    insertAllMappoint(keyframe_);
    LOG(INFO) << "local map has " << getActiveMappoints().size() << "/" << getAllMappoints().size() << " map points now.";
}

void Map::insertMappoint(const MapPoint::Ptr& mappoint) {
    if(mappoint_list_.find(mappoint->id_) == mappoint_list_.end()){
        mappoint_list_.insert(make_pair(mappoint->id_, mappoint));
        mappoint_list_active_.insert(make_pair(mappoint->id_, mappoint));
    } else{
        mappoint_list_[mappoint->id_] = mappoint;
        mappoint_list_active_[mappoint->id_] = mappoint;
    }
}

void Map::addFrame(const Frame::Ptr &frame) {
    frame_ = frame;
    if (frame_->id_keyframe_ == -1) {
        frame_list_active_.insert(make_pair(frame_->id_, frame_));
        frame_pose_list_.emplace_back(frame_->id_keyframe_, frame_->relative_pose_.at(keyframe_->id_keyframe_));
    } else {
        for(const auto& f : frame_list_active_){
            f.second->active_ = false;
        }
        frame_list_active_.clear();
        frame_pose_list_.emplace_back(frame_->id_keyframe_, SE3d());
    }
}

void Map::fuseMappoint(const unordered_map<size_t, size_t>& match){
    unique_lock<mutex> lck(mtx_active_point_and_pose_);
    for (auto m : match) {
        auto mp1 = mappoint_list_.at(m.first);
        auto mp2 = mappoint_list_.at(m.second); // to remove
//        if ((mp1->pos_ - mp2->pos_).norm() > 15) continue;
        mp1->pos_ = (mp1->score_ * mp1->pos_ + mp2->score_ * mp2->pos_) / (mp1->score_ + mp2->score_);
        mp1->score_ = sqrt(mp1->score_ * mp1->score_ + mp2->score_ * mp2->score_);
        mp1->id_keyframe_ = mp2->id_keyframe_;

        for (const auto &ob : mp2->getObs()) {
            auto fp = ob.lock();
            mp2->removeObs(fp);
            mp1->addObs(fp);
            fp->map_point_ = mp1;
        }

        mappoint_list_active_.erase(m.second);
        mappoint_list_.erase(m.second);
        mp2.reset();
    }
}
