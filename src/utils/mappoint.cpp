//
// Created by drafens on 1/18/20.
//
#include "myslam/mappoint.h"

size_t MapPoint::id_nxt_ = 0;

MapPoint::MapPoint(Vector3d pw)
    : id_(id_nxt_++), pos_(move(pw)), observation_times_(0){}

MapPoint::Ptr MapPoint::CreatMappoint(const Vector3d& pw) {
    MapPoint::Ptr mappoint(new MapPoint(pw));
    return mappoint;
}

void MapPoint::addObs(const shared_ptr<FeaturePoint>& fp) {
    unique_lock<mutex> lck(data_mutex_);
    observations_.push_back(fp);
    observation_times_ ++;
}

void MapPoint::removeObs(const shared_ptr<FeaturePoint>& fp) {
    unique_lock<mutex> lck(data_mutex_);
    for (auto it=observations_.begin(); it!=observations_.end(); it++) {
        if (it->lock() == fp){
            observations_.erase(it);
            fp->map_point_.reset();
            observation_times_ --;
            break;
        }
    }
}

list<weak_ptr<FeaturePoint>> MapPoint::getObs() {
    unique_lock<mutex> lck(data_mutex_);
    return observations_;
}
