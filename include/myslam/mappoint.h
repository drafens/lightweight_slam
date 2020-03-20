//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "common_include.h"
#include "feature_point.h"

struct MapPoint {
public:
    typedef shared_ptr<MapPoint> Ptr;
    static MapPoint::Ptr CreatMappoint(const Vector3d& pw);

    void addObs(const shared_ptr<FeaturePoint>& fp);
    void removeObs(const shared_ptr<FeaturePoint>& fp);
    list<weak_ptr<FeaturePoint>> getObs();

    size_t id_;
    size_t id_keyframe_ = -1;
    Vector3d pos_;
    Mat descriptors_;
    bool is_outlier_ = false;
    int observation_times_ = 0;
    double score_ = 1;
    double uncertainty_ = 1;

private:
    explicit MapPoint(Vector3d p);
    static std::size_t id_nxt_;
    list<weak_ptr<FeaturePoint>> observations_; // observations by all feature point in keyframe
    mutex data_mutex_;
};


#endif //MYSLAM_MAPPOINT_H
