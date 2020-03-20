//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_FEATURE_POINT_H
#define MYSLAM_FEATURE_POINT_H

#include "common_include.h"

struct Frame;
struct MapPoint;

struct FeaturePoint : enable_shared_from_this<FeaturePoint>{
public:
    typedef shared_ptr<FeaturePoint> Ptr;
    FeaturePoint(const shared_ptr<Frame>& frame, cv::KeyPoint kp)
            : frame_(frame), keypoint_(move(kp)) {}

    weak_ptr<Frame> frame_;
    weak_ptr<MapPoint> map_point_;
    cv::KeyPoint keypoint_;
//    Vector3d pc_ = Vector3d::Zero(); // 只有存在mappoint时才有效
    bool is_outlier_ = false;
    bool is_on_right_ = false;
};

#endif //MYSLAM_FEATURE_POINT_H
