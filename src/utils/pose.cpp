//
// Created by drafens on 1/18/20.
//
#include "myslam/pose.h"
#include <g2o/types/slam3d/se3quat.h>

void Pose::setPose() {
    R_ = Eigen::Matrix3d::Identity();
    t_ = Vector3d::Zero();
    T_ = SE3d(R_, t_);
}

void Pose::setPose(const SE3d& T){
    R_ = T.rotationMatrix();
    t_ = T.translation();
    T_ = T;
}

void Pose::setPose(const g2o::SE3Quat& T) {
    R_ = T.rotation();
    t_ = T.translation();
    T_ = SE3d(R_, t_);
}

void Pose::setPose(const Mat& R, const Mat& t) {
    cv::cv2eigen(R, R_);
    cv::cv2eigen(t, t_);
    T_ = SE3d(R_, t_);
}

void Pose::setPose(const Eigen::Matrix3d& R, const Vector3d& t) {
    R_ = R;
    t_ = t;
    T_ = SE3d(R, t);
}

Mat Pose::R_cv() {
    Mat R;
    cv::eigen2cv(R_, R);
    return R;
}

Mat Pose::t_cv() {
    Mat t;
    cv::eigen2cv(t_, t);
    return t;
}

Pose::Pose() {
    setPose();
}

Pose::Pose(const SE3d &T) {
    setPose(T);
}

Pose::Pose(const Mat &R, const Mat &t) {
    setPose(R, t);
}

Pose::Pose(const Eigen::Matrix3d &R, const Vector3d &t) {
    setPose(R, t);
}
