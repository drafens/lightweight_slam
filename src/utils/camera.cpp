//
// Created by drafens on 1/13/20.
//
#include "myslam/camera.h"

Camera::Camera(){
    fx_ = Config::get<double>("Camera.fx");
    fy_ = Config::get<double>("Camera.fy");
    cx_ = Config::get<double>("Camera.cx");
    cy_ = Config::get<double>("Camera.cy");
    k1_ = Config::get<double>("Camera.k1");
    k2_ = Config::get<double>("Camera.k2");
    p1_ = Config::get<double>("Camera.p1");
    p2_ = Config::get<double>("Camera.p2");
    k3_ = Config::get<double>("Camera.k3");
    baseline_ = Config::get<double>("Camera.baseline");
    K_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    DistCoef_ = (cv::Mat_<double>(4, 1) << k1_, k2_, p1_, p2_, k3_);
    focus_length_ = (fx_ + fy_) / 2;
    principal_point_ = Eigen::Vector2d(cx_, cy_);
    right_ = SE3d(SO3d(), Vector3d(-baseline_, 0, 0));
}

Camera::Camera(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3, double baseline)
:fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), p1_(p1), p2_(p2), k3_(k3), baseline_(baseline){
    K_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    DistCoef_ = (cv::Mat_<double>(4, 1) << k1_, k2_, p1_, p2_, k3_);
    focus_length_ = (fx_ + fy_) / 2;
    principal_point_ = Eigen::Vector2d(cx_, cy_);
    right_ = SE3d(SO3d(), Vector3d(-baseline_, 0, 0));
}


#pragma clang diagnostic push
#pragma ide diagnostic ignored "MemberFunctionCanBeStatic"
Vector3d Camera::world2camera(const Vector3d &pw, const SE3d &Tcw) {
    return Tcw * pw;
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma ide diagnostic ignored "MemberFunctionCanBeStatic"
Vector3d Camera::camera2world(const Vector3d &pc, const SE3d &Tcw) {
    return Tcw.inverse() * pc;
}
#pragma clang diagnostic pop

Vector2d Camera::camera2pixel(const Vector3d &pc) {
    Vector2d pp(fx_ * pc ( 0,0 ) / pc ( 2,0 ) + cx_,
                fy_ * pc ( 1,0 ) / pc ( 2,0 ) + cy_);
    return pp;
}

Vector3d Camera::pixel2camera(const Vector2d &pp, double depth) {
    Vector3d pc((pp(0, 0) - cx_) / fx_,
                (pp(1, 0) - cy_) / fy_,
                1
    );
    return pc * depth;
}

Vector3d Camera::pixel2world(const Vector2d &pp, const SE3d &Tcw, double depth) {
    return camera2world(pixel2camera(pp, depth), Tcw);
}

Vector2d Camera::world2pixel(const Vector3d &pw, const SE3d &Tcw) {
    return camera2pixel(world2camera(pw, Tcw));
}

Vector3d Camera::pixel2camera(const Point2d &pp, double depth) {
    Vector2d v(pp.x, pp.y);
    return pixel2camera(v, depth);
}

Vector3d Camera::pixel2world(const Point2d &pp, const SE3d &Tcw, double depth) {
    Vector2d v(pp.x, pp.y);
    return pixel2world(v, Tcw, depth);
}

double Camera::computeDepth(double disparity) {
    if(disparity==0) return _max_value;
    return focus_length_ * baseline_ / disparity;
}
