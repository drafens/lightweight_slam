//
// Created by drafens on 1/11/20.
//

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "common_include.h"
#include "config.h"

class Camera{
public:
    typedef shared_ptr<Camera> Ptr;
    Camera();
    Camera(double fx, double fy, double cx, double cy, double k1=0, double k2=0, double p1=0, double p2=0, double k3=0, double baseline=0);

    Vector3d world2camera(const Vector3d& pw, const SE3d& Tcw);
    Vector3d camera2world(const Vector3d& pc, const SE3d& Tcw);
    Vector2d camera2pixel(const Vector3d& pc);
    Vector3d pixel2camera(const Vector2d& pp, double depth=1);
    Vector3d pixel2camera(const Point2d& pp, double depth=1);
    Vector3d pixel2world (const Vector2d& pp, const SE3d& Tcw, double depth=1);
    Vector3d pixel2world (const Point2d& pp, const SE3d& Tcw, double depth=1);
    Vector2d world2pixel (const Vector3d& pw, const SE3d& Tcw);
    double computeDepth(double disparity);

    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_, k3_;
    double baseline_, focus_length_;
    Vector2d principal_point_;
    Mat K_, DistCoef_;
    SE3d right_; //Trl
};

#endif //MYSLAM_CAMERA_H
