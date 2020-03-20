//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_POSE_H
#define MYSLAM_POSE_H

#include "common_include.h"
#include <g2o/types/slam3d/se3quat.h>

class Pose {
public:
    typedef shared_ptr<Pose> Ptr;
    Pose();
    explicit Pose(const SE3d& T);
    Pose(const Mat& R, const Mat& t);
    Pose(const Eigen::Matrix3d& R, const Vector3d& t);

    void setPose();
    void setPose(const SE3d& T);
    void setPose(const g2o::SE3Quat& T);
    void setPose(const Mat& R, const Mat& t);
    void setPose(const Eigen::Matrix3d& R, const Vector3d& t);

    SE3d T() {return T_;}
    Matrix3d R() {return R_;}
    Eigen::Quaterniond Q() {return Eigen::Quaterniond(R_);}
    Vector3d t() {return t_;}
    SE3d Tw() {return T_.inverse();}
    Matrix3d Rw() {return R_.inverse();}
    Vector3d tw() {return -Rw()*t_;}
    Eigen::Quaterniond Qw() {return Eigen::Quaterniond(Rw());}
    Mat R_cv();
    Mat t_cv();
private:
    SE3d T_;
    Matrix3d R_;
    Vector3d t_;
};


#endif //MYSLAM_POSE_H
