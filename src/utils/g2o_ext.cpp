//
// Created by drafens on 1/13/20.
//
#include "myslam/g2o_ext.h"

namespace g2o {

    void EdgeProjectXYZ2UVPoseOnly::computeError() {
        const auto *v = dynamic_cast<VertexSE3Expmap*>(_vertices[0]);
        _error = _measurement - camera_->camera2pixel(v->estimate().map(point_));
    }

    void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
        auto *v = dynamic_cast<VertexSE3Expmap*>(_vertices[0]);
        const SE3Quat& T(v->estimate());
        Vector3d pc = T.map(point_);
        double x = pc[0];
        double y = pc[1];
        double z = pc[2];
        double z_inv = 1.0 / (z + 1e-18);
        double z_inv2 = z_inv * z_inv;

        _jacobianOplusXi(0, 0) = camera_->fx_ * x * y * z_inv2;
        _jacobianOplusXi(0, 1) = -camera_->fx_ - camera_->fx_ * x * x * z_inv2;
        _jacobianOplusXi(0, 2) = camera_->fx_* y * z_inv;
        _jacobianOplusXi(0, 3) = -camera_->fx_ * z_inv;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = camera_->fx_ * x * z_inv2;

        _jacobianOplusXi(1, 0) = camera_->fy_ + camera_->fy_ * y * y * z_inv2;
        _jacobianOplusXi(1, 1) = -camera_->fy_ * x * y * z_inv2;
        _jacobianOplusXi(1, 2) = -camera_->fy_ * x * z_inv;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -camera_->fy_ * z_inv;
        _jacobianOplusXi(1, 5) = camera_->fy_ * y * z_inv2;
    }


    EdgeProjectXYZ2UVStereo::EdgeProjectXYZ2UVStereo(Camera::Ptr camera, bool is_on_right)
    :camera_(move(camera)) {
        if (is_on_right) {
            pose_ext_ = SE3Quat(camera_->right_.rotationMatrix(), camera_->right_.translation());
        }
    }

    void EdgeProjectXYZ2UVStereo::computeError() {
        const auto *v0 = dynamic_cast<VertexSE3Expmap*>(_vertices[0]);
        const auto *v1 = dynamic_cast<VertexSBAPointXYZ*>(_vertices[1]);
        Vector3d pc = (pose_ext_*v0->estimate()).map(v1->estimate());
        _error = _measurement - camera_->camera2pixel(pc);
        _error *= weight_;
    }

    void EdgeProjectXYZ2UVStereo::linearizeOplus() {
        const auto *v0 = dynamic_cast<VertexSE3Expmap*>(_vertices[0]);
        const auto *v1 = dynamic_cast<VertexSBAPointXYZ*>(_vertices[1]);

        SE3Quat T_ext = v0->estimate()*pose_ext_;
        Vector3d pc = T_ext.map(v1->estimate());
        double x = pc[0];
        double y = pc[1];
        double z = pc[2];
        double z_inv = 1.0 / (z + 1e-18);
        double z_inv2 = z_inv * z_inv;

        _jacobianOplusXi(0, 0) = -camera_->fx_ * z_inv;
        _jacobianOplusXi(0, 1) = 0;
        _jacobianOplusXi(0, 2) = camera_->fx_* x * z_inv2;
        _jacobianOplusXi(0, 3) = camera_->fx_ * x *y * z_inv2;
        _jacobianOplusXi(0, 4) = -camera_->fx_ - camera_->fx_ * x * x * z_inv2;
        _jacobianOplusXi(0, 5) = camera_->fx_ * y * z_inv;

        _jacobianOplusXi(1, 0) = 0;
        _jacobianOplusXi(1, 1) = -camera_->fy_ * z_inv;
        _jacobianOplusXi(1, 2) = camera_->fy_ * y * z_inv2;
        _jacobianOplusXi(1, 3) = camera_->fy_ + camera_->fy_ * y * y * z_inv2;
        _jacobianOplusXi(1, 4) = -camera_->fy_ * x * y * z_inv2;
        _jacobianOplusXi(1, 5) = -camera_->fy_ * x * z_inv;

        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T_ext.rotation().matrix();
    }

}
