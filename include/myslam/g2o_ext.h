//
// Created by drafens on 1/13/20.
//
#ifndef MYSLAM_G2O_EXT_H
#define MYSLAM_G2O_EXT_H

#include "common_include.h"
#include "camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace g2o {

    class EdgeProjectXYZ2UVPoseOnly : public BaseUnaryEdge<2, Vector2d, VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeProjectXYZ2UVPoseOnly(Vector3d pos, Camera::Ptr camera) : point_(move(pos)), camera_(move(camera)) {}
        void computeError() override;
        void linearizeOplus() override;
        bool read(istream &is) override {}
        bool write(ostream &os) const override {}
    private:
        Vector3d point_;
        Camera::Ptr camera_;
    };

    class EdgeProjectXYZ2UVStereo : public BaseBinaryEdge<2, Vector2d, VertexSE3Expmap, VertexSBAPointXYZ> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeProjectXYZ2UVStereo(Camera::Ptr camera, bool is_on_right);
        void computeError() override;
        void linearizeOplus() override;
        bool read(istream &is) override {}
        bool write(ostream &os) const override {}
        number_t chi2() const override {return _error.dot(information()*_error)/(weight_*weight_);}
        void setWeight(int weight) {weight_ = weight;}

    private:
        Camera::Ptr camera_;
        SE3Quat pose_ext_ = SE3Quat();
        int weight_ = 1;
    };
}

#endif //MYSLAM_G2O_EXT_H
