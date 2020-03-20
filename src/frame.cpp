//
// Created by drafens on 1/18/20.
//

#include "myslam/frame.h"
#include "myslam/config.h"

size_t Frame::id_nxt_ = 0;
size_t Frame::id_keyframe_nxt_ = 0;

Frame::Frame(double timestamp, Mat left, Mat right)
:timestamp_(timestamp), image_(move(left)), image_right_(move(right)){
    pose_ = make_shared<Pose>();
    width_ = image_.cols;
    height_ = image_.rows;
}

shared_ptr<Frame> Frame::CreatFrame(double timestamp, const Mat &left, const Mat &right) {
    Frame::Ptr frame(new Frame(timestamp, left, right));
    return frame;
}

void Frame::setKeyFrame() {
    id_keyframe_ = id_keyframe_nxt_++;
    LOG(INFO) << "Set frame " << id_ << " as keyframe " << id_keyframe_;
}

bool Frame::insertKeyFrame(const Frame::Ptr& keyframe) {
    static int min_fpoints = Config::get<int>("Tracking.min_fpoints_keyframe");
    static auto max_rotation = Config::get<double>("Tracking.max_rotation_keyframe");
    static auto max_translation = Config::get<double>("Tracking.max_translation_keyframe");

    Sophus::Vector6d relative_pose = (pose_->T().inverse() * keyframe->pose_->T()).log();
    Vector3d trans = relative_pose.head<3>();
    Vector3d rot = relative_pose.tail<3>();
    if(N_<min_fpoints/* || rot.norm()>max_rotation || trans.norm()>max_translation*/) {
        setKeyFrame();
        return true;
    }
    return false;
}

bool Frame::isInFrame(const Vector3d& pw) {
    int expand = 0;
    auto pc = camera_->world2camera(pw, pose_->T());
    if(pc[2] < 0) return false;
    auto px = camera_->camera2pixel(pc);
    return px[0] > -expand && px[0] < width_ + expand
        && px[1] > -expand && px[1] < height_ + expand;
}
