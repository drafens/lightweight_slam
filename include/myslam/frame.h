//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
#include "pose.h"
#include "camera.h"
#include "feature_point.h"

class Frame {
public:
    typedef shared_ptr<Frame> Ptr;
    static shared_ptr<Frame> CreatFrame(double timestamp, const Mat &left, const Mat &right);
    Frame(double timestamp, Mat left, Mat right);
    void setCamera(Camera::Ptr camera){camera_=move(camera);}

    void setKeyFrame();
    bool insertKeyFrame(const Frame::Ptr& keyframe);
    bool isInFrame(const Vector3d& pw);

    cv::Mat image_, image_right_;
    vector<shared_ptr<FeaturePoint>> feature_points_, feature_points_right_;
    vector<cv::KeyPoint> keypoints_;
    Mat descriptors_;
    Pose::Ptr pose_;
    unordered_map<size_t, SE3d> relative_pose_; // to last keyframe or to loop keyframe

    bool active_ = true; // active keyframe or (frame after newest keyframe)
    double timestamp_;
    double loop_score_ = 0.02; // for loop closure
    int N_ = 0; // inlier features num
    size_t id_ = id_nxt_++;
    size_t id_keyframe_ = -1; // -1 denote not keyframe
    //    mutex pose_mutex_;

private:
    Camera::Ptr camera_;
    int width_, height_;
    static size_t id_nxt_;
    static size_t id_keyframe_nxt_;
};

#endif //MYSLAM_FRAME_H
