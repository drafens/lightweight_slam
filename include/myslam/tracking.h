//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_TRACKING_H
#define MYSLAM_TRACKING_H

#include <opencv2/features2d/features2d.hpp>

#include "common_include.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "viewer.h"

class Optimizing;
class Mapper;
class Viewer;

class Tracking {
public:
    typedef shared_ptr<Tracking> Ptr;
    explicit Tracking(size_t max_frame=_max_value);
    ~Tracking() = default;
    enum TrackingState{
        INITIAL = -1,
        OK = 0,
        LOST = 1
    } state_ = INITIAL;
    void close();
    void setMap(Map::Ptr map){map_=move(map);}
    void setCamera(Camera::Ptr camera){camera_=move(camera);}
    void setViewer(shared_ptr<Viewer> viewer){viewer_=move(viewer);}
    void setOptimizer(shared_ptr<Optimizing> optimizer){optimizer_=move(optimizer);}
    void setLoopClosure(shared_ptr<LoopClosure> loop_closure){loop_closure_=move(loop_closure);};
    void setMapper(shared_ptr<Mapper> mapper){mapper_=move(mapper);}

    bool track(const Frame::Ptr& frame);
private:
    SE3d pose_relative_; // to last frame
    Camera::Ptr camera_;
    Map::Ptr map_;
    shared_ptr<Optimizing> optimizer_;
    shared_ptr<Viewer> viewer_;
    shared_ptr<Mapper> mapper_;
    cv::Ptr<cv::FeatureDetector> detector_;
    Frame::Ptr frame_, frame_last_;
    Frame::Ptr keyframe_; // last keyframe

    int num_feature_point_init_;
    double max_stereo_match_tan_;
    int num_tracking_lost_;
    size_t max_frame_;
    atomic<bool> running_{};

    // init stereo match
    bool stereoInit();

    // update keyframe_ and frame_last_ feature_point with LK method
    void trackLastFrameLK();

    // use g2o
    void estimatePose();

    // feature_point in left image
    void detectFeaturePoint();

    // find on right image use LK, not find will save a nullptr
    void findFeaturePointRight();

    // check and insert as keyframe to viewer and map
    void insertKeyFrame();
};


#endif //MYSLAM_TRACKING_H
