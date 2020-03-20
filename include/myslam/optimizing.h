//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_OPTIMIZING_H
#define MYSLAM_OPTIMIZING_H

#include "common_include.h"
#include "g2o_ext.h"
#include "camera.h"
#include "map.h"
#include "loop_closure.h"
#include <opencv2/features2d/features2d.hpp>

class Frame;
class Viewer;

class Optimizing {
public:
    typedef shared_ptr<Optimizing> Ptr;
    explicit Optimizing(bool running);
    void close();
    void setMap(Map::Ptr map){map_=move(map);}
    void setCamera(Camera::Ptr camera){camera_=move(camera);}
    void setLoopClosure(LoopClosure::Ptr loop_closure){loop_closure_=move(loop_closure);}

    void updateMap(const Frame::Ptr& frame);
    static void optimizeOnlyPose(const shared_ptr<Frame>& frame, const Map::Ptr& map, const Camera::Ptr& camera); // for tracking
    static void optimizePoseGraph(int id, unordered_map<size_t, SE3d>& loop_list, const Map::Ptr& map, const Camera::Ptr& camera); // for loop closure
private:
    void ThreadLoop();
    void matchLastKeyframe(unordered_map<size_t, size_t>& match);
    void optimizePosePointKeyframe(unordered_map<size_t, size_t> match); // for local map

    cv::Ptr<cv::FeatureDetector> descriptor_;
    cv::FlannBasedMatcher flann_;
    Camera::Ptr camera_;
    Map::Ptr map_;
    Frame::Ptr frame_;
    LoopClosure::Ptr loop_closure_;
    Map::KeyframesType keyframe_list_active_;
    Map::MappointsType mappoint_list_active_;

    atomic<bool> running_{}; // if running = false, this thread is still running for compute keyframe descriptors.
    atomic<bool> closing_{}; // when slam is shutdown, close all the thread
    condition_variable cond_map_update_;
    thread optimizing_thread_;
    mutex data_mutex_;
    static bool discard_optmize_data_;
};


#endif //MYSLAM_OPTIMIZING_H
