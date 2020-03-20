//
// Created by drafens on 2/19/20.
//
#ifndef MYSLAM_LOOP_CLOSURE_H
#define MYSLAM_LOOP_CLOSURE_H

#include "common_include.h"
#include "frame.h"
#include "map.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <DBoW3/DBoW3.h>

class LoopClosure {
public:
    typedef shared_ptr<LoopClosure> Ptr;
    explicit LoopClosure(bool running);
    void close();
    void setMap(Map::Ptr map){map_=move(map);}
    void setCamera(Camera::Ptr camera){camera_=move(camera);}

    void updateMap(const Frame::Ptr& frame);

private:
    void ThreadLoop();
    void updateVoc();
    void checkLoop();
    bool verifyLoop(SE3d& relative, const Frame::Ptr& frame);

    cv::FlannBasedMatcher matcher_;
    Frame::Ptr frame_, frame_loop_;
    Map::Ptr map_;
    Camera::Ptr camera_;
    double min_score_;
    double max_score_;
    double score_factor_;

    DBoW3::Vocabulary vocabulary_;
    DBoW3::Database database_;
    DBoW3::BowVector bow_vector_last_;

    atomic<bool> running_{};
    condition_variable cond_map_update_;
    thread loop_thread_;
    mutex data_mutex_;
};


#endif //MYSLAM_LOOP_CLOSURE_H
