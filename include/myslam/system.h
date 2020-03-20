//
// Created by drafens on 1/18/20.
//
#ifndef MYSLAM_SYSTEM_H
#define MYSLAM_SYSTEM_H

#include "common_include.h"
#include "camera.h"
#include "dataset.h"
#include "tracking.h"
#include "optimizing.h"
#include "loop_closure.h"
#include "map.h"
#include "mapper.h"
#include "viewer.h"
#include <ros/ros.h>

class System {
public:
    typedef shared_ptr<System> Ptr;
    System()= default;
    bool init(int argc, char **argv);
    bool run();
    bool saveTrajectory();

private:
    bool runStep();
    static void checkExit(int sig);

    Camera::Ptr camera_;
    Tracking::Ptr tracker_;
    Optimizing::Ptr optimizer_;
    LoopClosure::Ptr loop_closure_;
    Viewer::Ptr viewer_;
    Mapper::Ptr mapper_;
    Map::Ptr map_;
    Dataset::Ptr dataset_;
    int fps_ = 30; // 30 for default
    static bool running_;
};

#endif //MYSLAM_SYSTEM_H
