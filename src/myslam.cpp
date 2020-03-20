//
// Created by drafens on 1/28/20.
//
#include <ros/ros.h>
#include "myslam/system.h"

// TODO mapper 3D map represent
int main(int argc, char** argv){
    ros::init(argc, argv, _proj_name);
    ros::NodeHandle nh;

    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO);

    System::Ptr slam(new System);

    bool success = slam->init(argc, argv);
    assert(success);
    slam->run();
    slam->saveTrajectory();

    google::ShutdownGoogleLogging();
    return 0;
}

