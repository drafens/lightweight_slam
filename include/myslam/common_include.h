//
// Created by drafens on 1/11/20.
//
#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

// const val
#define _max_value 99999
#define _proj_name "Light_Weight_SLAM"

// for stl
#include <iostream>
#include <memory>
#include <string>
#include <iomanip>
#include <set>
#include <vector>
#include <list>
#include <unordered_map>
#include <map>
//for thread
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
using namespace std;

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using Sophus::SO3d;
using Sophus::SE3d;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
using cv::Mat;
using cv::Point3d;
using cv::Point2d;

#include <glog/logging.h>

#include "covert.h"

#endif //MYSLAM_COMMON_INCLUDE_H
