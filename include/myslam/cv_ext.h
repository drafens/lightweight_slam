//
// Created by drafens on 2/23/20.
//
#ifndef MYSLAM_CV_EXT_H
#define MYSLAM_CV_EXT_H

#include "common_include.h"
#include "frame.h"
#include "opencv2/features2d/features2d.hpp"

namespace cv {
    bool ransac(cv::InputOutputArray _pts1, cv::InputOutputArray _pts2, double inlier_rate=0, int inlier_num=3);
    void computeDescriptors(const cv::Ptr<cv::FeatureDetector>& descriptor, const Frame::Ptr& frame);
    int computeDescriptorDistance(const Mat& a, const Mat& b);
    bool matchDescriptors(const cv::FlannBasedMatcher& matcher, const Frame::Ptr& frame1, const Frame::Ptr& frame2, vector<Point2d>& pts2d, vector<Point3d>& pts3d);
    void drawLK(const Mat& image1, const Mat& image2, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, const vector<int>& outliers=vector<int>());
}

#endif //MYSLAM_CV_EXT_H
