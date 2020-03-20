//
// Created by drafens on 2/23/20.
//
#include "myslam/cv_ext.h"
#include "myslam/feature_point.h"
#include "myslam/mappoint.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

namespace cv {

    // compute Fundamental to outlier points
    bool ransac(cv::InputOutputArray _pts1, cv::InputOutputArray _pts2, double inlier_rate, int inlier_num){
        vector<uchar> status;
        Mat pts1 = _pts1.getMat();
        Mat pts2 = _pts2.getMat();
        Mat pts1_new, pts2_new;
        cv::findFundamentalMat(pts1, pts2, status, cv::FM_RANSAC);
        for (int i=0; i < status.size(); i++) {
            if (status[i]) {
                pts1_new.push_back(pts1.col(i));
                pts2_new.push_back(pts2.col(i));
            }
        }
        if(pts1_new.rows < inlier_num) return false;
        if(pts1_new.rows/1.0/pts1.cols < inlier_rate) return false;
        pts1 = pts1_new;
        pts2 = pts2_new;
        return true;
    }

    // update frame.keypoints_ and frame.descriptors_
    void computeDescriptors(const cv::Ptr<cv::FeatureDetector>& descriptor, const Frame::Ptr& frame) {
        for(int i=0; i < frame->feature_points_.size(); i++){
            auto kp = frame->feature_points_[i]->keypoint_;
            kp.class_id = i;
            frame->keypoints_.push_back(kp);
        }
        descriptor->compute(frame->image_, frame->keypoints_, frame->descriptors_);
        for(int i=0; i<frame->keypoints_.size(); i++){
            auto mp = frame->feature_points_[frame->keypoints_[i].class_id]->map_point_.lock();
            if(mp){
                mp->descriptors_ = frame->descriptors_.row(i).clone();
            }
        }
    }

    int computeDescriptorDistance(const Mat& a, const Mat& b){
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();
        int dist = 0;
        for(int i = 0; i < 8; i++, pa++, pb++) {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }
        return dist;
    }

    bool matchDescriptors(const cv::FlannBasedMatcher& matcher, const Frame::Ptr& frame1, const Frame::Ptr& frame2, vector<Point2d>& pts2d, vector<Point3d>& pts3d) {
        vector<cv::DMatch> matches;
        matcher.match(frame1->descriptors_, frame2->descriptors_, matches);
        if(matches.size() < frame1->descriptors_.rows/2) return false;
        double dist_min = min_element(matches.begin(), matches.end(), [](const cv::DMatch& m1, const cv::DMatch& m2){return m1.distance < m2.distance;})->distance;
        double dist_th = min(max(int(2 * dist_min), 30), 45);//30以下通常不会误匹配,45左右开始出现误匹配
        for (auto m : matches) {
            if(m.distance < dist_th) {
                auto fp1 = frame1->feature_points_[frame1->keypoints_[m.queryIdx].class_id];
                auto fp2 = frame2->feature_points_[frame2->keypoints_[m.trainIdx].class_id];
                auto mp2 = fp2->map_point_.lock();
                if(!mp2) continue;
                pts2d.push_back(fp1->keypoint_.pt);
                pts3d.push_back(toPoint(mp2->pos_));
            }
        }
        int match_th = 60;
        return pts3d.size() > match_th;
    }

    void drawLK(const Mat& image1, const Mat& image2, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, const vector<int>& outliers){
        cv::Mat image1_o, image2_o;
        cv::cvtColor(image1, image1_o, CV_GRAY2BGR);
        cv::cvtColor(image2, image2_o, CV_GRAY2BGR);
        vector<bool> flag(pts1.size(), true);
        for (auto o : outliers) flag[o] = false;
        for (size_t i = 0; i < flag.size(); i++){
            const int RADIUS = 2;
            if(flag[i]) {
                cv::circle(image1_o, pts1[i], RADIUS, CV_RGB(255, 0, 0), FILLED);
                cv::circle(image2_o, pts2[i], RADIUS, CV_RGB(255, 0, 0), FILLED);
                cv::line(image2_o, pts1[i], pts2[i], CV_RGB(0, 255, 0));
            }else{
                cv::circle(image1_o, pts1[i], RADIUS, CV_RGB(255, 128, 0), FILLED);
            }
        }
        cv::imshow("src", image1_o);
        cv::imshow("dest", image2_o);
        cv::waitKey(0);
    }

}