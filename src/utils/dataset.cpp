//
// Created by drafens on 1/28/20.
//
#include "include/myslam/dataset.h"
#include "include/myslam/config.h"

#include <boost/format.hpp>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

Dataset::Dataset() {
    path_ = Config::get<string>("Path.dataset");
    img_index_ = 0;
    ifstream file(path_ + "times.txt");
    timestamps_.reserve(5000);
    while(!file.eof()){
        string s;
        double t;
        getline(file, s);
        if(!s.empty()){
            stringstream ss(s);
            ss >> t;
            timestamps_.push_back(t);
        }
    }
    N_ = timestamps_.size();
}

Frame::Ptr Dataset::nxtFrame() {
    boost::format fmt(path_ + "image_%d/%06d.png");
    Mat img_l, img_r;
    img_l = cv::imread((fmt % 0 % img_index_).str(), cv::IMREAD_GRAYSCALE);
    img_r = cv::imread((fmt % 1 % img_index_).str(), cv::IMREAD_GRAYSCALE);

    if(!img_l.data && !img_r.data){
        return nullptr;
    } else if(!img_l.data || !img_r.data){
        LOG(WARNING) << "cannot find image " << img_index_;
        return nxtFrame();
    }

//    Mat img_l_resized, img_r_resized;
//    cv::resize(img_l, img_l_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
//    cv::resize(img_r, img_r_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    auto frame = Frame::CreatFrame(timestamps_[img_index_], img_l, img_r);
    img_index_ ++;
    return frame;
}



