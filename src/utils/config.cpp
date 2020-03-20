//
// Created by drafens on 1/13/20.
//
#include "myslam/config.h"

#include <ros/package.h>

shared_ptr<Config> Config::config_ = nullptr;

Config::~Config() {
    if(file_.isOpened()) file_.release();
}

void Config::setParaFile() {
    string filename = ros::package::getPath("myslam") + "/config/kitti.yaml";
    if(config_ == nullptr){
        config_ = shared_ptr<Config>(new Config);
        config_->file_.open(filename, cv::FileStorage::READ);
        if (!config_->file_.isOpened()){
            LOG(ERROR) << filename << " does not exist."<< std::endl;
            config_->file_.release();
            exit(1);
        }
    }
}

