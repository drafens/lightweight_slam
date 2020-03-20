//
// Created by drafens on 1/11/20.
//
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "common_include.h"

class Config{
public:
    ~Config();
    static void setParaFile();
    template<typename T> static T get(const std::string& key);

private:
    Config()= default;
    static shared_ptr<Config> config_;
    cv::FileStorage file_;
};


////src
template<typename T>
T Config::get(const std::string &key) {
    auto val = T(config_->file_[key]);
    LOG(WARNING) << "config..." << key << " = " << val;
    return val;
}

#endif //MYSLAM_CONFIG_H
