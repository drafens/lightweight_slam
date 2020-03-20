//
// Created by drafens on 1/28/20.
//
#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "common_include.h"
#include "frame.h"

class Dataset {
public:
    typedef shared_ptr<Dataset> Ptr;
    Dataset();
    
    Frame::Ptr nxtFrame();

    size_t img_index_;

private:
    string path_;
    vector<double> timestamps_;
    int N_;
};


#endif //MYSLAM_DATASET_H
