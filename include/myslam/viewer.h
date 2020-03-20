//
// Created by drafens on 1/31/20.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include "common_include.h"
#include "map.h"
#include "frame.h"
#include "camera.h"
#include <pangolin/pangolin.h>

class Viewer {
public:
    typedef shared_ptr<Viewer> Ptr;
    explicit Viewer(bool running);
    void close();

    void setMap(Map::Ptr map){map_=move(map);}
    void addFrame(Frame::Ptr frame);
    void updateMap();

private:
    void ThreadLoop();
    Mat plotPointsOnImage(bool is_left, const double* color);
    void drawMap();
    void moveCamera(pangolin::OpenGlRenderState& cam);
    void drawFrame(const Frame::Ptr& frame, const double* color);

    Map::Ptr map_;
    Frame::Ptr frame_last_ = nullptr, frame_ = nullptr;
    Camera::Ptr camera_;
    Map::MappointsType mappoint_list_active_;
    Map::KeyframesType keyframe_list_;
    Map::KeyframesType frame_list_active_;

    int width_;
    int height_;
    int image_width_=1024, image_height_=370;
    const double blue_[3] = {0, 0, 1};
    const double green_[3] = {0, 1, 0};
    const double red_[3] = {1, 0, 0};

    atomic<bool> running_{};
    condition_variable cond_map_update_;
    thread viewer_thread_;
    mutex data_mutex_;

};

#endif //MYSLAM_VIEWER_H
