//
// Created by drafens on 1/18/20.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "common_include.h"
#include "mappoint.h"
#include "camera.h"
#include "frame.h"

class Map {
public:
    typedef shared_ptr<Map> Ptr;
    typedef unordered_map<size_t, MapPoint::Ptr> MappointsType;
    typedef unordered_map<size_t, Frame::Ptr> KeyframesType;
    Map();

    void initMap(const Frame::Ptr& frame);
    void setCamera(Camera::Ptr camera){camera_=move(camera);}
    MappointsType getAllMappoints();
    MappointsType getActiveMappoints();
    KeyframesType getAllKeyframes();
    KeyframesType getActiveKeyframes();
    KeyframesType getActiveFrames();
    vector<pair<size_t, SE3d>> getAllFramePoses();
    void checkMappoints();
    void checkKeyframes();
    void insertKeyframe(const Frame::Ptr& frame);
    void addFrame(const Frame::Ptr& frame);
    void insertMappoint(const MapPoint::Ptr& mappoint);
    void fuseMappoint(const unordered_map<size_t, size_t>& match);

    mutex mtx_active_point_and_pose_;
private:
    void insertAllMappoint(const Frame::Ptr& frame);

    MappointsType mappoint_list_;
    MappointsType mappoint_list_active_;
    KeyframesType keyframe_list_;
    KeyframesType keyframe_list_active_;
    KeyframesType frame_list_active_;
    vector<pair<size_t, SE3d>> frame_pose_list_;
    Camera::Ptr camera_;

    double min_frame_distance_;
    double min_disparity_, max_disparity_;
    int num_active_keyframe_;
    Frame::Ptr keyframe_, frame_;
    mutex data_mutex_;
};

#endif //MYSLAM_MAP_H
