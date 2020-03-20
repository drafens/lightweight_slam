//
// Created by drafens on 3/16/20.
//
#ifndef MYSLAM_MAPPER_H
#define MYSLAM_MAPPER_H

#include "common_include.h"
#include "map.h"
#include "camera.h"
#include "frame.h"

class DelaunaryTriangle{
public:
    typedef shared_ptr<DelaunaryTriangle> Ptr;
//    DelaunaryTriangle(){};
    bool sameAs(const DelaunaryTriangle::Ptr& tmp) {
        return id_ == tmp->id_ || false;
        /*return vertex_[0].lock()->id_ == tmp->vertex_[0].lock()->id_
            && vertex_[1].lock()->id_ == tmp->vertex_[1].lock()->id_
            && vertex_[2].lock()->id_ == tmp->vertex_[2].lock()->id_;*/
    }
    void push_back(const MapPoint::Ptr& mp){
        vertex_.insert(make_pair(mp->id_, mp));
        if(id_ == -1 && vertex_.size() >= 3){
            // TODO compute norm
            id_ = id_nxt_++;
        }
    }

    map<size_t, weak_ptr<MapPoint>> vertex(){
        return vertex_;
    }

    // TODO
//    bool inTriangle(MapPoint::Ptr){
//        return false;
//    }

    size_t id_ = -1;
private:
    map<size_t, weak_ptr<MapPoint>> vertex_;
    static size_t id_nxt_;
};

class Mapper {
public:
    typedef unordered_map<size_t, DelaunaryTriangle::Ptr> TrianglesType;
    typedef shared_ptr<Mapper> Ptr;
    explicit Mapper(bool running);
    void close();
    void setCamera(Camera::Ptr camera){camera_=move(camera);}
    void setMap(Map::Ptr map){map_=move(map);}

    void updateMap(const Frame::Ptr& frame);
    TrianglesType getTriangles();
private:
    void ThreadLoop();
    void getDelaunary();
    void fuseTriangles();
    void checkTriangles();
    void deleteTriangle(DelaunaryTriangle::Ptr);

    // triangle_id & triangle
    TrianglesType triangles_curr_, triangles_map_;
    // each mappoint(id) has a triangles vector
    unordered_map<size_t, vector<DelaunaryTriangle::Ptr>> triangles_in_mp_;
    // mappoint & pixel in new keyframe
    unordered_map<MapPoint::Ptr, cv::Point2f> pts_3D2D_new_;
    // the mp has already exist in map
    vector<size_t> pts_update_;
    Frame::Ptr frame_;
    Camera::Ptr camera_;
    Map::Ptr map_;
    Map::MappointsType mappoint_list_active_;

    atomic<bool> running_{};
    condition_variable cond_map_update_;
    thread mapper_thread_;
    mutex data_mutex_;
};

#endif //MYSLAM_MAPPER_H
