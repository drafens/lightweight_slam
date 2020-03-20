//
// Created by drafens on 3/16/20.
//
#include "myslam/mapper.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

size_t DelaunaryTriangle::id_nxt_ = 0;

Mapper::Mapper(bool running){
    running_.store(running);
    if(running_.load()) {
        mapper_thread_ = thread(bind(&Mapper::ThreadLoop, this));
    }
}

void Mapper::close() {
    if(!running_.load()) return;
    running_.store(false);
    cond_map_update_.notify_one();
    mapper_thread_.join();
}

void Mapper::ThreadLoop(){
    while (running_.load()){
        unique_lock<mutex> lock(data_mutex_);
        cond_map_update_.wait(lock);

        if(!frame_) continue;
        getDelaunary();
        checkTriangles();
        fuseTriangles();
    }
}


void Mapper::getDelaunary() {
    static int width = frame_->image_.cols;
    static int height = frame_->image_.rows;
    auto frame = frame_;
    const cv::Rect rect(0, 0, width, height);
    cv::Subdiv2D subdiv2D;
    subdiv2D.initDelaunay(rect);

    pts_3D2D_new_.clear();
    pts_update_.clear();
    triangles_curr_.clear();

    {
        unique_lock<mutex> lck(map_->mtx_active_point_and_pose_);
        // mp in local map project to curr
        for (const auto &p : mappoint_list_active_) {
            auto mp = p.second;
            if (frame->isInFrame(mp->pos_)) {
                if (mp->is_outlier_) continue;
                Point2d px = toPoint(camera_->world2pixel(mp->pos_, frame->pose_->T()));
                pts_3D2D_new_.insert(make_pair(mp, px));
                subdiv2D.insert(px);
                if(mp->id_keyframe_!=frame->id_keyframe_) pts_update_.push_back(mp->id_);
            }
        }
    }

    vector<cv::Vec6f> results;
    subdiv2D.getTriangleList(results);
    for (auto & result : results) {
        // put result in triangeles
        DelaunaryTriangle::Ptr triangle(new DelaunaryTriangle);
        vector<bool> flag(3, true);
        for(const auto& pt : pts_3D2D_new_){
            if(!flag[0] && !flag[1] && !flag[2]) break;
            for(int i=0; i < 3; i++){
                if(flag[i] && result[2*i]==pt.second.x){
                    if(result[2*i+1]==pt.second.y){
                        triangle->push_back(pt.first);
                        flag[i] = false;
                        continue;
                    }
                }
            }
        }
        if(triangle->id_!=-1) {
            triangles_curr_.insert(make_pair(triangle->id_, triangle));
        }
    }


////    show 2D image
//    static vector<Mat> img;
//    Mat image_o;
//    cv::cvtColor(frame->image_, image_o, CV_GRAY2BGR);
//    LOG(ERROR) << "tri size " << triangles_curr_.size();
//    for (const auto& m : triangles_curr_) {
//        vector<cv::Point2f> tri;
//        for (const auto& mp : m.second->vertex_) {
//            Vector3d v = mp.lock()->pos_;
//            Vector3d pc = frame->pose_->T()*v;
//            Vector2d px(707.0912 * pc ( 0,0 ) / pc ( 2,0 ) + 601.8873,
//                        707.0912 * pc ( 1,0 ) / pc ( 2,0 ) + 183.1104);
//            tri.push_back(toPoint(px)); //
//        }
//        cv::circle(image_o, tri[0], 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(image_o, tri[1], 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(image_o, tri[2], 2, CV_RGB(255, 0, 0), -1);
//        cv::line(image_o, tri[0], tri[1], CV_RGB(0, 0, 255));
//        cv::line(image_o, tri[1], tri[2], CV_RGB(0, 0, 255));
//        cv::line(image_o, tri[2], tri[0], CV_RGB(0, 0, 255));
//    }
//    img.push_back(image_o);
//
//    char idx = 'a';
//    if(img.size()>=2) {
//        for (auto i : img)
//            cv::imshow(idx++, i);
//        cv::waitKey(0);
//        exit(0);
//    }
}

void Mapper::fuseTriangles() {
    // 删除当前视野内过时的三角形
    for(auto mp : pts_update_){
        if(triangles_in_mp_.find(mp) == triangles_in_mp_.end()) continue;
        for(const auto& triangle : triangles_in_mp_.at(mp)){
            deleteTriangle(triangle);
        }
        triangles_in_mp_.erase(mp);
    }
    // update and fuse
    for(auto tri : triangles_curr_){
        triangles_map_.insert(tri);
        for(const auto& v : tri.second->vertex()) {
            size_t mp_id = v.first;
            vector<DelaunaryTriangle::Ptr> tmp;
            tmp.push_back(tri.second);
            triangles_in_mp_.insert(make_pair(mp_id, tmp));
        }
    }
}

void Mapper::deleteTriangle(DelaunaryTriangle::Ptr triangle){
    for (const auto& v : triangle->vertex()) {
        auto triangle_in_v = triangles_in_mp_[v.first];
        for(auto it=triangle_in_v.begin();it!=triangle_in_v.end();it++){
            if(triangle->sameAs(*it)){
                triangle_in_v.erase(it);
                break;
            }
        }
    }
    if(triangles_map_.find(triangle->id_)!=triangles_map_.end()) {
        triangles_map_.erase(triangle->id_);
        triangle.reset();
    }
}

void Mapper::checkTriangles() {
    // delete the triangle using mp has been delete
    for(const auto& mp : triangles_in_mp_){
        if(mappoint_list_active_.find(mp.first) == mappoint_list_active_.end()) {
            // map_point 被删除
            for(const auto & it : mp.second){
                if(it) { // 什么时候被删除的？
                    // each triangle in map point 将被删除
                    deleteTriangle(it);
                }
            }
            triangles_in_mp_.erase(mp.first);
        }
    }
    // TODO 检查不合规则的三间形
}

void Mapper::updateMap(const Frame::Ptr &frame) {
    frame_ = frame;
    unique_lock<mutex> lock(data_mutex_);
    mappoint_list_active_ = map_->getActiveMappoints();
    cond_map_update_.notify_one();
}

Mapper::TrianglesType Mapper::getTriangles(){
    return triangles_map_;
}
