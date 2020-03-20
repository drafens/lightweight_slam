//
// Created by drafens on 1/18/20.
//
#include "myslam/optimizing.h"
#include "myslam/frame.h"
#include "myslam/feature_point.h"
#include "myslam/cv_ext.h"

#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/robust_kernel_impl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

bool Optimizing::discard_optmize_data_ = false;

void Optimizing::ThreadLoop() {
    while (!closing_.load()) {
        unique_lock<mutex> lock(data_mutex_);
        cond_map_update_.wait(lock);

        // compute descriptors and notify loop closure
        if(!frame_) continue;
        Frame::Ptr frame = frame_;
        cv::computeDescriptors(descriptor_, frame);
        loop_closure_->updateMap(frame);

        // optimizing thread
        if(!map_) continue;
        unordered_map<size_t, size_t> match; // mp_new, mp_map
        matchLastKeyframe(match); // check redundant map point
        keyframe_list_active_ = map_->getActiveKeyframes();
        mappoint_list_active_ = map_->getActiveMappoints();
        discard_optmize_data_ = false;
        optimizePosePointKeyframe(match);
    }
}

Optimizing::Optimizing(bool running) {
    running_.store(running);
    closing_.store(false);
    descriptor_ = cv::ORB::create();
    flann_ = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(5, 10, 2));
    optimizing_thread_ = thread(bind(&Optimizing::ThreadLoop, this));
}

void Optimizing::close() {
    closing_.store(true);
    cond_map_update_.notify_one();
    optimizing_thread_.join();
}

void Optimizing::updateMap(const Frame::Ptr& frame){
    frame_ = frame;
    unique_lock<mutex> lock(data_mutex_);
    cond_map_update_.notify_one();
}

// use for loop closure global pose graph optimize
void Optimizing::optimizePoseGraph(int id, unordered_map<size_t, SE3d>& loop_list, const Map::Ptr& map, const Camera::Ptr& camera) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> Block;
    unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>);
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver)));
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto keyframe_list = map->getAllKeyframes();
    vector<g2o::VertexSE3*> vertices_pose;
    vector<g2o::VertexSE3*> vertices_pose_active;
    for(const auto& kf : keyframe_list){
        auto* v = new g2o::VertexSE3();
        v->setId(int(kf.first));
        v->setEstimate(toIsometry3d(kf.second->pose_->Tw()));
        if(0 == kf.first) v->setFixed(true);
        optimizer.addVertex(v);
        if(kf.second->active_) vertices_pose_active.push_back(v);
        else vertices_pose.push_back(v);
    }

    int index = 0;
    Eigen::Matrix<double, 6, 1> sigma;
    sigma << 10000, 10000, 10000, 40000, 40000, 40000;
    for (const auto& kf : keyframe_list) {
//        if(kf.first == 0) continue;
        for(const auto& delta : kf.second->relative_pose_){
            auto* e = new g2o::EdgeSE3();
            e->setId(index ++);
            e->setVertex(0, optimizer.vertex(int(delta.first)));
            e->setVertex(1, optimizer.vertex(int(kf.first)));
            e->setMeasurement(toIsometry3d(delta.second.inverse()));
            e->setInformation(sigma.asDiagonal());
            optimizer.addEdge(e);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(100);

    auto frame_list = map->getActiveFrames();
    // update active frames
    {
        unique_lock<mutex> lck(map->mtx_active_point_and_pose_);
        discard_optmize_data_ = true;
        for (auto v : vertices_pose_active) {
            auto kf = keyframe_list.at(v->id());
            auto T_old = kf->pose_->T();
            SE3d T_new = SE3d(v->estimate().rotation(), v->estimate().translation()).inverse();
            kf->pose_->setPose(T_new);
            if (v->id() == id) {
                for (const auto &f : frame_list) {
                    auto pose = f.second->pose_;
                    pose->setPose(relativeSE3(T_old, T_new) * pose->T());
                }
            }
            for (const auto &fp : kf->feature_points_) {
                auto mp = fp->map_point_.lock();
                if (!mp) continue;
                mp->pos_ = camera->camera2world(camera->world2camera(mp->pos_, T_old), T_new);
            }
        }
        lck.unlock();
    }

    // update other frames
    for(auto v : vertices_pose) {
        auto kf = keyframe_list.at(v->id());
        auto T_old = kf->pose_->T();
        SE3d T_new = SE3d(v->estimate().rotation(), v->estimate().translation()).inverse();
        kf->pose_->setPose(T_new);
        for (const auto& fp : kf->feature_points_) {
            auto mp = fp->map_point_.lock();
            if(!mp) continue;
            mp->pos_ = camera->camera2world(camera->world2camera(mp->pos_, T_old), T_new);
        }
    }
}

// use for tracking estimate pose
void Optimizing::optimizeOnlyPose(const shared_ptr<Frame>& frame, const Map::Ptr& map, const Camera::Ptr& camera) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>);
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver)));
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto* pose_cam = new g2o::VertexSE3Expmap();
    pose_cam->setEstimate(g2o::SE3Quat(frame->pose_->R(), frame->pose_->t()));
    pose_cam->setId(0);
    optimizer.addVertex(pose_cam);

    vector<FeaturePoint::Ptr> features;
    vector<g2o::EdgeProjectXYZ2UVPoseOnly*> edges;
    vector<size_t> idx;
    {
        unique_lock<mutex> lck(map->mtx_active_point_and_pose_);
        for (size_t i = 0, index = 0; i < frame->feature_points_.size(); i++) {
            auto fp = frame->feature_points_[i];
            auto mp = fp->map_point_.lock();
            auto pt = fp->keypoint_.pt;
            if (!mp) continue;
            features.push_back(fp);
            auto *edge = new g2o::EdgeProjectXYZ2UVPoseOnly(mp->pos_, camera);
            edge->setId(index++);
            edge->setVertex(0, pose_cam);
            edge->setMeasurement(toVector(pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto *kernel = new g2o::RobustKernelHuber();
            edge->setRobustKernel(kernel);
            edge->setLevel(0);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            idx.push_back(i);
        }
    }

    const double chi2_th[4] = {8, 4, 2, 2};
    for (int i = 0; i < 4; ++i) {
        pose_cam->setEstimate(g2o::SE3Quat(frame->pose_->R(), frame->pose_->t()));
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        for (size_t j = 0; j < edges.size(); ++j) {
            auto e = edges[j];
            if(features[j]->is_outlier_){
                e->computeError();
            }
            if (e->chi2() > chi2_th[i]){
                features[j]->is_outlier_ = true;
                e->setLevel(1);
            } else{
                features[j]->is_outlier_ = false;
                e->setLevel(0);
            }
            if (i == 2){
                e->setRobustKernel(nullptr);
            }
        }
    }

    int N = features.size();
    frame->pose_->setPose(pose_cam->estimate());
    for(size_t i=0; i< N; i++){
        auto f = features[i];
        if(f->is_outlier_){
            frame->feature_points_[idx[i]]->map_point_.reset();
            frame->feature_points_[idx[i]]->is_outlier_ = false;
            N --;
        }
    }
    frame->N_ = N;
}

void Optimizing::matchLastKeyframe(unordered_map<size_t, size_t>& match) {
    Mat descriptors_map, descriptors_new;
    vector<cv::Point2f> pts_map, pts_new;
    vector<size_t> idx_map, idx_new;
    {
        unique_lock<mutex> lck(map_->mtx_active_point_and_pose_);
        // mp in local map project to curr
        for (const auto &p : mappoint_list_active_) {
            auto mp = p.second;
            if (mp->id_keyframe_ == -1 || mp->id_keyframe_ == frame_->id_keyframe_) continue;
            if (frame_->isInFrame(mp->pos_)) {
                if (mp->is_outlier_) continue;
                if (mp->descriptors_.empty()) continue;
                descriptors_map.push_back(mp->descriptors_);
                pts_map.push_back(toPoint(camera_->world2pixel(mp->pos_, frame_->pose_->T())));
                idx_map.push_back(mp->id_);
            }
        }

        for (const auto &fp : frame_->feature_points_) {
            auto mp = fp->map_point_.lock();
            if (mp) {
                if (mp->descriptors_.empty()) continue;
                descriptors_new.push_back(mp->descriptors_);
                pts_new.push_back(fp->keypoint_.pt);
                idx_new.push_back(mp->id_);
            }
        }
        lck.unlock();
    }

    double euclidean_distance=3, descriptor_distance=100;
    // match with distance and descriptors
//    vector<pair<size_t, size_t>> match;
    for(int i=0; i<pts_map.size(); i++){
        for(int j=0; j<pts_new.size(); j++){
            if (pts_map[i].x-pts_new[j].x>euclidean_distance || pts_map[i].x-pts_new[j].x<-euclidean_distance) continue;
            if (pts_map[i].y-pts_new[j].y>euclidean_distance || pts_map[i].y-pts_new[j].y<-euclidean_distance) continue;
            if (norm(pts_map[i]-pts_new[j]) > euclidean_distance) continue;
            int d = cv::computeDescriptorDistance(descriptors_map.row(i), descriptors_new.row(j));
            if (d > descriptor_distance) continue;
            match.insert(make_pair(idx_new[j], idx_map[i]));
            break;
        }
    }

//    LOG(ERROR) << match.size();
    if(match.size() > 3){
        vector<cv::Point3f> pts3f;
        vector<cv::Point2f> pts2f;
        for (auto m : match){
            auto mp = mappoint_list_active_.at(m.second);
            pts3f.push_back(toPoint(mp->pos_));
        }
        // TODO solvePnP() compute relative pose and compare with estimate by optical flow
    }
    map_->fuseMappoint(match);
}

void Optimizing::optimizePosePointKeyframe(unordered_map<size_t, size_t> match){
    if(!running_.load()) return;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>);
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver)));
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    map<size_t, g2o::VertexSE3Expmap*> vertices_pose;
    size_t max_index = 0;
    size_t min_id_kf = _max_value;
    for (const auto& k : keyframe_list_active_) {
        auto kf = k.second;
        auto* pose_cam = new g2o::VertexSE3Expmap();
        pose_cam->setId(max_index++);
        pose_cam->setEstimate(g2o::SE3Quat(kf->pose_->R(), kf->pose_->t()));
        optimizer.addVertex(pose_cam);
        vertices_pose.insert({kf->id_keyframe_, pose_cam});
        if(kf->id_keyframe_ > max_index) max_index = kf->id_keyframe_;
        if(kf->id_keyframe_ < min_id_kf) min_id_kf = kf->id_keyframe_;
    }
    vertices_pose.at(min_id_kf)->setFixed(true);

    double chi2_th = 5;
    map<size_t, g2o::VertexSBAPointXYZ*> vertices_point;
    vector<g2o::EdgeProjectXYZ2UVStereo*> edges;
    vector<FeaturePoint::Ptr> features;
    size_t index = 0;
    for (const auto& m : mappoint_list_active_) {
        auto mp = m.second;
        if (mp->is_outlier_) continue;
        if (vertices_point.find(mp->id_) == vertices_point.end()) {
            auto *point = new g2o::VertexSBAPointXYZ();
            point->setId(max_index++);
            point->setEstimate(mp->pos_);
            point->setMarginalized(true);
            optimizer.addVertex(point);
            vertices_point.insert({mp->id_, point});
        }

        int weight = 1;
        // 为fused的点增加优化权重
        if(match.find(mp->id_)!=match.end()) weight=500;
        for (const auto &ob : mp->getObs()) {
            auto fp = ob.lock();
            if (!fp) continue;

            auto *edge = new g2o::EdgeProjectXYZ2UVStereo(camera_, fp->is_on_right_);
            edge->setId(index++);
            edge->setWeight(weight);
            edge->setVertex(0, vertices_pose[fp->frame_.lock()->id_keyframe_]);
            edge->setVertex(1, vertices_point[mp->id_]);
            edge->setMeasurement(toVector(fp->keypoint_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto kernel = new g2o::RobustKernelHuber();
            kernel->setDelta(chi2_th*weight);
            edge->setRobustKernel(kernel);
            edge->setParameterId(0, 0); // or will be a bad_typeid error
            optimizer.addEdge(edge);
            edges.push_back(edge);
            features.push_back(fp);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // avoid outlier too much points
    for (int i = 0; i < 5; ++i) {
        int outlier = 0;
        for (auto e : edges) {
            if (e->chi2() > chi2_th) {
                outlier++;
            }
        }
        double rate = outlier / 1.0 / edges.size();
        if(rate < 0.2) break;
        chi2_th *= 2;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        if (edges[i]->chi2() > chi2_th) {
            features[i]->is_outlier_ = true;
            features[i]->map_point_.lock()->removeObs(features[i]);
            edges[i]->setLevel(1);
        } else {
            features[i]->is_outlier_ = false;
            edges[i]->setRobustKernel(nullptr);
            edges[i]->setLevel(0);
        }
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    {
        unique_lock<mutex> lck(map_->mtx_active_point_and_pose_);
        if(discard_optmize_data_) return;
        for (auto v : vertices_pose) {
            auto kf = keyframe_list_active_.at(v.first);
            kf->pose_->setPose(v.second->estimate());
            for (auto f : vertices_pose) {
                if (v.first > f.first) {
                    SE3d relative = relativeSE3(SE3d(f.second->estimate().rotation(), f.second->estimate().translation()), kf->pose_->T());
                    if(kf->relative_pose_.find(f.first)==kf->relative_pose_.end())
                        kf->relative_pose_.insert(make_pair(f.first, relative));
                    else
                        kf->relative_pose_.at(f.first) = relative;
                }
            }
        }
        for (auto v : vertices_point) {
            mappoint_list_active_.at(v.first)->pos_ = v.second->estimate();
        }
        lck.unlock();
    }

}

