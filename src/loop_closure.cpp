//
// Created by drafens on 2/19/20.
//
#include "myslam/loop_closure.h"
#include "myslam/config.h"
#include "myslam/cv_ext.h"
#include "myslam/optimizing.h"

LoopClosure::LoopClosure(bool running) {
    running_.store(running);
    auto t1 = chrono::steady_clock::now();
    if(running_.load()) {
        matcher_ = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(5, 10, 2));
        auto path = Config::get<string>("Path.vocabulary");
        vocabulary_ = DBoW3::Vocabulary(path);
        if (vocabulary_.empty()) {
            LOG(ERROR) << "Vocabulary does not exist, so unable loop closure.";
            running_.store(false);
        }
        database_ = DBoW3::Database(vocabulary_, false, 0);
        min_score_ = Config::get<double>("LoopClosure.min_score");
        max_score_ = Config::get<double>("LoopClosure.max_score");
        score_factor_ = Config::get<double>("LoopClosure.score_factor");
        loop_thread_ = thread(bind(&LoopClosure::ThreadLoop, this));
        auto t2 = chrono::steady_clock::now();
        auto t = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        LOG(WARNING) << "read vocabulary from " << path << " use " << t.count()*1000 << " ms.";
    }
}

void LoopClosure::close() {
    if(!running_.load()) return;
    running_.store(false);
    cond_map_update_.notify_one();
    loop_thread_.join();
}

void LoopClosure::ThreadLoop(){
    while (running_.load()){
        unique_lock<mutex> lock(data_mutex_);
        cond_map_update_.wait(lock);

        if(!frame_) continue;
        updateVoc();
        checkLoop();
    }
}

void LoopClosure::updateMap(const Frame::Ptr& frame){
    if(!running_.load()) return;
    frame_ = frame;
    cond_map_update_.notify_one();
}

void LoopClosure::updateVoc() {
    if(!running_.load()) return;
    DBoW3::BowVector bow_vector;
    vocabulary_.transform(frame_->descriptors_, bow_vector);
    if(database_.size() <= 0) {
        frame_->loop_score_ = max_score_ / 2;
    } else {
        frame_->loop_score_ = score_factor_ * vocabulary_.score(bow_vector_last_, bow_vector);
    }
    database_.add(bow_vector);
    bow_vector_last_ = bow_vector;
}

void LoopClosure::checkLoop() {
    if (!running_.load()) return;
    if(database_.size() < 15) return;

    DBoW3::QueryResults results;
    Frame::Ptr frame = frame_;
    database_.query(frame->descriptors_, results, 4);

    unordered_map<size_t, SE3d> loop_results;
    for(auto result : results) {
        if (result.Score < min_score_ || database_.size() - result.Id < 5) continue;
        auto keyframe_list = map_->getAllKeyframes();
        frame_loop_ = keyframe_list.at(result.Id);
        double loop_score = min(max_score_, (frame_loop_->loop_score_+frame->loop_score_)/2); // no more than 0.1 and no less than 0.02
        if (result.Score < loop_score) continue;
        LOG(INFO) << "searching for keyframe loop " << frame->id_ << " and " << frame_loop_->id_;

        SE3d relative_pose;
        if (verifyLoop(relative_pose, frame)) {
            loop_results.insert(make_pair(frame_loop_->id_keyframe_, relative_pose));
            frame_->relative_pose_.insert(make_pair(frame_loop_->id_keyframe_, relative_pose));
        }
    }
    if(!loop_results.empty()){
        Optimizing::optimizePoseGraph(frame->id_keyframe_, loop_results, map_, camera_);
        LOG(WARNING) << "loop success";
    }
}

// compute relative_pose
bool LoopClosure::verifyLoop(SE3d& relative, const Frame::Ptr& frame) {
    vector<cv::DMatch> matches;
    vector<cv::Point3d> pts3d;
    vector<cv::Point2d> pts2d;
    if(cv::matchDescriptors(matcher_, frame_loop_, frame, pts2d, pts3d)){
        if(!cv::ransac(pts3d, pts2d, 0.5, 30)) return false;
        cv::Mat R, r, t;
        cv::solvePnPRansac(pts3d, pts2d, camera_->K_, camera_->DistCoef_, r, t);
        cv::Rodrigues(r, R);
        relative = relativeSE3(SE3d(toMatrix(R), toVector(t)), frame->pose_->T());
        bool trans_ok = relative.log().head<3>().norm() < 10;
        bool rot_ok = relative.log().tail<3>().norm() < 0.2;
        return (trans_ok && rot_ok);
    }
    return false;
}

