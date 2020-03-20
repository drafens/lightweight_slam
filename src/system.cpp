//
// Created by drafens on 1/17/20.
//
#include <csignal>
#include <ros/ros.h>
#include "myslam/system.h"

bool System::running_ = true;
void System::checkExit(int sig){
    if (sig != SIGINT) return;
    LOG(INFO) << "exiting by catching ctrl+C...";
    running_ = false;
}

bool System::init(int argc, char **argv) {
    Config::setParaFile();

    fps_ = Config::get<int>("Camera.fps");
    if(argc>1) {
        int arg = int(strtol(argv[1], nullptr, 10));
        if(arg > 0) fps_ = arg;
    }

    dataset_ = make_shared<Dataset>();
    tracker_ = make_shared<Tracking>();
    optimizer_ = make_shared<Optimizing>(true);
    viewer_ = make_shared<Viewer>(true);
    loop_closure_ = make_shared<LoopClosure>(true);
    mapper_ = make_shared<Mapper>(false);
    map_ = make_shared<Map>();
    camera_ = make_shared<Camera>();

    tracker_->setCamera(camera_);
    tracker_->setMap(map_);
    tracker_->setViewer(viewer_);
    tracker_->setOptimizer(optimizer_);
    tracker_->setMapper(mapper_);

    optimizer_->setMap(map_);
    optimizer_->setCamera(camera_);
    optimizer_->setLoopClosure(loop_closure_);

    loop_closure_->setMap(map_);
    loop_closure_->setCamera(camera_);

    viewer_->setMap(map_);

    mapper_->setMap(map_);
    mapper_->setCamera(camera_);

    map_->setCamera(camera_);
}

bool System::run() {
    ros::Rate loop(fps_);
    LOG(WARNING) << "beginning...";
    auto t1 = chrono::steady_clock::now();
    while(running_){
        if(!runStep()){
            break;
        }

        signal(SIGINT, checkExit);
        loop.sleep();
    }
    auto t2 = chrono::steady_clock::now();
    auto t = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    LOG(WARNING) << "total cost time: " << t.count() << " s.";

    optimizer_->close();
    loop_closure_->close();
    viewer_->close();
    tracker_->close();
    LOG(WARNING) << "finishing...";

    return true;
}

bool System::runStep() {
    Frame::Ptr frame = dataset_->nxtFrame();
    if (frame == nullptr) return false;
    auto t1 = chrono::steady_clock::now();
    bool ok = tracker_->track(frame);
    auto t2 = chrono::steady_clock::now();
    auto t = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    LOG(WARNING) << " frame " << frame->id_ << " cost time: " << t.count()*1000 << " ms.";
    return ok;
}

bool System::saveTrajectory() {
    string path = Config::get<string>("Path.output") + "CameraTrajectory.txt";
    auto frames = map_->getAllFramePoses();
    auto keyframes = map_->getAllKeyframes();
    SE3d kf_pose = SE3d();

    ofstream file(path, ios::in | ios::trunc);
    if(file) {
        for (const auto &f : frames) {
            auto T = SE3d().matrix();
            if(f.first != -1){
                kf_pose = keyframes.at(f.first)->pose_->T();
                T = kf_pose.inverse().matrix();
            } else{
                T = (f.second * kf_pose).inverse().matrix();
            }
            file << scientific << setprecision(6);
            for (int i = 0; i < 12; ++i) {
                string tail = " ";
                if (i == 11) tail = "\n";
                file << T(i/4, i%4) << tail;
            }
        }
        file.close();
        LOG(WARNING) << "save trajectory at "<< path << ".";
    }else{
        LOG(ERROR) << path << " open failed.";
    }
    return false;
}
