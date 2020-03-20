//
// Created by drafens on 1/31/20.

#include "myslam/viewer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

struct PangolinMatrix3d{
    explicit PangolinMatrix3d(Matrix3d m=Matrix3d::Identity()){data = move(m);}
    Matrix3d data;
};
struct PangolinVector3d{
    explicit PangolinVector3d(Vector3d v=Vector3d::Zero()){data = move(v);}
    Vector3d data;
};

ostream& operator <<(ostream& out, const PangolinMatrix3d& m){
    auto R = m.data.eulerAngles(0, 1, 2); //roll-pitch-yaw
    out << fixed << setprecision(2);
    out << "["<<setw(5)<<R[0]<<","<<setw(5)<<R[1]<<","<<setw(5)<<R[2] <<"]";
    return out;
}
istream& operator >>(istream& in, PangolinMatrix3d& m){
    return in;
}

ostream& operator <<(ostream& out, const PangolinVector3d& v){
    auto t = v.data;
    out << fixed << setprecision(1);
    out << "["<<setw(5)<<t[0]<<','<<setw(5)<<t[1]<<','<<setw(5)<<t[2] <<"]";
    return out;
}
istream& operator >>(istream& in, PangolinVector3d& v){
    return in;
}

void Viewer::ThreadLoop(){
    if(!running_.load()) return;
    pangolin::CreateWindowAndBind(_proj_name, width_, height_);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pcl
    pangolin::OpenGlRenderState vis_cam(
            pangolin::ProjectionMatrix(width_, height_, camera_->fx_, camera_->fy_, camera_->cx_, camera_->cy_, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -30, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::View& vis_map = pangolin::CreateDisplay().SetBounds(0.2, 1.0, 0.2, 1.0, -width_ / 1.0 / height_).SetHandler(new pangolin::Handler3D(vis_cam));
    // image
    double image_view_height = image_height_/1.0/image_width_*0.4;
    pangolin::View& vis_image = pangolin::Display("left").SetBounds(0.2-image_view_height, 0.2, 0.2, 0.6, -image_width_/1.0/image_height_).SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::View& vis_image_right = pangolin::Display("right").SetBounds(0.2-image_view_height, 0.2, 0.6, 1.0, -image_width_/1.0/image_height_).SetLock(pangolin::LockLeft, pangolin::LockBottom);
    pangolin::GlTexture imageTexture(image_width_, image_height_,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    // panel
    pangolin::CreatePanel("Setting").SetBounds(0.0, 1.0, 0.0, 0.2);
    pangolin::Var<bool> setting_box("Setting.box", true, true);
    pangolin::Var<double> setting_double("Setting.double", 30, 1, 100);
    pangolin::Var<bool> setting_button("Setting.button",false,false);
    pangolin::Var<int> setting_keyframe("Setting.keyframe", 0);
    pangolin::Var<string> setting_frame("Setting.frame","0");
    pangolin::Var<PangolinMatrix3d> setting_rotation("Setting.rpy=", PangolinMatrix3d());
    pangolin::Var<PangolinVector3d> setting_translation("Setting.t=", PangolinVector3d());

    while(!pangolin::ShouldQuit() && running_.load()){
        unique_lock<mutex> lock(data_mutex_);
        cond_map_update_.wait(lock);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // NOLINT(hicpp-signed-bitwise)
        glClearColor(1.0, 1.0, 1.0, 1.0);

        if(map_){
            vis_map.Activate(vis_cam);
            moveCamera(vis_cam);
            drawMap();
        }

        if(frame_){
            //image
            vis_image.Activate();
            Mat image = plotPointsOnImage(true, green_);
            imageTexture.Upload(image.data, GL_BGR, GL_UNSIGNED_BYTE);
            glColor3f(1.0,1.0,1.0);
            imageTexture.RenderToViewportFlipY();

            vis_image_right.Activate();
            Mat image_right = plotPointsOnImage(false, green_);
            imageTexture.Upload(image_right.data, GL_BGR, GL_UNSIGNED_BYTE);
            glColor3f(1.0,1.0,1.0);
            imageTexture.RenderToViewportFlipY();

            //pannel
            setting_rotation = PangolinMatrix3d(frame_->pose_->Rw());
            setting_translation = PangolinVector3d(frame_->pose_->tw());
            setting_frame = to_string(frame_->id_);
            if(-1 != frame_->id_keyframe_) setting_keyframe = frame_->id_keyframe_;
        }
        pangolin::FinishFrame();
//        usleep(10000);
    }
}

Viewer::Viewer(bool running) {
    running_.store(running);
    if(running_.load()) {
        width_ = Config::get<double>("Viewer.width");
        height_ = Config::get<double>("Viewer.height");
        camera_ = std::make_shared<Camera>(400, 400, 512, 384);
        viewer_thread_ = thread(bind(&Viewer::ThreadLoop, this));
    }
}

void Viewer::close() {
    if(!running_.load()) return;
    running_.store(false);
    cond_map_update_.notify_one();
    viewer_thread_.join();
}

void Viewer::drawFrame(const Frame::Ptr& frame, const double* color){
    if(!running_.load()) return;
    SE3d T = frame->pose_->Tw();
    glPushMatrix();
    glMultMatrixd(T.matrix().data());
    glColor3d(color[0], color[1], color[2]);
    glLineWidth(2.0);
    glBegin(GL_LINES);

    vector<Vector3d> vertex3ds;
    int vertex_idx[16] = {0, 1, 0, 2, 0, 3, 0, 4, 4, 3, 3, 2, 2, 1, 1, 4};
    vertex3ds.emplace_back(0, 0, 0);
    vertex3ds.push_back(camera_->pixel2camera(Vector2d(0, 0)));
    vertex3ds.push_back(camera_->pixel2camera(Vector2d(0, height_)));
    vertex3ds.push_back(camera_->pixel2camera(Vector2d(width_, height_)));
    vertex3ds.push_back(camera_->pixel2camera(Vector2d(width_, 0)));
    for (int i : vertex_idx) {
        Vector3d v = vertex3ds[i];
        glVertex3d(v[0], v[1], v[2]);
    }
    glEnd();
    glPopMatrix();
}

void Viewer::moveCamera(pangolin::OpenGlRenderState& cam){
    if(!running_.load()) return;
    SE3d T = SE3d();
    if(frame_) T = frame_->pose_->Tw();

    pangolin::OpenGlMatrix m(T.matrix());
    cam.Follow(m, true);
}

void Viewer::drawMap(){
    if(!running_.load()) return;

    if(frame_) {
        drawFrame(frame_, green_);
    }
    for (const auto& f : keyframe_list_) {
        drawFrame(f.second, red_);
    }
    for (const auto& f : frame_list_active_) {
        drawFrame(f.second, blue_);
    }
    glPointSize(2);
    glBegin(GL_POINTS);
    for (const auto& m : mappoint_list_active_) {
        auto pw = m.second->pos_;
        glColor3f(red_[0], red_[1], red_[2]);
        glVertex3d(pw[0], pw[1], pw[2]);
    }
    glEnd();
}

Mat Viewer::plotPointsOnImage(bool is_left, const double* color){
    if(!running_.load()) return Mat();
    Mat image_out;
    if(is_left) {
        cv::cvtColor(frame_->image_, image_out, CV_GRAY2BGR);
        for (auto & feature_point : frame_->feature_points_) {
            auto mp = feature_point->map_point_.lock();
            if (mp) {
                cv::circle(image_out, feature_point->keypoint_.pt, 4, cv::Scalar(color[2], color[1], color[0])*255, cv::FILLED);
            }
        }
    } else{
        cv::cvtColor(frame_->image_right_, image_out, CV_GRAY2BGR);
    }

    cv::Size size(image_width_, image_height_);
    cv::resize(image_out, image_out, size);
    return image_out;
}

void Viewer::addFrame(Frame::Ptr frame) {
    if(!running_.load()) return;
    unique_lock<mutex> lock(data_mutex_);
    if(frame->id_keyframe_==-1) {
        frame_last_ = frame_;
        frame_ = move(frame);
        frame_list_active_ = map_->getActiveFrames();
    }else{
        if(map_){
            keyframe_list_ = map_->getAllKeyframes();
            mappoint_list_active_ = map_->getActiveMappoints();
        }
    }
    cond_map_update_.notify_one();
}

void Viewer::updateMap() {
    if(!running_.load()) return;
    unique_lock<mutex> lock(data_mutex_);
    if(map_){
        keyframe_list_ = map_->getAllKeyframes();
        mappoint_list_active_ = map_->getActiveMappoints();
    }
}
