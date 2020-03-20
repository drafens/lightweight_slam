//
// Created by drafens on 2/26/20.
//
#ifndef MYSLAM_COVERT_H
#define MYSLAM_COVERT_H

// STL
template <typename T>
vector<T> operator +(vector<T> &v1, vector<T> &v2){
    vector<T> v = v1;
    v.insert(v.end(), v2.begin(), v2.end());
    return v;
}

inline Vector3d toVector(const cv::Matx31d& m){
    Vector3d v;
    cv::cv2eigen(m, v);
    return v;
}

inline Vector2d toVector(const Point2d& p) {
    Vector2d v(p.x, p.y);
    return v;
}

inline Vector3d toVector(const Point3d& p) {
    Vector3d v(p.x, p.y, p.z);
    return v;
}

inline Point2d toPoint(const Vector2d& v) {
    Point2d p(v[0], v[1]);
    return p;
}

inline Point3d toPoint(const Vector3d& v) {
    Point3d p(v[0], v[1], v[2]);
    return p;
}

inline Mat toMat(const Matrix3d& m) {
    Mat m_;
    cv::eigen2cv(m, m_);
    return m_;
}

inline Matrix3d toMatrix(const cv::Matx33d& m) {
    Matrix3d m_;
    cv::cv2eigen(m, m_);
    return m_;
}

inline Eigen::Isometry3d toIsometry3d(const SE3d& T){
    Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    I = Eigen::AngleAxisd(T.rotationMatrix());
    I.pretranslate(T.translation());
    return I;
}

inline SE3d relativeSE3(const SE3d& T1, const SE3d& T2){ // from T1 to T2 (T21)
    return T2 * T1.inverse();
}

#endif //MYSLAM_COVERT_H
