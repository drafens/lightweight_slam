# lightweight_slam

This is a light weight but complete SLAM framework，There are mainly 5 modules

+ Tracking: use optical flow to track (include stereo match with right image)

+ Optimizing: fused map point and optimize local map with G2O

+ Loop Closure: use DBow3 find, verify by match descriptors and optimize Pose Graph

+ Viewer: use pangolin display trajectory, point cloud and image

+ Map: use 2d Delaunay triangle back project to generate 3D Mesh represent (unfinished) 

each module can be easily opened or closed

## Result

+ APE w.r.t. translation part (m)
  (not aligned)

```
       max	5.814456
      mean	2.589880
    median	2.345237
       min	0.000000
      rmse	2.897114
       sse	23173.811510
       std	1.298379
```

![result](assest/result.png)

+ light-weight

  track keyframe: about 13ms

  track frame: about 3ms

  Easily handle high speed movement scenes more than 60-90Hz

## Run it

​	To run the demo, You need install  ROS, Eigen3, Sophus, OpenCV, G2O, DBow, Pangolin, and GLOG.

​	You can put the directory in `catkin_ws/src`, and run `catkin_make` to compile

​	You can edit parameters in `./config/kitti.yaml`

​	Run `rosrun light_weight_slam lightweight_slam` to run this demo

​	Run `evo_ape kitti path/to/ground_truth path/to/output/CameraTrajectory.txt -vp --plot_mode=xz`

> `path/to/output` can find in `./config/kitti.yaml`

## Framework

+ Tracking

  仅对关键帧做特征提取和立体匹配，普通帧使用光流追踪，对匹配点做RANSAC排除外点后使用G2O追小而成估计当前帧位姿。

  特征提取采用GFTT，可以提到分布均匀的高梯度特征点

  关键帧选取：根据剩余追踪到的地图点数量及旋转

  > 部分参考自：高博《SLAM十四讲》第二版ch13

+ Optimizing

  对相邻关键帧中重复冗余的地图点做融合，每个地图点保存其位置和评分

  通过重投影在当前帧的实际距离和描述子之间的距离判定为同一地图点，并通过一下方式更新

  `new_pos = (mp1_pos * mp2_score + mp2_pos * mp1_score) / (mp1_score + mp2_score)`

  `new_score = sqrt(mp1_scores^2 + mp2_score^2)`

  > 代码见`Optimizing::matchLastKeyframe`和`Map::fuseMappoint`

  对整个局部地图做G2O优化，并提高融合后的地图点的权重

  > 由于连续优化的过程，不提高权重，G2O容易将性融合的少数地图点视为外点不优化，提高权重的方式见 `include/g2o_ext`

+ Loop Closure

  使用DBoW3查找相似帧

  对相似帧做特征（ORB）匹配，验证回环

  全局地图做Pose Graph优化

+ Viewer

  使用Pangolin显示点云，轨迹，左右图像等内容

+ Map Represent (unfinished) 

  为实现轻量级地图表示，使用对当前关键帧提取2D Delaunay三角形拓扑结构提取

  逆投影到3D地图点，形成3D Mesh地图

  更新：查找局部地图所有投影值当前帧的地图点，将其形成的Mesh地图更新为最新关键帧形成的地图

  边缘化：对于视野外（不在局部地图中）的地图点，删除其形成的Mesh地图

  > 参考文章 Incremental Visual-Inertial 3D Mesh Generation with Structural Regularities
  >
  > 但此方法对室外开阔场景表现并不很理想，主要由于2D地图中相邻的地图点可能实际深度相差很大，所以局部区域Mesh地图表现尚可，但整体地图存在一定误差

## 代码

`src/myslam`为整个代码入口

`src/System::init`你可已通过次入口控制每一个模块的开启与关闭

其余模块均已其功能命名

> 有任何想法可以与我 19212020101@fudan.edu.cn 联系
