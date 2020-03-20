cmake_minimum_required(VERSION 3.0)
project(light_weight_slam)

add_compile_options(-std=c++11 -O3 -pthread)

IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
        #cv_bridge
        #image_transport
        roscpp
        roslib
        std_msgs)

find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)

find_package(OpenCV 4 REQUIRED) # 3 is also ok

find_package(G2O REQUIRED)

find_package(DBoW3 REQUIRED)

#find_package(PCL REQUIRED)

find_package(Pangolin REQUIRED)

find_package(Glog REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myslam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${DBoW3_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/utils/pose.cpp
        src/utils/camera.cpp
        src/utils/config.cpp
        src/utils/mappoint.cpp
        src/utils/dataset.cpp
        src/utils/g2o_ext.cpp
        src/utils/cv_ext.cpp
        src/frame.cpp
        src/map.cpp
        src/mapper.cpp
        src/tracking.cpp
        src/optimizing.cpp
        src/system.cpp
        src/viewer.cpp
        src/loop_closure.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${OpenCV_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d
        ${DBoW3_LIBRARIES}
#        ${PCL_LIBRARYIES}
        ${Pangolin_LIBRARIES}
        ${GLOG_LIBRARIES}
)

add_executable(lightweight_slam src/myslam.cpp)

target_link_libraries(lightweight_slam ${PROJECT_NAME})

#############
## Install ##
#############


#############
## Testing ##
#############

