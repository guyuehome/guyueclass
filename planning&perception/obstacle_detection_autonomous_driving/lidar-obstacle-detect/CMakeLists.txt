cmake_minimum_required(VERSION 2.8.3)
project(lidar_obstacle_detection)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
  pcl_ros
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES lidar_detect
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加子目录并构建
add_subdirectory(lib/pre_process/roi_clip)
add_subdirectory(lib/pre_process/voxel_grid_filter)
add_subdirectory(lib/euclidean_cluster)
add_subdirectory(lib/bounding_box)
add_subdirectory(lib/ground_detector/patchwork)
add_subdirectory(lib/visualization)

add_executable(${PROJECT_NAME}_node src/lidar_obstacle_detection_node.cpp src/lidar_obstacle_detection.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  roi_clip
  voxel_grid_filter
  euclidean_cluster
  bounding_box
  patchwork
  visualization
 )


