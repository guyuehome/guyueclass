cmake_minimum_required(VERSION 2.8.3)
project(pcl_turtorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
)

find_package(PCL REQUIRED)
include_directories(include  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(pcl_sample src/pcl_sample.cpp)
target_link_libraries(pcl_sample ${catkin_LIBRARIES} ${PCL_LIBRARIES})