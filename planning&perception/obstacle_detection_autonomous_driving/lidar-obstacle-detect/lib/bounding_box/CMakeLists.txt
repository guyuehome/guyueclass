cmake_minimum_required(VERSION 2.8.3)
project(bounding_box)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



aux_source_directory(. DIR_SRCS)

#SHARED 动态库
#STATIC 静态库
add_library(bounding_box STATIC ${DIR_SRCS})

target_link_libraries(bounding_box
  # ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
 )

