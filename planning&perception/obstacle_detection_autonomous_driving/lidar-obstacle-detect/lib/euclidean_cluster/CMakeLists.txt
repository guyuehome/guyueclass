cmake_minimum_required(VERSION 2.8.3)
project(euclidean_cluster)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  jsk_rviz_plugins
)

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(. DIR_SRCS)

add_library(euclidean_cluster STATIC ${DIR_SRCS})

target_link_libraries(euclidean_cluster
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
 )

