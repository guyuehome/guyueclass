cmake_minimum_required(VERSION 2.8.3)
project(visualization)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
aux_source_directory(. DIR_SRCS)

add_library(visualization ${DIR_SRCS})

