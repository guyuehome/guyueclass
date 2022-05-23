cmake_minimum_required(VERSION 3.0.2)
project(kdl)

find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
)

find_library(KDL_PARSER_LIB kdl_parser /opt/ros/kinetic/lib/)

add_executable(parser src/parser.cpp)
target_link_libraries(parser ${orocos_kdl_LIBRARIES} ${KDL_PARSER_LIB})

