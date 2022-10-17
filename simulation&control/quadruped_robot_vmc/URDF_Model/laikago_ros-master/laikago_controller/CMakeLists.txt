cmake_minimum_required(VERSION 2.8.3)
project(laikago_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    laikago_msgs
)

catkin_package(
    CATKIN_DEPENDS
    laikago_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories($(catkin_LIB_DIRS) lib)

add_library(laikago_joint_controller src/joint_controller.cpp)
add_dependencies(laikago_joint_controller laikago_msgs_gencpp)
target_link_libraries(laikago_joint_controller ${catkin_LIBRARIES} laikago_control_tool)
