cmake_minimum_required(VERSION 3.0.2)
project(marm_standardfirmata_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  urdf
  xacro
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
