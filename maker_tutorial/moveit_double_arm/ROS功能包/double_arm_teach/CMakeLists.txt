cmake_minimum_required(VERSION 3.0.2)
project(double_arm_teach)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES double_arm_teach
   CATKIN_DEPENDS rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

