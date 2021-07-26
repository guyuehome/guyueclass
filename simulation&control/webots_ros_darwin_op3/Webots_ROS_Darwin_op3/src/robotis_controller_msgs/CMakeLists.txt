################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robotis_controller_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  SyncWriteItem.msg
  JointCtrlModule.msg
  StatusMsg.msg
  WriteControlTable.msg
)

add_service_files(
  FILES
  GetJointModule.srv
  SetJointModule.srv
  SetModule.srv
  LoadOffset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
