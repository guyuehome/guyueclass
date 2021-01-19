cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  BoundingBox2D.msg
  BoundingBox3D.msg
  Classification2D.msg
  Classification3D.msg
  Detection2DArray.msg
  Detection2D.msg
  Detection3DArray.msg
  Detection3D.msg
  ObjectHypothesis.msg
  ObjectHypothesisWithPose.msg
  VisionInfo.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()