cmake_minimum_required(VERSION 3.0.2)
project(limo_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  LimoStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(limo_base_node
  src/limo_base_node.cpp
  src/limo_driver.cpp
  src/serial_port.cpp
)

add_dependencies(limo_base_node limo_base_generate_messages_cpp)

target_link_libraries(limo_base_node
  ${catkin_LIBRARIES}
)

install(TARGETS limo_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
