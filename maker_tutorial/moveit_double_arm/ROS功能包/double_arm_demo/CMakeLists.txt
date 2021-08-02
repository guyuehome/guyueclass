cmake_minimum_required(VERSION 2.8.3)
project(double_arm_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES probot_demo
   CATKIN_DEPENDS roscpp rospy moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS})


add_executable(moveit_fk_demo src/moveit_fk_demo.cpp)
target_link_libraries(moveit_fk_demo ${catkin_LIBRARIES})

add_executable(moveit_ik_demo src/moveit_ik_demo.cpp)
target_link_libraries(moveit_ik_demo ${catkin_LIBRARIES})

add_executable(moveit_cartesian_demo src/moveit_cartesian_demo.cpp)
target_link_libraries(moveit_cartesian_demo ${catkin_LIBRARIES})


add_executable(moveit_collision_demo src/moveit_collision_demo.cpp)
target_link_libraries(moveit_collision_demo ${catkin_LIBRARIES})

install(PROGRAMS 
   scripts/moveit_fk_demo.py
   scripts/moveit_ik_demo.py
   scripts/moveit_cartesian_demo.py
   scripts/moveit_attached_object_demo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
   moveit_collision_demo 
   moveit_fk_demo 
   moveit_ik_demo 
   moveit_cartesian_demo
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
