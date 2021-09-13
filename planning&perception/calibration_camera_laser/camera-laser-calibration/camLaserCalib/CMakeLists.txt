cmake_minimum_required(VERSION 2.8.3)
project(cam_laser_calib)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  message_filters
  tf_conversions
  tf
  laser_geometry
  dynamic_reconfigure
  cv_bridge
)

generate_dynamic_reconfigure_options(
        cfg/cam_laser_calib.cfg
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

 add_executable(cam_laser_calib_node
        src/camLaserCalib_node.cpp
        src/camLaserCalib.cpp
        src/pcl_util.cpp
		src/Marker.cpp
		src/MarkerDetector.cpp
		

        include/camLaserCalib.h
        include/pcl_util.h
		include/Marker.h
		include/MarkerDetector.h)

 add_dependencies(cam_laser_calib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(cam_laser_calib_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
