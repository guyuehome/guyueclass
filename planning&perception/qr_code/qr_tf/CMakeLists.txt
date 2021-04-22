cmake_minimum_required(VERSION 2.8.3)
project(qr_tf)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package (OpenCV)
find_package (ZBar0)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs tf)

find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})
find_library(ZBAR_LIBRARIES NAMES zbar 
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS} )
find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZBAR  DEFAULT_MSG  ZBAR_LIBRARIES ZBAR_INCLUDE_DIR)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(qr_tf 
      src/qr_tf.cpp
      src/qr_tf_node.cpp
      include/qr_tf/qr_tf.h
)

target_link_libraries(qr_tf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
  libdmtx.so
)
