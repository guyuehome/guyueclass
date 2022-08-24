cmake_minimum_required(VERSION 2.8.3)
project(loam_livox)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


# set(CMAKE_BUILD_TYPE DEBUG)
# set(CMAKE_C_FLAGS_DEBUG   " -O0 -g")    
# set(CMAKE_CXX_FLAGS_DEBUG " -O0 -g")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


#list(APPEND CMAKE_PREFIX_PATH "/home/jxl/third_softwares/pcl-1.9.1/INSTALL/")
#set(CMAKE_PREFIX_PATH "/home/jxl/third_softwares/pcl-1.9.1/INSTALL/")

set(PCL_DIR "/home/jxl/third_softwares/pcl-1.9.1/INSTALL/share/pcl-1.9/")  #use locall installed pcl

find_package(PCL 1.9 REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./include/tools/
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)


add_executable(livox_scanRegistration source/laser_feature_extractor.cpp)
target_link_libraries(livox_scanRegistration ${catkin_LIBRARIES} 
                          ${PCL_LIBRARIES}
                          ${OpenCV_LIBS})

add_executable(livox_laserMapping source/laser_mapping.cpp )
target_link_libraries(livox_laserMapping ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
   ${CERES_LIBRARIES} )

add_executable(read_camera source/read_camera.cpp)
target_link_libraries(read_camera ${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
	${CERES_LIBRARIES} ${OpenCV_LIBS})


