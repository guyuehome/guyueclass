cmake_minimum_required(VERSION 2.8.3)
project(laikago_gazebo)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    laikago_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    laikago_msgs 
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
)

add_dependencies(${PROJECT_NAME} laikago_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

add_library(LaikagoFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(LaikagoFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(LaikagoDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
target_link_libraries(LaikagoDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(laikago_external_force src/exe/external_force.cpp)
target_link_libraries(laikago_external_force ${catkin_LIBRARIES})

add_executable(laikago_servo src/exe/servo.cpp)
target_link_libraries(laikago_servo ${PROJECT_NAME} ${catkin_LIBRARIES})


