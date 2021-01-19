include_directories(${catkin_INCLUDE_DIRS})
catkin_add_gtest(vision_msgs_test main.cpp)
add_dependencies(vision_msgs_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
