
cmake_minimum_required(VERSION 2.8.3)
project(yobotics_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

