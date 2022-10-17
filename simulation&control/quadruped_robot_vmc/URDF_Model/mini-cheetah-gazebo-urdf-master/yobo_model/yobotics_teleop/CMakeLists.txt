cmake_minimum_required(VERSION 2.8.3)
project(yobotics_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   yobotics_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

