##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(op3_gui_demo)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  cmake_modules
  eigen_conversions
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_online_walking_module_msgs
  humanoid_nav_msgs
)

find_package(Eigen3 REQUIRED)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(op3_gui_demo_SRCS
  src/main.cpp
  src/main_window.cpp
  src/preview_walking_form.cpp
  src/qnode.cpp
  src/qnode_default_demo.cpp
  src/qnode_preview_walking.cpp
)

set(op3_gui_demo_HDRS
  include/op3_gui_demo/main_window.hpp
  include/op3_gui_demo/preview_walking_form.h
  include/op3_gui_demo/qnode.hpp
)

set(op3_gui_demo_UIS
  ui/main_window.ui
  ui/preview_walking_form.ui
)

set(op3_gui_demo_RESOURCES
    resources/images.qrc
)

set(op3_gui_demo_INCLUDE_DIRECTORIES
  include
  include/${PROJECT_NAME}
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
endif()

catkin_package(
  INCLUDE_DIRS ${op3_gui_demo_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui  rqt_gui_cpp  roscpp  roslib  std_msgs  sensor_msgs  geometry_msgs  visualization_msgs  interactive_markers  cmake_modules  eigen_conversions  robotis_controller_msgs  op3_walking_module_msgs  op3_online_walking_module_msgs  humanoid_nav_msgs
)
#catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(op3_gui_demo_MOCS ${op3_gui_demo_HDRS})
else()
  qt4_wrap_cpp(op3_gui_demo_MOCS ${op3_gui_demo_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(op3_gui_demo_UIS_H ${op3_gui_demo_UIS})
else()
  qt4_wrap_ui(op3_gui_demo_UIS_H ${op3_gui_demo_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_add_resources(op3_gui_demo_RCC ${op3_gui_demo_RESOURCES})
else()
  qt4_add_resources(op3_gui_demo_RCC ${op3_gui_demo_RESOURCES})
endif()


include_directories(
  ${op3_gui_demo_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} ${op3_gui_demo_SRCS} ${op3_gui_demo_MOCS} ${op3_gui_demo_UIS_H} ${op3_gui_demo_RCC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} yaml-cpp)
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/op3_gui_demo/*.hpp include/op3_gui_demo/*.h)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

#add_executable(op3_gui_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#add_dependencies(op3_gui_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(op3_gui_demo
#  ${QT_LIBRARIES}
#  ${catkin_LIBRARIES}
#  yaml-cpp)
#install(TARGETS op3_gui_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
