CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(PROJECT_NAME solvepnp)
PROJECT(${PROJECT_NAME})
SET( CMAKE_BUILD_TYPE Debug )
SET( CMAKE_CXX_COMPILER "g++" )
SET( CMAKE_CXX_FLAGS "-std=c++0x" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add OpenCV library
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
	#MESSAGE(STATUS "\n ~~~~~~~~OpenCV_FOUND~~~~~~~~~~\n")
	include_directories(${OpenCV_INCLUDE_DIRS})
	# message("OpenCV_INCLUDE_DIRS : "  ${OpenCV_INCLUDE_DIRS}"\n" )
         message("OpenCV_VERSION : "  ${OpenCV_VERSION} \n )
	link_directories(${OpenCV_LIBRARY_DIRS})
endif()

LIST(APPEND LINK_LIBS
    ${OpenCV_LIBS}           #this should be OpenCV_LIBS or it will not work
)
LIST(APPEND SOURCEFILES
    src/solvepnp.cpp
)
add_executable(solvepnp ${SOURCEFILES})
target_link_libraries(solvepnp ${LINK_LIBS})



