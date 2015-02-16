cmake_minimum_required (VERSION 2.6)
project(rovio)
add_definitions(-std=c++0x)
add_definitions(-O3)

##################### Set some variables #####################
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


##################### Find, include, and compile library #####################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Kindr REQUIRED)
include_directories(./include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Kindr_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)


##################### Install ROS stuff #####################
find_package(catkin REQUIRED COMPONENTS
	lightweight_filtering
	roscpp
	roslib
	cv_bridge
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	lightweight_filtering 
	roscpp
	roslib
	cv_bridge
)

add_executable(rovioNode src/rovioNode.cpp)
target_link_libraries(rovioNode ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})

add_executable(test_rovio src/test_rovio.cpp)
target_link_libraries(test_rovio ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES})
