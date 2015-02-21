cmake_minimum_required (VERSION 2.6)
project(rovio)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3")

##################### Find, include, and compile library #####################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(1)
	find_package(OpenMP REQUIRED)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

##################### Install ROS stuff #####################
find_package(catkin REQUIRED COMPONENTS
	lightweight_filtering
  	kindr
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
  	kindr
	roscpp
	roslib
	cv_bridge
)

add_executable(rovioNode src/rovioNode.cpp)
target_link_libraries(rovioNode ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS})

add_executable(test_rovio src/test_rovio.cpp)
target_link_libraries(test_rovio ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS})
