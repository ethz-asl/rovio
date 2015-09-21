cmake_minimum_required (VERSION 2.6)
project(rovio)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -march=native")

##################### Find, include, and compile library #####################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(0)
	find_package(OpenMP REQUIRED)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if(MAKE_SCENE)
	message(STATUS "Building ROVIO with openGL Scene Visualization")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAKE_SCENE=1")
	find_package(GLEW REQUIRED)
	include_directories(${GLEW_INCLUDE_DIRS})

	find_package(GLUT REQUIRED)
	include_directories(${GLUT_INCLUDE_DIRS})
	link_directories(${GLUT_LIBRARY_DIRS})
	add_definitions(${GLUT_DEFINITIONS})

	find_package(OpenGL REQUIRED)
	include_directories(${OpenGL_INCLUDE_DIRS})
	link_directories(${OpenGL_LIBRARY_DIRS})
	add_definitions(${OpenGL_DEFINITIONS})
endif()

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

##################### Install ROS stuff #####################
find_package(catkin REQUIRED COMPONENTS
	lightweight_filtering
  	kindr
	roscpp
	roslib
	cv_bridge
	message_generation
	nav_msgs
	geometry_msgs
	sensor_msgs
	std_msgs
	tf
)

add_message_files(
  FILES
  RovioOutput.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
	INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	lightweight_filtering
  	kindr
	roscpp
	roslib
	cv_bridge
	message_runtime
	nav_msgs
	geometry_msgs
	sensor_msgs
	std_msgs
	tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/test_rovio.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_rovio src/test_rovio.cpp)
target_link_libraries(test_rovio ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY})

add_executable(rovio_node src/rovio_node.cpp)
target_link_libraries(rovio_node ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS})

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
target_link_libraries(feature_tracker_node ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} ${OpenMP_EXE_LINKER_FLAGS})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	#add_subdirectory(gtest gtest)
	enable_testing()
	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
	add_executable(test_patch src/test_patch.cpp)
	target_link_libraries(test_patch gtest_main gtest pthread ${catkin_LIBRARIES})
	add_test(test_patch test_patch)
	add_executable(test_mlp src/test_mlp.cpp)
	target_link_libraries(test_mlp gtest_main gtest pthread ${catkin_LIBRARIES})
	add_test(test_mlp test_mlp)
endif()
