cmake_minimum_required(VERSION 3.16)
project(rovio)

# Compile settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_VALGRIND_COMPATIBILITY "Disable AVX-512 to allow Valgrind memory profiling" OFF)

# Architecture-specific optimization
if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64|ARM64")
    message(STATUS "Building for L4T/ARM64 system with high-performance compile flags")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mtune=cortex-a76 -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays")
else()
    message(STATUS "Building for x86/non-arm system")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
    if(ENABLE_VALGRIND_COMPATIBILITY)
        message(STATUS "Valgrind compatibility mode ON: Disabling AVX-512 instructions")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-avx512f")
    endif()
endif()

# ROVIO Configuration
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image levels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(
    -DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE}
    -DROVIO_NCAM=${ROVIO_NCAM}
    -DROVIO_NLEVELS=${ROVIO_NLEVELS}
    -DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE}
    -DROVIO_NPOSE=${ROVIO_NPOSE}
)

# Build the math submodule inline
add_subdirectory(lightweight_filtering)
include_directories(${PROJECT_SOURCE_DIR}/lightweight_filtering/include)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rovio_interfaces REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

# Includes
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${YamlCpp_INCLUDE_DIRS}
)

# Core library
set(ROVIO_SRC
    src/Camera.cpp
    src/FeatureCoordinates.cpp
    src/FeatureDistance.cpp
)

# Optional scene rendering
if(MAKE_SCENE)
    message(STATUS "Building ROVIO with openGL Scene Visualization")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAKE_SCENE=1")
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
    list(APPEND ROVIO_SRC src/Scene.cpp)
endif()

add_library(${PROJECT_NAME}_lib ${ROVIO_SRC})

# Link external C++ libraries
target_link_libraries(${PROJECT_NAME}_lib
    ${YamlCpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
if(MAKE_SCENE)
    target_link_libraries(${PROJECT_NAME}_lib ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES})
endif()

# Link ROS2 libraries
ament_target_dependencies(${PROJECT_NAME}_lib
    rclcpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport
    rosbag2_cpp
    rosbag2_storage
    rovio_interfaces
)

# Executables
add_executable(rovio_node src/rovio_node.cpp)
target_link_libraries(rovio_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rovio_node rclcpp rovio_interfaces)

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
target_link_libraries(feature_tracker_node ${PROJECT_NAME}_lib)
ament_target_dependencies(feature_tracker_node rclcpp rovio_interfaces)

add_executable(rovio_rosbag_loader src/rovio_rosbag_loader.cpp)
target_link_libraries(rovio_rosbag_loader ${PROJECT_NAME}_lib)
ament_target_dependencies(rovio_rosbag_loader rclcpp rosbag2_cpp rosbag2_storage rovio_interfaces)

# Install targets
install(TARGETS
    ${PROJECT_NAME}_lib
    rovio_node
    feature_tracker_node
    rovio_rosbag_loader
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
    DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY shaders/
    DESTINATION share/${PROJECT_NAME}/shaders
)

# Export for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
    rclcpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport
    rovio_interfaces
)

# Tests
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Test Patch
    ament_add_gtest(test_patch src/test_patch.cpp)
    target_link_libraries(test_patch ${PROJECT_NAME}_lib)

    # Test MLP (Multilevel Patch)
    ament_add_gtest(test_mlp src/test_mlp.cpp)
    target_link_libraries(test_mlp ${PROJECT_NAME}_lib)
endif()

ament_package()
