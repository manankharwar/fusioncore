cmake_minimum_required(VERSION 3.16)
project(fusioncore VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Build options
option(BUILD_ROS "Build ROS 2 wrapper" OFF)
option(BUILD_TESTS "Build unit tests" ON)

# Core library (no ROS dependency)
add_subdirectory(fusioncore_core)

# ROS 2 wrapper (optional)
if(BUILD_ROS)
  add_subdirectory(fusioncore_ros)
endif()
