cmake_minimum_required(VERSION 3.16)
project(fusioncore_ublox VERSION 0.3.8)

# Default to Release when the caller has not chosen. A plain "colcon build" sets
# no build type, which means no optimisation flags at all, and Eigen without
# inlining is drastically slower: the consistency suite took 298 s that way and
# 4.3 s at -O3, a factor of 69. That is the difference between a 23-state UKF
# holding 100 Hz on a Raspberry Pi and starving. An explicit -DCMAKE_BUILD_TYPE
# still wins, so debug builds are unaffected.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ublox_msgs QUIET)
if(ublox_msgs_FOUND)
  add_executable(gnss_doppler_bridge src/gnss_doppler_bridge.cpp)
  ament_target_dependencies(gnss_doppler_bridge rclcpp nav_msgs std_msgs ublox_msgs)
  install(TARGETS gnss_doppler_bridge DESTINATION lib/${PROJECT_NAME})
  message(STATUS "fusioncore_ublox: ublox_msgs found, gnss_doppler_bridge will be built")
else()
  message(STATUS "fusioncore_ublox: ublox_msgs not found, gnss_doppler_bridge skipped")
  message(STATUS "  To enable: sudo apt install ros-$ENV{ROS_DISTRO}-ublox-msgs")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bridge_math test/test_bridge_math.py)
endif()

ament_package()
