cmake_minimum_required(VERSION 3.16)
project(fusioncore_ublox VERSION 0.3.3)

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
