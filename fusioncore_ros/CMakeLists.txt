cmake_minimum_required(VERSION 3.16)
project(fusioncore_ros VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fusioncore_core REQUIRED)

add_executable(fusioncore_node
  src/fusion_node.cpp
)

target_include_directories(fusioncore_node PRIVATE
  include
)

ament_target_dependencies(fusioncore_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(fusioncore_node
  fusioncore_core
  Eigen3::Eigen
)

install(TARGETS fusioncore_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
