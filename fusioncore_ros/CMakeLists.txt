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
find_package(compass_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fusioncore_core REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PROJ REQUIRED)

add_executable(fusioncore_node
  src/fusion_node.cpp
)

target_include_directories(fusioncore_node PRIVATE
  ${fusioncore_core_INCLUDE_DIRS}
)

target_link_directories(fusioncore_node PRIVATE
  ${fusioncore_core_LIB_DEPENDS}
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
  fusioncore_core
  compass_msgs
  diagnostic_msgs
  std_srvs
)

target_link_libraries(fusioncore_node
  Eigen3::Eigen
  PROJ::proj
)

# Explicitly link the static library
find_library(FUSIONCORE_CORE_LIB
  NAMES fusioncore_core
  HINTS ${fusioncore_core_DIR}/../../../lib
)
if(FUSIONCORE_CORE_LIB)
  target_link_libraries(fusioncore_node ${FUSIONCORE_CORE_LIB})
else()
  message(FATAL_ERROR "Could not find libfusioncore_core.a")
endif()

install(TARGETS fusioncore_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_proj_axis_order tests/test_proj_axis_order.cpp)
  target_link_libraries(test_proj_axis_order PROJ::proj)
endif()

ament_package()
