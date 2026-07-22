cmake_minimum_required(VERSION 3.16)
project(fusioncore_ros VERSION 0.3.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Non-ROS deps not picked up by ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(PROJ QUIET)
if(NOT PROJ_FOUND)
  # PROJ 8 (Ubuntu 22.04 / Humble) ships no CMake config; fall back to pkg-config
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(PROJ REQUIRED proj)
  if(NOT TARGET PROJ::proj)
    add_library(PROJ::proj INTERFACE IMPORTED)
    target_include_directories(PROJ::proj INTERFACE ${PROJ_INCLUDE_DIRS})
    target_link_libraries(PROJ::proj INTERFACE ${PROJ_LINK_LIBRARIES})
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "msg/GnssStatus.msg"
  "msg/FilterHealth.msg"
  DEPENDENCIES geographic_msgs geometry_msgs std_msgs
)

ament_auto_add_executable(fusioncore_node
  src/fusion_node.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fusioncore_node
  ${cpp_typesupport_target}
  Eigen3::Eigen
  PROJ::proj
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_proj_axis_order tests/test_proj_axis_order.cpp)
  target_link_libraries(test_proj_axis_order PROJ::proj)

  ament_add_gtest(test_imu_orientation_validity tests/test_imu_orientation_validity.cpp)
endif()

ament_auto_package()
