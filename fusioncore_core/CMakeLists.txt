cmake_minimum_required(VERSION 3.16)
project(fusioncore_core VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(fusioncore_core STATIC
  src/ukf.cpp
  src/fusioncore.cpp
)

target_include_directories(fusioncore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fusioncore_core PUBLIC Eigen3::Eigen)

ament_export_targets(fusioncore_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS fusioncore_core
  EXPORT fusioncore_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_package()
