cmake_minimum_required(VERSION 3.16)
project(fusioncore_core VERSION 0.2.2)

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
  $<INSTALL_INTERFACE:include/fusioncore_core>
)

target_link_libraries(fusioncore_core PUBLIC Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ukf tests/test_ukf.cpp)
  target_link_libraries(test_ukf fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_imu tests/test_imu.cpp)
  target_link_libraries(test_imu fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_encoder tests/test_encoder.cpp)
  target_link_libraries(test_encoder fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_fusioncore tests/test_fusioncore.cpp)
  target_link_libraries(test_fusioncore fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_gnss tests/test_gnss.cpp)
  target_link_libraries(test_gnss fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_gnss_manager tests/test_gnss_manager.cpp)
  target_link_libraries(test_gnss_manager fusioncore_core Eigen3::Eigen)
endif()

ament_export_targets(fusioncore_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_export_include_directories(include/fusioncore_core)

install(
  DIRECTORY include/
  DESTINATION include/fusioncore_core
)

install(
  TARGETS fusioncore_core
  EXPORT fusioncore_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
