cmake_minimum_required(VERSION 3.16)
project(fusioncore_core VERSION 0.3.7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(fusioncore_core STATIC
  src/ukf.cpp
  src/fusioncore.cpp
  src/motion_model.cpp
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

  # TIMEOUT: this file holds integration-style tests that drive a simulated robot
  # for tens of seconds each, and it measured 62.8 s on a Release desktop build.
  # The ctest default is 60, so it was failing on the clock rather than on any
  # assertion. Give it real headroom; a suite that fails on timing is a suite
  # nobody trusts.
  ament_add_gtest(test_gnss tests/test_gnss.cpp TIMEOUT 600)
  target_link_libraries(test_gnss fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_gnss_manager tests/test_gnss_manager.cpp)
  target_link_libraries(test_gnss_manager fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_vslam tests/test_vslam.cpp)
  target_link_libraries(test_vslam fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_magnetometer tests/test_magnetometer.cpp TIMEOUT 300)
  target_link_libraries(test_magnetometer fusioncore_core Eigen3::Eigen)

  # Long timeout: these are integration-style runs (tens of seconds of simulated
  # 100 Hz drive each), and the file now holds seven of them. Measured 186 s on a
  # Release desktop build, so 300 left only 38% headroom and a slower CI runner
  # would have started failing intermittently, which is worse than a slow test
  # because nobody trusts a flaky suite. 600 gives 3x headroom.
  ament_add_gtest(test_gnss_coast tests/test_gnss_coast.cpp TIMEOUT 600)
  target_link_libraries(test_gnss_coast fusioncore_core Eigen3::Eigen)

  ament_add_gtest(test_clock_skew tests/test_clock_skew.cpp TIMEOUT 300)
  target_link_libraries(test_clock_skew fusioncore_core Eigen3::Eigen)
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
