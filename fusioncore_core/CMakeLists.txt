cmake_minimum_required(VERSION 3.16)

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

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
