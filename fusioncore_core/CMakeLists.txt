cmake_minimum_required(VERSION 3.16)

add_library(fusioncore_core STATIC
  src/ukf.cpp
)

target_include_directories(fusioncore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
