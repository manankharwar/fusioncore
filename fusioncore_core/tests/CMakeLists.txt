include(FetchContent)

# Download GoogleTest automatically
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

# UKF test binary
add_executable(test_ukf test_ukf.cpp)

target_link_libraries(test_ukf
  fusioncore_core
  GTest::gtest_main
)

# Find Eigen
find_package(Eigen3 REQUIRED)
target_link_libraries(test_ukf Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(test_ukf)
