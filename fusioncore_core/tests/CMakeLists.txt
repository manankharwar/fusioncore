include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

find_package(Eigen3 REQUIRED)

# UKF tests
add_executable(test_ukf test_ukf.cpp)
target_link_libraries(test_ukf fusioncore_core GTest::gtest_main Eigen3::Eigen)

# IMU sensor tests
add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu fusioncore_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(test_ukf)
gtest_discover_tests(test_imu)
