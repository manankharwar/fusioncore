include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

find_package(Eigen3 REQUIRED)

add_executable(test_ukf test_ukf.cpp)
target_link_libraries(test_ukf fusioncore_core GTest::gtest_main Eigen3::Eigen)

add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu fusioncore_core GTest::gtest_main Eigen3::Eigen)

add_executable(test_encoder test_encoder.cpp)
target_link_libraries(test_encoder fusioncore_core GTest::gtest_main Eigen3::Eigen)

add_executable(test_fusioncore test_fusioncore.cpp)
target_link_libraries(test_fusioncore fusioncore_core GTest::gtest_main Eigen3::Eigen)

add_executable(test_gnss test_gnss.cpp)
target_link_libraries(test_gnss fusioncore_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(test_ukf)
gtest_discover_tests(test_imu)
gtest_discover_tests(test_encoder)
gtest_discover_tests(test_fusioncore)
gtest_discover_tests(test_gnss)

add_executable(test_gnss_manager test_gnss_manager.cpp)
target_link_libraries(test_gnss_manager fusioncore_core GTest::gtest_main Eigen3::Eigen)
gtest_discover_tests(test_gnss_manager)
