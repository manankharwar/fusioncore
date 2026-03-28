include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)

ament_add_gtest(test_ukf test_ukf.cpp)
target_link_libraries(test_ukf fusioncore_core Eigen3::Eigen)

ament_add_gtest(test_imu test_imu.cpp)
target_link_libraries(test_imu fusioncore_core Eigen3::Eigen)

ament_add_gtest(test_encoder test_encoder.cpp)
target_link_libraries(test_encoder fusioncore_core Eigen3::Eigen)

ament_add_gtest(test_fusioncore test_fusioncore.cpp)
target_link_libraries(test_fusioncore fusioncore_core Eigen3::Eigen)

ament_add_gtest(test_gnss test_gnss.cpp)
target_link_libraries(test_gnss fusioncore_core Eigen3::Eigen)

ament_add_gtest(test_gnss_manager test_gnss_manager.cpp)
target_link_libraries(test_gnss_manager fusioncore_core Eigen3::Eigen)
