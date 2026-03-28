cmake_minimum_required(VERSION 3.16)
project(fusioncore_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/gz_pose_to_gps.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_pose_to_gps
)

install(PROGRAMS
  launch/integration_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME integration_test
)

ament_package()
