cmake_minimum_required(VERSION 3.8)
project(fusioncore_datasets)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/nclt_player.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
