cmake_minimum_required(VERSION 3.8)
project(kahuil_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch config worlds rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
