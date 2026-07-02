cmake_minimum_required(VERSION 3.8)
project(quadruped_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY xacro meshes launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
