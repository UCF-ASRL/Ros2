cmake_minimum_required(VERSION 3.5)
project(roam_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
