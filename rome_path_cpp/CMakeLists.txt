cmake_minimum_required(VERSION 3.5)
project(rome_path_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(follow_path src/follow_path.cpp)
ament_target_dependencies(follow_path
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  control_msgs
)

target_link_libraries(follow_path yaml-cpp)

install(TARGETS follow_path
  DESTINATION lib/${PROJECT_NAME}
)

# Install CSV files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
