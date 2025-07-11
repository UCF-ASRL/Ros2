cmake_minimum_required(VERSION 3.16)
project(omniwheel_drive_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
  -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
  -Werror=missing-braces)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(omniwheel_drive_controller_parameters
  src/omniwheel_drive_controller.yaml
)

add_library(
  omniwheel_drive_controller
  SHARED
  src/omniwheel_drive_controller.cpp
  src/odometry.cpp
)
target_compile_features(omniwheel_drive_controller PUBLIC cxx_std_17)
target_include_directories(omniwheel_drive_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(omniwheel_drive_controller PUBLIC
  omniwheel_drive_controller_parameters)
ament_target_dependencies(omniwheel_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface omniwheel_drive_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)


  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_omniwheel_drive_controller test/test_load_omniwheel_drive_controller.cpp)
  ament_target_dependencies(test_load_omniwheel_drive_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_omniwheel_drive_controller test/test_omniwheel_drive_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/omniwheel_drive_controller_params.yaml)
  target_include_directories(test_omniwheel_drive_controller PRIVATE include)
  target_link_libraries(test_omniwheel_drive_controller omniwheel_drive_controller)
  ament_target_dependencies(
    test_omniwheel_drive_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_omniwheel_drive_controller_preceeding test/test_omniwheel_drive_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/omniwheel_drive_controller_preceeding_params.yaml)
  target_include_directories(test_omniwheel_drive_controller_preceeding PRIVATE include)
  target_link_libraries(test_omniwheel_drive_controller_preceeding omniwheel_drive_controller)
  ament_target_dependencies(
    test_omniwheel_drive_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/omniwheel_drive_controller
)

install(
  TARGETS omniwheel_drive_controller omniwheel_drive_controller_parameters
  EXPORT export_omniwheel_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_omniwheel_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
