cmake_minimum_required(VERSION 3.5.0)
project(view_controller_msgs)

set(CMAKE_BUILD_TYPE RELEASE)

set(MSG_DEPS
  std_msgs
  geometry_msgs)

set(MSG_FILES
  msg/CameraPlacement.msg
  msg/CameraMovement.msg
  msg/CameraTrajectory.msg
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  geometry_msgs std_msgs
)

ament_auto_package(
  INSTALL_TO_SHARE
  )