find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/object_recognition/ImageDecision.msg"
  "msg/object_recognition/ObjectInImage.msg"
  "msg/object_recognition/ObjectsInImage.msg"
  DEPENDENCIES
  std_msgs
  )

ament_auto_package()
