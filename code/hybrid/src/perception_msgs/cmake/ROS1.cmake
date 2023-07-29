find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  )

add_message_files(
  DIRECTORY msg/object_recognition/
  FILES
  ImageDecision.msg
  ObjectInImage.msg
  ObjectsInImage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
