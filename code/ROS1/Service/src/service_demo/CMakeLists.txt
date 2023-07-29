cmake_minimum_required(VERSION 3.0.2)
project(service_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Generate services in the 'srv' folder
add_service_files(
  # 默认的文件夹为srv
  DIRECTORY srv
  FILES AddTwoInts.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES service_demo
#  CATKIN_DEPENDS rospy rospy_tutorials std_msgs
#  DEPENDS system_lib
)

