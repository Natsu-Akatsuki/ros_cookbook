cmake_minimum_required(VERSION 3.11)
project(perception_msgs)
include(cmake/Color.cmake)

# 检查ROS版本
if (DEFINED ENV{ROS_VERSION})
  if ($ENV{ROS_VERSION} STREQUAL 1)
    message(STATUS "${Green}ROS1 is available!${ColourReset}")
    include(cmake/ROS1.cmake)
  elseif ($ENV{ROS_VERSION} STREQUAL 2)
    message(STATUS "${Green}ROS2 is available!${ColourReset}")
    include(cmake/ROS2.cmake)
  endif ()
endif ()

