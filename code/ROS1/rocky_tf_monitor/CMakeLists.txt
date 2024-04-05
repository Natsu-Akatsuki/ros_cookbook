cmake_minimum_required(VERSION 3.0.2)
project(rocky_tf_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES rocky_tf_monitor
  #  CATKIN_DEPENDS rospy tf2_msgs
  #  DEPENDS system_lib
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/rocky_tf_monitor.py
  # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  #
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)