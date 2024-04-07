cmake_minimum_required(VERSION 3.5)
project(rviz_animated_view_controller)

find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED system filesystem)

set(CMAKE_BUILD_TYPE Release)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

ament_auto_add_library(rviz_animated_view_controller SHARED
  src/rviz_animated_view_controller.cpp
)

target_link_libraries(rviz_animated_view_controller ${QT_LIBRARIES})

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)
ament_auto_package(
  INSTALL_TO_SHARE
)