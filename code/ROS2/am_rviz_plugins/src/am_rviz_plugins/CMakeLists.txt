cmake_minimum_required(VERSION 3.11)
project(am_rviz_plugins)

# >>> 通用配置 >>>
# 设置优化等级
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

# 设置标准
set(CMAKE_CXX_STANDARD 17)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_AUTOMOC ON)

# >>> 导入三方包 >>>
find_package(Qt5 REQUIRED Core Widgets)
find_package(OpenCV REQUIRED)
find_package(FMT REQUIRED)

# >>> 导入 ROS 包 >>>
find_package(ament_cmake_auto REQUIRED)

# >>> 查找相关的依赖 >>>
ament_auto_find_build_dependencies()

# >>> 生成自定义消息类型 >>>
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoundingBox.msg
  msg/BoundingBoxArray.msg
  msg/OverlayMenu.msg
  DEPENDENCIES
  geometry_msgs std_msgs
)

# >>> 构建目标文件 >>>
ament_auto_add_library(rviz_plugins SHARED
  src/log_panel.cpp
  src/log_display.cpp
  src/bounding_box_array_display.cpp
  src/bounding_box_display.cpp
  src/polar_grid_display.cpp
  src/bev_controller.cpp
)

# This CMake code is only required when you want to use interfaces in the same package as the one in which they are defined.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rviz_plugins "${cpp_typesupport_target}")

# 指定第三方库依赖（等价于 target_link_library()）
ament_target_dependencies(rviz_plugins
  OpenCV
  Qt5)

# 导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

# >>> 导出相关的配置文件和进行安装 >>>
ament_auto_package(
  INSTALL_TO_SHARE
  icons
)