# CMake and Package

## Macro

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29">catkin_package()</a>
    </summary>

服务于 catkin build system 的 CMake 宏，生成可以被其他 package 可以调用和利用的 .config 或 .cmake 文件，以让他们可以通过 find_package() 来使用当前包的库文件、头文件和依赖，具体是能得到这个包的相关信息，如头文件、库、依赖的相关位置

当且仅当这个`package`需要被其他包用到时，比如使用这个`package`的库文件和头文件才需要提供详细的信息，如果只是纯粹将可执行文件导出到`devel`目录的话则只需要调用 `catkin_package()`

实测在 ROS1 中其并不会将当前的 include 等文件夹拷贝到 devel 目录中，还是需要使用 install() 命令来完成

必须要在声明 targets 前（即使用 `add_library()` 或 `add_executable()` 前）调用该宏

```cmake
catkin_package(
  INCLUDE_DIRS      # 导出头文件
    include
  LIBRARIES         # 导出库文件
    euclidean_cluster
  CATKIN_DEPENDS    # 导出当前包依赖的CATKIN包，以被调用方继承
    pcl_ros
    roscpp
    sensor_msgs
    nodelet
    pcl_conversions
    autoware_perception_msgs
)
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        ament auto 系列宏
    </summary>

为 ament 的高级封装，具体参考 [Here](https://zhuanlan.zhihu.com/p/438191834)

```cmake
find_package(ament_cmake_auto REQUIRED)

# automatically link the dependency according to the xml (without find_package)
ament_auto_find_build_dependencies()

# 生成目标文件
ament_auto_add_library(listener_node SHARED src/listener_node.cpp)
ament_auto_add_executable(listener_node_exe src/listener_main.cpp)

# replace the export, install and ament_package command
ament_auto_package()
```

</details>

## CMakeLists.txt

<details>
    <summary>:wrench: <b>用例 1：</b>
        CMakeLists.txt
    </summary>

大型工程配置参考 [Here](https://github.com/Natsu-Akatsuki/RangeNetTrt8)

<!-- tabs:start -->

#### **ROS2**

具体参考 [Here](https://docs.ros.org/en/iron/How-To-Guides/Ament-CMake-Documentation.html)

由于 ament_auto_find_build_dependencies() 命令会根据 package.xml 导入相关的依赖，所以一定得有 package.xml 文件

当前的最佳实践是基于 targets 的导入，因此推荐使用 ament_target_dependencies() 命令来替代 target_link_library() 和 include_directory() 命令

```cmake
cmake_minimum_required(VERSION 3.8)  
project(<工程名>)

# >>> 通用配置 >>>
# 设置优化等级
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0"))
# 设置标准
set(CMAKE_CXX_STANDARD 17)
# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# >>> 导入三方包 >>>
find_package(PCL REQUIRED)

# >>> 导入 ROS 包 >>>
find_package(ament_cmake_auto REQUIRED)

# >>> 查找相关的依赖 >>>
ament_auto_find_build_dependencies()
  
# >>> 构建目标文件 >>>
ament_auto_add_executable(<target_name> <sources>)
ament_auto_add_library(<target_name> <sources>)
ament_target_dependencies(<target_name> <pkg_name|library_name>)
  
# >>> 导出相关的配置文件和进行安装 >>>
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz  
)
```

#### **ROS1**

```cmake
cmake_minimum_required(VERSION 3.8)
project(<工程名>)

# >>> 通用配置 >>>
# 设置优化等级
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0"))
# 设置标准
set(CMAKE_CXX_STANDARD 17)
# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# >>> 导入三方包 >>>
find_package(PCL REQUIRED)

# >>> 导入 ROS 包 >>>
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
)

# >>> 导出相关的配置文件 >>>
catkin_package(
  INCLUDE_DIRS      # 导出当前的头文件以被其他ROS包调用
  include
  LIBRARIES         # 导出当前生成的库文件以被其他ROS包调用
  route_handler
  CATKIN_DEPENDS    # 导出当前的ROS依赖，以被其他ROS包继承
  roscpp    
  pcl_ros
  sensor_msgs
  DEPENDS 
  PCL
)

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_library(route_handler SHARED
  src/route_handler.cpp
  )

target_link_libraries(route_handler ${catkin_LIBRARIES})
```

<!-- tabs:end -->

</details>

## Package.xml

<details>
    <summary>:wrench: <b>用例 1：</b>
        package.xml
    </summary>

```xml

<package format="3">
    <name>包名</name>
    <version>0.1.0</version>
    <description>包描述</description>
    <maintainer email="hong877381@gmail.com">Natsu-Akatsuki</maintainer>
    <license>Apache License 2.0</license>

    <!-- （mandatory）ROS1 -->
    <buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
    <depend condition="$ROS_VERSION == 1">roscpp</depend>
    <!-- （optional）有关消息类型的生成 -->
    <build_depend condition="$ROS_VERSION == 1">message_generation</build_depend>
    <exec_depend condition="$ROS_VERSION == 1">message_runtime</exec_depend>

    <!-- （mandatory）ROS2 -->
    <buildtool_depend condition="$ROS_VERSION == 2">ament_cmake</buildtool_depend>
    <depend condition="$ROS_VERSION == 2">rclcpp</depend>
    <member_of_group condition="$ROS_VERSION == 2">rosidl_interface_packages</member_of_group>
    <!-- （optional）有关消息类型的生成 -->
    <build_depend condition="$ROS_VERSION == 2">rosidl_default_generators</build_depend>
    <exec_depend condition="$ROS_VERSION == 2">rosidl_default_runtime</exec_depend>

    <!-- （mandatory）共性部分 -->
    <depend>geometry_msgs</depend>
    <depend>nav_msgs</depend>
    <depend>pcl_ros</depend>
    <depend>sensor_msgs</depend>
    <depend>std_msgs</depend>
    <depend>tf2</depend>
    <depend>libpcl-all-dev</depend>

    <export>
        <!-- （mandatory）不添加或无法暴露包名，从而影响 ROS2 launch时包名的查找 -->
        <build_type condition="$ROS_VERSION == 2">ament_cmake</build_type>
    </export>
</package>

```

常用可选项：

1）ROS 依赖：[geometry_msgs](http://wiki.ros.org/geometry_msgs)，std_msgs, visualization_msgs，tf2_ros, tf2_geometry_msgs，roscpp \
2）系统依赖：pugixml-dev，range-v3

</details>

## Reference

- CMakeLists.txt 官方文档 for [ROS1](https://wiki.ros.org/catkin/CMakeLists.txt)
- [Migrating from format 1 to format 2 for package.xml](https://docs.ros.org/en/melodic/api/catkin/html/howto/format2/migrating_from_format_1.html#migrating-from-format1-to-format2)