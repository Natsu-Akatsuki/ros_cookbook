# RViz

在 ROS1 中不推荐使用 RViz 的 Python 拓展库（ROS2 中暂时还没提供封装），一是文档太少，开发成本大；二是还有一些尚待解决的问题。比如退出 Qt 应用程序后，RViz 节点将成为僵尸节点，不能被`rosnode kill`掉，只能使用`rosnode cleanup`清理，实测在 C++中不存在这个问题，进程可以退出得很干净；另外不能够在 Qt 的`RViz`中添加图像面板，否则会有段错误（暂无解决方案）

# Usage

## Docker

<details>
    <summary>:wrench: <b>用例 1：</b>
        在 Docker 容器中出现：OgreGLSupport.cpp:57: virtual void Ogre::GLSupport::initialiseExtensions(): Assertion `pcVer && "Problems getting GL version string using glGetString"' failed.
    </summary>

No OpenGL Support for nvidia render，于容器中检查一下命令行 `nvidia-smi` 是否有正常的输出

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        Noetic 容器下渲染 RViz会有很强的撕裂效果，具体参考 <a herf="https://github.com/ros-visualization/rviz/issues/1780">Here</a>
    </summary>

```bash
$ sudo add-apt-repository ppa:beineri/opt-qt-5.12.10-focal
$ sudo apt update
$ sudo apt install qt512charts-no-lgpl qt512svg qt512xmlpatterns qt512tools qt512translations    qt512graphicaleffects qt512quickcontrols2 qt512wayland qt512websockets qt512serialbus qt512serialport qt512location qt512imageformats qt512script qt512scxml qt512gamepad qt5123d 
$ source /opt/qt512/bin/qt512-env.sh
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        在容器中出现：qt.qpa.xcb: could not connect to display. qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found. This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem. Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx, xcb.
    </summary>

检查 `DISPLAY` 环境变量是否设置正确（此处的 could not connect to to display 是没有 :0 这些信息的）

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        Authorization required, but no authorization protocol specified qt.qpa.xcb: could not connect to display :0
    </summary>

追加访问权限

```bash
# 添加权限，使容器能访问宿主机的Xserver
$ xhost +
```

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        <a href="https://natsu-akatsuki.github.io/ambook/#/Computer%20Graphics/OpenGL">指定使用特定的显卡进行渲染</a>
    </summary>

如果使用 Intel Mesa 进行渲染，则还需要进行如下配置：具体参考 [Here](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)

```bash
$ xhost +
# 主要是添加/dev/dri
$ docker run \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY=$DISPLAY" \
  <image_name>
(container) $ sudo apt install libgl1-mesa-glx libgl1-mesa-dri
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20221129234927875.png)

</details>

## Marker

具体字段可看 [Here](http://wiki.ros.org/rviz/DisplayTypes/Marker)

|     Field     |                                                                                                                          说明                                                                                                                           |
|:-------------:|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|   lifetime    |                                                                              marker 的持续时间，[0 表示一直保留](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)                                                                              |
|     color     |                                                                                                           （mandated）子字段为：`rgba`，不指定则默认是透明状态                                                                                                           |
| mesh_resource | "package://turtlebot3_description/meshes/bases/burger_base.stl"<br />"file://<绝对路径>/meshes/cube.dae"<br />"https://storage.googleapis.com/foxglove-public-assets/Avocado.glb"<br />支持binary（.stl），Ogre（.mesh），COLLADA（.dae）, blend（.bld）（不支持mtl, glb） |
|    colors     |                                                                                                                只适用有 points 成员的 Markers                                                                                                                |

<details>
    <summary>:wrench: <b>用例 1：</b>
        发布新一帧的 marker 时，移除上一帧的 marker
    </summary>

RViz 的 marker 是叠加式显示的，除非被替换或者设置显示时间。可以参考如下代码，在下一次发布前，先发布一次清空 marker 的操作（相当于用空的 marker 进行替换）

```python
def clear_bounding_box_marker(stamp, identity, ns="uname", frame_id="lidar"):
    box_marker = Marker()
    box_marker.header.stamp = stamp
    box_marker.header.frame_id = frame_id

    box_marker.ns = ns
    box_marker.id = identity

    box_marker.action = Marker.DELETEALL
    if __ROS__VERSION__ == 1:
        box_marker.lifetime = rospy.Duration(0.02)
    elif __ROS__VERSION__ == 2:
        box_marker.lifetime = Duration(seconds=0.02).to_msg()

    return box_marker
```

</details>

## Header or Class

具体参考 [Here](https://github.com/ros2/rviz/blob/rolling/docs/migration_guide.md)

- rviz -> rviz_common
- ogre_helper -> rviz_rendering 或其子文件夹

```cpp
// >>> ROS2 >>>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <rviz/frame_manager.hpp>
#include <rviz_default_plugins/rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp>

// >>> ROS1 >>>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/frame_manager_iface.hpp>
#include <rviz/default_plugin/point_cloud_transformers.h>
```

- default_plugin -> rviz_default_plugins
- NodeHandle：ROS1 中没有`getRosNodeAbstraction`，需要显式构建句柄

```cpp
// >>> ROS2 >>>
rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

// >>> ROS1 >>>
ros::NodeHandle nh;
```

## Plugins

RViz 的插件类型包括：Display, Panel, Tool, View Controller，开发时具体参考 [Here](https://github.com/ros2/rviz/blob/rolling/docs/plugin_development.md)

其中 Display 一般用于显示（如渲染点云、文本、线条），Panel 一般用于控制，Tool 一般用于交互（键鼠交互）

### Display

<details>
    <summary>:wrench: <b>用例 1：</b>
        编写 Display 类插件（不包含 C++ 源程序）
    </summary>

<details>
    <summary>1）步骤 1：编写 CMakeLists.txt</summary>

<!-- tabs:start -->

#### **ROS1**

```cmake
# 触发 Qt 的 MOC 编译器对 Qt 宏进行处理
set(CMAKE_AUTOMOC ON)

# ROS1
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
find_package(catkin REQUIRED COMPONENTS rviz)
target_link_libraries(... ${QT_LIBRARIES})
```

#### **ROS2**

```cmake
# 触发 Qt 的 MOC 编译器对 Qt 宏进行处理
set(CMAKE_AUTOMOC ON)

# ROS2
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
include_directory(${Qt5Widgets_INCLUDE_DIRS})

# pluginlib_export_plugin_description_file(rviz_common <插件描述文件的位置>)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

register_rviz_ogre_media_exports(DIRECTORIES "media")
```

<!-- tabs:end -->

</details>


<details>
    <summary>2）步骤 2：编写 package.xml</summary>

<!-- tabs:start -->

#### **ROS2**

```xml

<package format="3">

    <name>rviz_plugins</name>
    <version>0.0.1</version>
    <description>rviz plugins</description>
    <maintainer email="you@example.com">Your Name</maintainer>
    <license>TODO</license>

    <!-- Build and export dependencies. -->
    <build_depend>ament_cmake_auto</build_depend>
    <depend>rviz_common</depend>
    <depend>rviz_default_plugins</depend>
    <depend>rviz_rendering</depend>
    <depend>rviz_visual_tools</depend>
    <export>
        <build_type>ament_cmake</build_type>
    </export>

</package>
```

#### **ROS1**

```xml

<package format="2">
    <!-- 若不添加 rviz 依赖，则在 RViz 中无法识别到 plugin -->
    <depend>rviz</depend>
    <!-- 在 ROS2 中这部分功能在 CMakeLists.text 中实现 -->
    <export>
        <rviz plugin="${prefix}/plugins/plugin_description.xml"/>
    </export>
</package>
```

> [!note]
>
> 可通过命令行 `rospack plugins --attrib=plugin rviz` 来判断插件是否导出成功

<!-- tabs:end -->

</details>

<details>
    <summary>3）步骤 3：编写 plugin_description.xml</summary>

<!-- tabs:start -->

#### **ROS2**

相比于 ROS1，ROS2 的动态库路径有所简化，只需要提供动态库名称即可（如不需要前缀 lib/）

```xml

<library path="rviz_plugins">
    <class name="rviz_plugins/LogPanels"
           type="rviz_plugins::LogPanels"
           base_class_type="rviz_common::Panel">
        <description>rviz panel for 3D object detection</description>
    </class>
</library>
```

#### **ROS1**

```xml

<library path="lib/libtier4_perception_rviz_plugin">  <!--动态库的路径（不需要.so 后缀，或要前缀）-->
    <class name="rviz_plugins/PedestrianInitialPoseTool"
           type="rviz_plugins::PedestrianInitialPoseTool"
           base_class_type="rviz::Tool">
    </class>
    <class name="rviz_plugins/CarInitialPoseTool"
           type="rviz_plugins::CarInitialPoseTool"
           base_class_type="rviz::Tool">
    </class>
</library>
```

> [!note]
>
> 当出现"Could not load panel in rviz -- PluginlibFactory: The plugin for class..."时可检查库路径是否正确

<!-- tabs:end -->

</details>

<details>
    <summary>4）步骤 4：在源程序中添加插件宏，将插件（即类）导入到库文件中以被调用</summary>

```cpp
// >>> ROS2 >>>
// 在源程序末尾追加导出插件的宏
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteAllObjectsTool, rviz_common::Tool)

// >>> ROS1 >>>
// 在源程序末尾追加导出插件的宏
#include <pluginlib/class_list_macros.h>
// 插件类，基类（含命令空间）
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteAllObjectsTool, rviz::Tool)
```

</details>

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        编写 Display 类插件（含 C++ 源程序，非虚函数）
    </summary>

<details>
    <summary>1）步骤 1：配置 Display 的属性</summary>

<!-- tabs:start -->

#### **ROS2**

```cpp
// color 属性
rviz_common::properties::ColorProperty color_property_ = new rviz_common::properties::ColorProperty(<属性名>, QColor(204, 51, 204), <属性描述>, this, SLOT(updateColorAndAlpha()));

// alpha 属性
rviz_common::properties::FloatProperty alpha_property_ = new rviz_common::properties::FloatProperty(<属性名>, <默认取值>, <属性描述>, this, SLOT(updateColorAndAlpha()));

// int 属性
rviz_common::properties::IntProperty history_length_property_ = new rviz_common::properties::IntProperty(<属性名>, <默认取值>, <属性描述>, this, SLOT(updateHistoryLength()));
// 设置取值范围
history_length_property_->setMin(1);
history_length_property_->setMax(100000);

// topic 属性
update_topic_property_ = new rviz_common::properties::RosTopicProperty(this, SLOT(updateMapUpdateTopic()));
```

#### **ROS1**

TODO

<!-- tabs:end -->

</details>

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        编写 Display 类插件（含C++源程序，含虚函数）
    </summary>

<details>
    <summary>案例 1：重写 ROSTopicDisplay 的 onInitialize() 函数</summary>

重写 MFDClassDisplay 的 onInitialize() 函数，则需预先调用 MFDClassDisplay::onInitialize()

```cpp
void OverlayMenuDisplay::onInitialize() {
    // 初始化主题信息    
    RTDClass::onInitialize(); // （mandatory）用于初始化 ROS 节点
    using MsgT = am_rviz_plugins_msgs::msg::OverlayMenu;
    QString message_name = QString::fromStdString(rosidl_generator_traits::name<MsgT>());
    std::string topic_name = "/default";
    this->topic_property_->setMessageType(message_name);
    this->topic_property_->setValue(topic_name.c_str());
    this->topic_property_->setDescription("Topic to subscribe to.");
    
    // 等价于：
    // QString topic_name = "/default";
    // QString topic_type = rosidl_generator_traits::data_type<MsgT>();
    // RTDClass::setTopic(topic_name, topic_type);
}
```

</details>

<details>
    <summary>案例 2：重写 ROSTopicDisplay 的 reset()，onEnable()，onDisable() 函数</summary>

reset() 会在 Display 创建时会调用，onEnable() 会在 Display 启用时调用，onDisable() 则会在 Display 关闭时调用

```cpp
// Called to tell the display to clear its state
void OverlayMenuDisplay::reset() {
    RosTopicDisplay::reset();
}

void OverlayMenuDisplay::onEnable() {
    if (overlay_) {
        overlay_->show();
    }
}

void OverlayMenuDisplay::onDisable() {
    if (overlay_) {
        overlay_->hide();
    }
}
```

</details>

</details>

### Panel

TODO

### Tool

TODO

## Others

<details>
    <summary>:wrench: <b>用例 1：</b>
        给插件添加 logo
    </summary>

<!-- tabs:start -->

#### **ROS2**

1）步骤 1：在当前包目录下创建 icon/classes 文件夹，并在 icon/classes 目录下添加`.png`文件（`icon`文件名需同插件名），比如以下的插件名为`Teleop`，则 icon 文件名为 `Teleop.png` 。如果没有 name 属性，则使用类名，即文件名应为 `TeleopPanel`

```xml

<library path="lib/librviz_plugin_tutorials">
    <class name="rviz_plugin_tutorials/Teleop"
           type="rviz_plugin_tutorials::TeleopPanel"
           base_class_type="rviz::Panel">
        <description>
            A panel widget allowing simple diff-drive style robot base control.
        </description>
    </class>
</library>
```

2）步骤 2：修改 CMakeLists.txt，将文件安装到 install/share 目录下

```cmake
# 导出相关的共享库、依赖等信息
ament_auto_package(
  INSTALL_TO_SHARE
  icons
)
```

#### **ROS1**

TODO

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        RViz2 中如何使用 Logger
    </summary>

```cpp
// 不会输出到 /rosout
RVIZ_COMMON_LOG_INFO("Hello, world!");
RVIZ_COMMON_LOG_INFO_STREAM("Hello" << "world!");

// 会发布到 /rosout
// 其中的节点为 rviz 而非 rviz2
RCLCPP_INFO(rclcpp::get_logger("rviz"), "clicked: (%d, %d)", event.x, event.y);
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        将 RViz1 的配置文件转换为 RViz2 的配置文件（版本需大于等于 <a href="https://github.com/ros2/rviz/blob/iron/rviz2/scripts/rviz1_to_rviz2.py">Iron</a>）
    </summary>
</details>

---

# Shortcut

| 快捷键 |        功能        |
|:---:|:----------------:|
|  m  |   Move Camera    |
|  s  |      Select      |
|  g  |   2D Nav Goal    |
|  p  | 2D Pose Estimate |
|  c  |  Publish Point   |


# Plugins

|                                                            插件                                                             |           备注            |
|:-------------------------------------------------------------------------------------------------------------------------:|:-----------------------:|
|                           [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)                           |     目前只提供了 ROS1 版本      |
|                      [vision_msgs_rviz_plugins](https://github.com/NovoG93/vision_msgs_rviz_plugins)                      |     目前只提供了 ROS2 版本      |
|             [pointcloud2_normal_rviz_plugin](https://github.com/UCR-Robotics/pointcloud2_normal_rviz_plugin)              | 目前只提供了 ROS1 版本，用于可视化法向量 |
|                               [miv_rviz_panel](https://github.com/quantumxt/miv_rviz_panel)                               |  用于在一个 Display 中显示多张图片  |
| [plugin_lecture](https://github.com/project-srs/ros_lecture/tree/014c2e409c8eed7a17300cb73407c77379cbfba1/plugin_lecture) |    包含了 overlay 显示等插件    |

# Reference

- [RViz plugin tutorials ](https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/rviz_plugin_tutorials) 