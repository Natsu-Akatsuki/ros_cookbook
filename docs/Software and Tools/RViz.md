# RViz

校验等级：:star::star::star::star:

在 ROS1 中不推荐使用 RViz 的 Python 拓展库（ROS2 中暂时还没提供封装），一是文档太少，开发成本大；二是还有一些尚待解决的问题。比如退出 Qt 应用程序后，RViz 节点将成为僵尸节点，不能被`rosnode kill`掉，只能使用`rosnode cleanup`清理，实测在 C++中不存在这个问题，进程可以退出得很干净；另外不能够在 Qt 的`RViz`中添加图像面板，否则会有段错误（暂无解决方案）

## Docker

<details>
    <summary>:wrench: <b>用例 1：</b>
        在 Docker 容器中出现：OgreGLSupport.cpp:57: virtual void Ogre::GLSupport::initialiseExtensions(): Assertion `pcVer && "Problems getting GL version string using glGetString"' failed.
    </summary>

No OpenGL Support for nvidia render，于容器中检查一下命令行 `nvidia-smi` 是否有正常的输出

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://github.com/ros-visualization/rviz/issues/1780">Noetic 容器下渲染 RViz会有很强的撕裂效果</a>
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
def init_marker_array():
    """
    Initialize a new MarkerArray with an empty marker for deleting all markers.

    :return: A MarkerArray object with an empty marker for deleting all markers.
    """
    marker_array = MarkerArray()
    empty_marker = Marker()
    empty_marker.action = Marker.DELETEALL
    marker_array.markers.append(empty_marker)
    return marker_array


marker_array = am_marker.init_marker_array()
# 创建 marker
marker_array.markers.append(marker)
```

</details>

## Header

- `rviz` -> `rviz_common`，`h` -> `hpp`

| ROS1                                                                                                                                                                                                                                                                                                       | ROS2                                                                                                                                                                                                                                                                                                                                                             |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| #include "rviz/properties/float_property.h"<br/>#include "rviz/properties/vector_property.h"<br/>#include "rviz/properties/bool_property.h"<br/>#include "rviz/properties/tf_frame_property.h"<br/>#include "rviz/properties/editable_enum_property.h"<br/>#include "rviz/properties/ros_topic_property.h" | #include "rviz_common/properties/float_property.hpp"<br/>#include "rviz_common/properties/vector_property.hpp"<br/>#include "rviz_common/properties/bool_property.hpp"<br/>#include "rviz_common/properties/tf_frame_property.hpp"<br/>#include "rviz_common/properties/editable_enum_property.hpp"<br/>#include "rviz_common/properties/ros_topic_property.hpp" |

| ROS1                                                                                                              | ROS2                                                                                                                                         |
|-------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| #include "rviz/view_controller.h" <br />#include "rviz/view_manager.h" <br />#include "rviz/render_panel.h"<br /> | #include "rviz_common/view_controller.hpp" <br />#include "rviz_common/view_manager.hpp" <br />#include "rviz_common/render_panel.hpp"<br /> |

- `ogre_helper` -> `rviz_rendering/objects`，`h` -> `hpp`

| ROS1                                                                                                                                                                           | ROS2                                                                                                                                                                                                       |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| #include "rviz/ogre_helpers/shape.h"                                                                                                                                           | #include "rviz_rendering/objects/shape.hpp"<br />#include "rviz_rendering/geometry.hpp"                                                                                                                    |
| #include <rviz/ogre_helpers/billboard_line.h><br/>#include <rviz/ogre_helpers/shape.h><br/>#include <rviz/ogre_helpers/arrow.h><br/>#include <rviz/ogre_helpers/point_cloud.h> | #include <rviz_rendering/objects/billboard_line.hpp><br/>#include <rviz_rendering/objects/shape.hpp><br/>#include <rviz_rendering/objects/arrow.hpp><br/>#include <rviz_rendering/objects/point_cloud.hpp> |

- `default_plugin` -> `rviz_default_plugins`

| ROS1                                                      | ROS2                                                                                                 |
|-----------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| #include <rviz/frame_manager.hpp>                         | #include <rviz_common/frame_manager_iface.hpp>                                                       |
| #include <rviz/default_plugin/point_cloud_transformers.h> | #include <rviz_default_plugins/rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp> |

## CMake

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://cmake.org/cmake/help/latest/prop_tgt/AUTOMOC.html">触发 Qt 的 MOC 预处理器对 Qt 宏进行处理</a>
    </summary>

<!-- tabs:start -->

#### **方案 1**

```cmake
// 默认处理目标文件
// 如果目标文件只有源文件，一种方法是将头文件放在目标文件同目录（且文件名需相同，e.g. a.cpp 和 a.hpp）
set(CMAKE_AUTOMOC ON)
add_library(... a.cpp)

// 另一种方法是，将头文件也作为目标文件
set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} a.cpp a.hpp)
```

#### **方案 2**

```cmake
// 显式调用
qt5_wrap_cpp(MOC_FILES
  src/a.cpp include/a.hpp
)

add_library(${PROJECT_NAME} ${MOC_FILES})
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        基础模版
    </summary>

<!-- tabs:start -->

#### **ROS2**

```cmake
# 触发 Qt 的 MOC 编译器对 Qt 宏进行处理
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
include_directory(${Qt5Widgets_INCLUDE_DIRS})

ament_auto_add_library(库名 依赖)
pluginlib_export_plugin_description_file(rviz_common <插件描述文件的位置>)

register_rviz_ogre_media_exports(DIRECTORIES "media")

ament_auto_package(
  INSTALL_TO_SHARE
)
```

#### **ROS1**

```cmake
# 触发 Qt 的 MOC 编译器对 Qt 宏进行处理
set(CMAKE_AUTOMOC ON)

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

<!-- tabs:end -->

</details>

## Package

<details>
    <summary>:wrench: <b>用例 1：</b>
        基础模板
    </summary>

<!-- tabs:start -->

#### **ROS2**

```xml

<package format="3">

    <name>rviz_plugins</name>
    <version>0.0.1</version>
    <description>rviz plugins</description>
    <maintainer email="you@example.com">Your Name</maintainer>
    <license>TODO</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>

    <!-- Build and export dependencies. -->
    <build_depend>ament_cmake_auto</build_depend>

    <depend>rviz_common</depend>
    <depend>rviz_default_plugins</depend>
    <depend>rviz_rendering</depend>
    <depend>rviz_visual_tools</depend>
    <depend>plugin_lib</depend>


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

</details>

<!-- tabs:end -->

## Plugins

RViz 的插件类型包括：Display, Panel, Tool, View Controller，其中 Display 一般用于显示（如渲染点云、文本、线条），Panel 一般用于控制，Tool 一般用于交互（键鼠交互）

<details>
    <summary>:wrench: <b>用例 1：</b>
        获取 RViz 主窗口的大小
    </summary>

<!-- tabs:start -->

#### **ROS2**

```cpp
// 获取主窗口
#include "rviz_common/window_manager_interface.hpp"
// QWidget* parent = getWindowManager()->getParentWindow();
context_->getWindowManager()->getParentWindow()->width();
context_->getWindowManager()->getParentWindow()->height();
```

#### **ROS1**

```cpp
context_->getViewManager()->getRenderPanel()->getRenderWindow()->getWidth();
context_->getViewManager()->getRenderPanel()->getRenderWindow()->getHeight();
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        设置光标 logo
    </summary>

<!-- tabs:start -->

#### **ROS2**

```cpp
makeIconCursor("package://rviz_common/icons/forbidden.svg");
```

#### **ROS1**

```cpp
makeIconCursor("package://rviz/icons/forbidden.svg");
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        编写 plugin_description.xml
    </summary>
<!-- tabs:start -->

#### **ROS2**

```xml
<!-- 相比于 ROS1，ROS2 的动态库路径有所简化（如不需要路径前缀 lib/）-->
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

<library path="lib/libtier4_perception_rviz_plugin">  <!-- 动态库的路径（不需要.so 后缀）-->
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
    <summary>:wrench: <b>用例 4：</b>
        在源程序末尾追加导出插件的宏，将插件（即类）导入到库文件中以被调用
    </summary>

| ROS2(C++)                                                                                                                                                                                                                                                               | ROS1(C++)                                                                                                                                                                                                                                               |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| // 插件类，基类（含命令空间）<br />`#include <pluginlib/class_list_macros.hpp>`<br/>`PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteAllObjectsTool, rviz_common::Tool)`<br/>`PLUGINLIB_EXPORT_CLASS(rviz_animated_view_controller::AnimatedViewController, rviz_common::ViewController)` | // 插件类，基类（含命令空间）<br />`#include <pluginlib/class_list_macros.h>`<br/>`PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteAllObjectsTool, rviz::Tool)`<br/>`PLUGINLIB_EXPORT_CLASS(rviz_animated_view_controller::AnimatedViewController, rviz::ViewController)` |

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        给插件添加 logo
    </summary>

<!-- tabs:start -->

#### **ROS2**

步骤 1：在当前包目录下创建 icon/classes 文件夹，并在 icon/classes 目录下添加`.png`文件（`icon`文件名需同插件名），比如以下的插件名为`Teleop`，则 icon 文件名为 `Teleop.png` 。如果没有 name 属性，则使用类名，即文件名应为 `TeleopPanel`

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

步骤 2：修改 CMakeLists.txt，将文件安装到 install/share 目录下

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
    <summary>:wrench: <b>用例 6：</b>
        判断插件是否顺利导出
    </summary>

<!-- tabs:start -->

> [!note]
>
> 即便导出了，但可能因为写错 plugin_description.xml 而导致 RViz 无法找到相应的插件

#### **ROS2**

[ROS2 中暂时没有显示插件的工具，该 feature 还没 merge 进主分支中](https://github.com/ros2/ros2cli/pull/340)，需额外下载

```bash
$ git clone https://github.com/artivis/pluginlib/tree/feature/ros2plugin -b feature/ros2plugin
$ cd pluginlib/ros2plugin/
$ python3 develop setup.py

$ ros2 plugin list
$ ros2 plugin list --packages
```

#### **ROS1**

```bash
$ rospack plugins --attrib=plugin rviz
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 7：</b>
        将 RViz1 的配置文件转换为 RViz2 的配置文件（版本需大于等于 <a href="https://github.com/ros2/rviz/blob/iron/rviz2/scripts/rviz1_to_rviz2.py">Iron</a>）
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 8：</b>
        重载 onInitialize 的注意事项
    </summary>

```cpp
rclcpp::Node::SharedPtr rviz_node_;
rviz_common::properties::RosTopicProperty *ros_topic_property_;

onInitialize() {  
  // context_ 的调用不能放在构造函数，此时的构造函数 context_ 为 nullptr
  rviz_node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  // ROS topic 的相关属性在此处进行初始化（不初始化，会有段错误）
  ros_topic_property_->initialize(context_->getRosNodeAbstraction());
}
```

</details>

<details>
    <summary>:wrench: <b>用例 9：</b>
        虚函数
    </summary>

| 类              | ROS2                                                                                 | ROS1                      |
|----------------|--------------------------------------------------------------------------------------|---------------------------|
| ViewController | virtual void lookAt(const Ogre::Vector3 & point) = 0;<br />virtual void reset() = 0; | virtual void reset() = 0; |

</details>

<details>
    <summary>:wrench: <b>用例 10：</b>
        示例代码
    </summary>

| 需求                     | ROS2                                                                                           | ROS1 |
|------------------------|------------------------------------------------------------------------------------------------|------|
| Control Viewer 追加快捷键设置 | https://github.com/ros2/rviz/blob/rolling/rviz_common/src/rviz_common/view_controller.cpp#L228 | TODO |

TODO

</details>

<details>
    <summary>:wrench: <b>用例 1：</b>
        创建句柄
    </summary>

ROS1 中没有`getRosNodeAbstraction`，需要显式构建句柄

<!-- tabs:start -->

#### **ROS2**

```cpp
rclcpp::Node::SharedPtr node_;

void 类名::onInitialize()
{
  MFDClass::onInitialize();  
      
  // context_ 等价于 this->getDisplayContext()
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  // 配置订阅器和发布器
  sub_ = node_->create_subscription(...)
}
```

#### **ROS1**

```cpp
ros::NodeHandle nh;
```

<!-- tabs:end -->


</details>

<details>
    <summary>:question: <b>问题 1：</b>
        RViz2 中 /usr/include/OGRE/OgreGpuProgramParams.h:1251:11: error: ‘HashMap’ does not name a type        
    </summary>

库冲突，需使用 RViz 中的 OGRE 而非系统的 OGRE 文件

```cpp
// 使用 .../opt/rviz_orge_vendor/include/OGRE/ 下的文件
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

// 使用 /usr/include/OGRE/ 下的文件
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
```

</details>

<details>
    <summary>:question: <b>问题 2：</b> 
        运行期出现：undefined symbol: _ZTVN29rviz_animated_view_controller22AnimatedViewControllerE, at ./src/shared_library.c:99
    </summary>

CMakeLists.txt 中没开 automoc 处理 Qt 宏，导致当前类不完整（incomplete）

</details>

<details>
    <summary>:question: <b>问题 3：</b>
        从代码层面考虑 View controller 的视角更新是怎么触发的
    </summary>

```cpp
/// Called at 30Hz by ViewManager::update() while this view is active.
/**
 * Override with code that needs to run repeatedly.
 */
virtual void update(float dt, float ros_dt);
```

</details>

## Shortcut

| 快捷键 |        功能        |
|:---:|:----------------:|
|  m  |   Move Camera    |
|  s  |      Select      |
|  g  |   2D Nav Goal    |
|  p  | 2D Pose Estimate |
|  c  |  Publish Point   |

## Reference

| 概要                      | ROS2                                                                                                | ROS1                                                                                                        |
|-------------------------|-----------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------|
| API 迁移                  | https://github.com/ros2/rviz/blob/rolling/docs/migration_guide.md                                   | -                                                                                                           |
| 官方 GitHub               | https://github.com/ros2/rviz                                                                        | https://github.com/ros-visualization/rviz                                                                   |
| 插件教程                    | https://github.com/ros-visualization/visualization_tutorials/tree/ros2/rviz_plugin_tutorials/src)   | https://docs.ros.org/en/kinetic/api/rviz_plugin_tutorials/html/                                             |
| interactive marker 代码示例 | https://github.com/ros-visualization/visualization_tutorials/tree/ros2/interactive_marker_tutorials | https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/interactive_marker_tutorials |

## Repository

|                                                            插件                                                             |                 备注                  |
|:-------------------------------------------------------------------------------------------------------------------------:|:-----------------------------------:|
|                           [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)                           |           目前只提供了 ROS1 版本            |
|                      [vision_msgs_rviz_plugins](https://github.com/NovoG93/vision_msgs_rviz_plugins)                      |           目前只提供了 ROS2 版本            |
|             [pointcloud2_normal_rviz_plugin](https://github.com/UCR-Robotics/pointcloud2_normal_rviz_plugin)              |       目前只提供了 ROS1 版本，用于可视化法向量       |
|                               [miv_rviz_panel](https://github.com/quantumxt/miv_rviz_panel)                               |        用于在一个 Display 中显示多张图片        |
| [plugin_lecture](https://github.com/project-srs/ros_lecture/tree/014c2e409c8eed7a17300cb73407c77379cbfba1/plugin_lecture) |          包含了 overlay 显示等插件          |
|                  [rviz2_camera_ray_tool](https://github.com/schornakj/rviz2_camera_ray_tool/tree/master)                  | 选点 + 可视化射线（提供了使用 rclcpp::Node 的新思路） |
|            [rviz_animated_view_controller](https://github.com/ros-visualization/rviz_animated_view_controller)            |               视图控制插件                |
|                         [rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer)                          |                TODO                 |
