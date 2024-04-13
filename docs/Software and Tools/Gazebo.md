# Gazebo

## Install

<!-- tabs:start -->

#### **[ROS2](https://classic.gazebosim.org/tutorials?tut=ros2_installing)**

```bash
# Ubuntu 22.04（会与Garden冲突）
$ sudo apt install ros-humble-gazebo-ros-pkgs

# Test
# 加载模型
$ gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
# 触发行进
$ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1

# 导入模型~/.gazebo/model
$ wget -c https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip
```

- 对应的版本：

|     ROS version     | Gazebo version |
|:-------------------:|:--------------:|
| Ubuntu18.04 Melodic |   Gazebo 9.x   |
| Ubuntu20.04 Noetic  | Gazebo 11.11.0 |
|        Foxy         |  Gazebo 11.x   |
| Ubuntu22.04 Rolling | Gazebo 11.10.2 |

#### **[Garden](https://gazebosim.org/docs/garden/install_ubuntu)**

```bash
# >>> Install >>>
# 仅适用于Ubuntu20.04 和 Ubuntu22.04
$ sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install gz-garden

# >>> Uninstall >>>
$ sudo apt remove gz-garden && sudo apt autoremove
```

<!-- tabs:end -->

## Environment Variable

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=components">相关的环境变量配置</a>
    </summary>

```bash
# 插件的环境变量
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<插件位置>

# 可作为sdf中<uri>model://<...></uri>的相对路径
$ export GAZEBO_MODEL_PATH=<模型位置>
```

</details>

## File

|                    文件格式                    |        编辑或可视化工具         |                   备注                    |
|:------------------------------------------:|:-----------------------:|:---------------------------------------:|
|                    URDF                    | rviz, vscode, foxyglove |                用于表示单个模型                 |
| [xacro](https://github.com/ros/xacro/wiki) |     rviz, foxyglove     |          用于表示模型，使用时需要转换为URDF格式          |
|                    .stl                    |         Open3D          |             STL 文件（只含单一颜色）              |
|                    .dae                    |         Blender         |           Collada 文件（含纹理，颜色）            |
|               .sdf / .world                |         gazebo          | 用于描述多个模型（插件、传感器文件）+ 表示世界；不适用于 ROS<br /> |

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros">用 Gazebo 打开 .world 文件（即 sdf 格式的文件）</a>（CLI）
    </summary>
SDF（Simulation Description Format），专属于 Gazebo 的格式，相关标签信息可参考 [Here](http://sdformat.org/spec?ver=1.11&elem=link)

```bash
# 方案 1：
(ROS) $ gazebo <.world>
# 方案 2：
(ROS) $ rosrun gazebo_ros gazebo TD3.world

# 方案 3：
(ROS) $ gzserver # 无图形化界面
(ROS) $ gzclient # 追加图形化界面
# -u: 以暂停模式打开

# 方案 4：
(ROS1) $ roslaunch gazebo_ros empty_world.launch
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        验证当前的 URDF 文件是否合法，合法时输出对应的 sdf 文件
    </summary>

方案 1：

```bash
$ gz sdf -p <URDF 文件>
```

[方案 2](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros#Tutorial:UsingaURDFinGazebo)：（纯粹检查 URDF 的合法性）

```bash
# sudo apt install liburdfdom-tools
$ check_urdf calibration.urdf
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a href="将 xacro 文件转换为 urdf 文件](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File">将 Xacro 文件转换为 URDF 文件，含语法检查</a>  
    </summary>

```bash
# for Melodic
$ xacro --inorder model.xacro > model.urdf
# for Noetic+
$ xacro model.xacro > model.urdf
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20230204100549407.png ':size=700 触发宏替换后的 Xacro')

</details>

## Model

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=modifying_world&cat=build_world">修改 world 的物理量：界面背景色和自然光</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a herf="http://classic.gazebosim.org/tutorials?tut=model_editor#Overview">在 Gazebo 中实现模型建模</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a href="https://answers.ros.org/question/29437/possible-to-declare-static-object-in-urdf-file/">设置模块不受重力影响</a>
    </summary>

方案 1：设置所有模块不受影响

```xml

<gazebo>
    <static>true</static>
</gazebo>
```

方案 2：绑定到 world 坐标系

```xml

<link name="world"/>
<joint name="world_joint" type="fixed">
<origin xyz="2 0 1.5" rpy="0 0 0"/>
<parent link="world"/>
<child link="子link"/>
</joint>
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros#%3Ccollision%3Eand%3Cvisual%3Eelements">指定 Gazebo 模型的颜色</a>
    </summary>

```xml

<gazebo reference="link1">
    <material>Gazebo/Orange</material>
</gazebo>
```

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        Gazebo 中的模型偏暗
    </summary>

去除 Shadow 属性

</details>

<details>
    <summary>:wrench: <b>用例 6：</b>
        使用 Gazebo 搭建虚拟场景
    </summary>

Edit | Building Editor（或 CTRL+B）

</details>

<details>
    <summary>:wrench: <b>用例 7：</b>
        <a href="https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
">搭建差速模型插件</a>
    </summary>

TODO

</details>

<details>
    <summary>:wrench: <b>用例 8：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=build_model">构建基础模型</a>
    </summary>

文件的配置具体参考 [Here](https://classic.gazebosim.org/tutorials?tut=model_structure)

</details>

<details>
    <summary>:wrench: <b>用例 9：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=color_model">使用 Orge Material Script 进行贴图</a>
    </summary>
</details>

## Visualization

<details>
    <summary>:wrench: <b>用例 1：</b>
        可视化 Xacro 和 URDF 文件（<a href="http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File">Launch</a>）
    </summary>

[//]: # (@formatter:off)
```xml
<!-- for Xacro-->
<param name="robot_description" command="xacro 'xacro文件名'"/>
<!-- for URDF-->
<param name="robot_description" textfile="urdf文件名"/>
```
[//]: # (@formatter:on)

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        在 WEB 端查看 URDF 文件（<a href="https://mymodelrobot.appspot.com/5629499534213120">Mymodelrobot</a>）
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        使用 <a hef="https://foxglove.dev/docs/studio/panels/3d#add-unified-robot-description-format-urdf">Foxyglove</a> 查看 URDF 文件
    </summary>

https://foxglove.dev/docs/studio/panels/3d#add-unified-robot-description-format-urdf

```bash
# >>> Install >>>
$ wget -c https://github.com/foxglove/studio/releases/download/v1.39.0/foxglove-studio-1.39.0-linux-amd64.deb
$ sudo dpkg -i foxglove-studio-1.39.0-linux-amd64.deb
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20230204023829587.png ':size=700')

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        使用 VSCode 查看 URDF 文件
    </summary>

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20230204023929743.png ':size=700')

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        使用 RViz 可视化 URDF 文件
    </summary>

> [!note]
>
> 若是 XACRO 生成的文件，需要删除顶部的注释，否则会影响发布的主题 /robot_description 具体原因未知

<!-- tabs:start -->

#### **rosrun**

```bash
(ROS1) $ rosrun robot_state_publisher robot_state_publisher <URDF 文件>
(ROS2) $ ros2 run robot_state_publisher robot_state_publisher <URDF 文件名>
```

#### **roslaunch**

对于 ROS1 RVIZ launch（需 TF + Link + 时钟源正确才能显示）

<!-- tabs:end -->

```xml

<launch>

    <param name="/use_sim_time" value="false"/>

    <!-- Load the URDF into the ROS Parameter Server -->
    <param name="robot_description" command="$(find xacro)/xacro 'calibration.urdf'"/>

    <!-- 启动Gazebo -->
    <node name="gazebo" pkg="gazebo_ros" type="gazebo" respawn="false" output="screen"/>

    <!-- 导入相机模型 -->
    <node name="d435i_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -model realsense_d435i -param robot_description"/>

    <!-- 发布TF -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>

</launch>
```

</details>

## Plugins

具体参考 [Here](http://classic.gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)

|      插件       |              备注               |
|:-------------:|:-----------------------------:|
| model plugin  |   需要 robot 根标签和 gazebo 子标签    |
| sensor plugin | 需要提供 robot 标签，link，gazebo 子标签 |

<details>
    <summary>:wrench: <b>用例 1：</b>
         <a href="https://answers.ros.org/question/377196/find-intrisic-camera-matrix-from-gazebo-model/">使用 libgazebo_ros_camera 插件如何获取相机的内参？</a>
    </summary>

```bash
(ROS1) $ rostopic echo camera_info
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        如何检查插件是否成功导入
    </summary>

启动 gazebo 时，追加 --verbose 选项

</details>

## Shortcut

<details>
    <summary>:wrench: <b>用例 1：</b>
        快捷鍵
    </summary>

具体参考 [Here](http://classic.gazebosim.org/hotkeys)

|  快捷键   |       作用       |
|:------:|:--------------:|
| CTRL+T | topic selector |

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        在 Docker 中启动 Gazebo
    </summary>

实测 VNC 下无效，使用 host 的 XServer 则有效

```bash
$ __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gazebo
```

</details>

## Others

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行
    </summary>

```bash
# 追加模型，启动后含格式检测
# 启动含服务的gazebo
(ROS1) $ rosrun gazebo_ros gazebo
(ROS1) $ rosrun gazebo_ros spawn_model -file <文件名> -sdf -model <模型名> -y 1

# 启动含服务的gazebo
(ROS2) $ gazebo --verbose -s libgazebo_ros_factory.so
(ROS2) $ ros2 run gazebo_ros spawn_entity.py -file <文件名> -entity <模型名> -topic /robot_description
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        ROS 主题和服务
    </summary>

| 服务名                   | 服务类型             | 作用                                                         |
| ------------------------ | -------------------- | ------------------------------------------------------------ |
| /gazebo/pause_physics    | std_srvs/Empty       | 暂停物理引擎的更新（物理运动如物体移动）会暂停，仿真时间也会停止。特别适合需要在静止环境中进行观察。 |
| /gazebo/unpause_physics  | std_srvs/Empty       | 恢复物理引擎的更新                                           |
| /gazebo/reset_simulation | std_srvs/Empty       | 重置模型的位置、状态、仿真时间）                             |
| /gazebo/reset_world      | std_srvs/Empty       | 重置模型的位置和状态，但不会重置仿真时间                     |
| /gazebo/set_model_state  | gazebo/SetModelState | [重置模型的位置](https://answers.gazebosim.org//question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/) |

| 主题名                  | 主题类型               | 作用                                                         |
| ----------------------- | ---------------------- | ------------------------------------------------------------ |
| /gazebo/set_model_state | gazebo_msgs/ModelState | [重置模型的位置](http://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros) |

```python
rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
rospy.wait_for_service("/gazebo/unpause_physics")
try:
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
except (rospy.ServiceException) as e:
    print("/gazebo/unpause_physics service call failed")
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        搭建虚拟展厅
    </summary>

![](_asset%2Frobot_sim_demo.gif '虚拟展厅（中科院）')

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        <a href="https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin">world to grid_map</a>
    </summary>

![](_asset%2Fworld_to_grid_map.png ':size=700 world_to_grid_map')

通过基于栅格的判断该区域是否可行，来取代之前的 if else 判断，用于强化学习中判断某个区域是否可行

```bash
# 实测，可直接使用
# 安装相关依赖
$ apt install ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-map-server
$ git clone https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin
$ catkin build
$ rosservice call /gazebo_2Dmap_plugin/generate_map
$ rosrun map_server map_saver -f <map_name> /map:=/map2d
```

相似的还有 Eric_Pxz 提供的插件，具体参考 [Here](https://blog.csdn.net/Eric_Pxz/article/details/125412242)

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        <a href="https://github.com/Adlink-ROS/map2gazebo">grid_map to world</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 6：</b>
        <a href="https://github.com/navigation-gridmap/gazebo_gridmap_plugin">gazebo to height map (for ROS2)</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 7：</b>
        <a href="https://github.com/CyberAgentAILab/pointcloud2gazebo?tab=readme-ov-file">pointcloud to world</a>
    </summary>
</details>

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        <a href="https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros#%3Ccollision%3Eand%3Cvisual%3Eelements">为什么有的物体在 Gazebo 中无法被激光雷达扫描？</a>
    </summary>

该物体没有 collision 标签

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        Gazebo 启动后黑屏，加载时间过长
    </summary>

检查 `.world` 文件是否有不存在的资源。其中，基础模型可以在 GitHub 下载如：

```bash
$ git clone https://github.com/osrf/gazebo_models.git --depth=1 ~/.gazebo/models
```

</details>

<details>
    <summary>:question: <b>问题 3：</b>
        origin 所表示的含义
    </summary>

[joint 的 origin 是 joint 相对于父系的](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)

</details>

## Reference

| 摘要           | ROS2                                                                                  | ROS1                                                                                     |
|--------------|---------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------|
| 官方文档         | https://classic.gazebosim.org/tutorials?tut=ros2_overview                             | https://classic.gazebosim.org/tutorials?tut=ros_overview#Tutorial:ROSintegrationoverview |
| 官方 API       | https://github.com/gazebosim/gazebo-classic/blob/gazebo11/Migration.md                | 无                                                                                        |
| Ignition API | https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/namespaceignition.html | 无                                                                                        |

| 摘要   | Gazebo Sim                 | Gazebo-classic                 |
|------|----------------------------|--------------------------------|
| 官方文档 | https://gazebosim.org/home | https://classic.gazebosim.org/ |

- URDF 官方文档：http://wiki.ros.org/urdf/Tutorials
- [惯性属性的作用](https://www.youtube.com/watch?v=sHzC--X0zQE)
- [用 Blender 构建场景级模型](https://www.bilibili.com/video/BV1rT4y1P7HN/)
- [Gazebo 模型库](https://app.gazebosim.org/dashboard)