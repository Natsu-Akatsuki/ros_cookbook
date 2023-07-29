# Launch

> [!note]
>
>`.xml`文件在 ROS1 下无法被`roslaunch`识别而补全

## Usage

### attribute

<details>
    <summary>:wrench: <b>用例 1：</b>
        namespace
    </summary>

[//]: # (@formatter:off)
```xml

<!-- ROS1 -->
<group ns="map">
    <include file="$(find map_loader)/launch/lanelet2_map_loader.launch">
        <arg name="file_name" default="$(arg lanelet2_map_path)"/>
    </include>
</group>

<!-- ROS2 -->
<group>
    <push-ros-namespace namespace="map"/>
    <include file="$(find-pkg-share map_loader)/launch/lanelet2_map_loader.launch.xml">
        <arg name="lanelet2_map_path" value="$(var lanelet2_map_path)"/>
    </include>
</group>
```
[//]: # (@formatter:on)

</details>

### substitution

<details>
    <summary>:wrench: <b>用例 1：</b>
        substitution
    </summary>

|         ROS1         |    ROS2    |                 备注                 |
|:--------------------:|:----------:|:----------------------------------:|
|      $(arg 变量名)      | $(var 变量名) |                 ——                 |
| $(find-pkg-share 包名) | $(find 包名) | substitutions（ROS2 的需要安装到 install） |

</details>

### tag

具体参考 [ROS1](http://wiki.ros.org/roslaunch/XML#Tag_Reference)

<details>
    <summary>:wrench: <b>用例 1：</b>
        arg 标签
    </summary>

|                 ROS1                 |    ROS2     |
|:------------------------------------:|:-----------:|
|                 type                 |    exec     |
|                  ns                  |  namespace  |
|                 doc                  | description |
| machine, respawn_delay, clear_params |      —      |

在 ROS2 中对某个变量进行赋值，将使用 let 标签

```xml

<arg name="foo" value="foo"/>   <!-- ROS1 -->
<let name="foo" value="foo"/>   <!--ROS2-->
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html#access-to-math-module-when-evaluating-python-expressions">log 标签</a>
    </summary>

只适用于 ROS2，暂并不支持`substitution` 语法，

```xml

<log message="日志信息"/>
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a>group 标签</a>
    </summary>

1）<a href="https://github.com/ros2/launch/blob/master/launch/launch/actions/group_action.py">scope</a>：相关作用类似于作用域，表示其中的配置参数在 group 外是否还能被使用，默认值为 true，表示该变量的作用域只局限于 group 内部

```xml

<group scoped="false">
    <!-- TODO -->
</group>
```

2）[条件判断](http://wiki.ros.org/roslaunch/XML#if_and_unless_attributes)

```xml

<arg name="mode" default="..."/>

<group if="$(eval mode=='...')">TODO</group>
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        rosparam 和 param
    </summary>

|                            ROS1                             |                                                 ROS2                                                 | 备注 |
|:-----------------------------------------------------------:|:----------------------------------------------------------------------------------------------------:|:--:|
| [rosparam tag](https://wiki.ros.org/roslaunch/XML/rosparam) | [param tag](https://docs.ros.org/en/humble/How-To-Guides/Launch-files-migration-guide.html#rosparam) | —— |

[//]: # (@formatter:off)
```xml
<!-- ROS1 -->
<node pkg="my_package" exec="my_executable" name="my_node" ns="/an_absolute_ns">
    <rosparam command="load" file="/path/to/file"/>
</node>

<!-- ROS2 -->
<node pkg="my_package" exec="my_executable" name="my_node" ns="/an_absoulute_ns">
    <param from="/path/to/file"/>
</node>
```
[//]: # (@formatter:on)

> [!note]
>
> ROS2 没有全局参数的概念，param 标签只能嵌套放在 node 等标签中，同时该标签也没有 type 等属性

</details>

### Python

<details>
    <summary>:wrench: <b>用例 1：</b>
        启动一个节点
    </summary>

```python
import launch
import launch_ros.actions
from launch import LaunchDescription


# 方案一：
def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareArgument(name='log_level', default_value='info'),
        launch_ros.actions.Node(
            package='包名',
            executable='可执行文件名',
            name='节点名',
            remappings=[
                ("input/initialpose", "/initialpose3d"),
                ("input/ackermann_control_command", "/control/command/control_cmd")
            ],
            parameters=[{
                参数名: 参数值
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')], output="screen"
        )
    ])


# 方案二：
def generate_launch_description():
    ld = LaunchDescription()

    node = Node(package="包名",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"])

    ld.add_action(node)

    return ld
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html">基于命令设置 param 参数</a>
    </summary>

```python
set_prarm = ExecuteProcess(
    cmd=[['ros2 param set ', 节点名, '参数名 参数值']], shell=True)
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        导入参数文件
    </summary>

```python
import yaml

behavior_velocity_planner_param_path = os.path.join(
    # .perform(context) 相当于$(find <包名>)
    LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
    "scenario_planning",
    "lane_driving",
    "behavior_planning",
    "behavior_velocity_planner",
    "behavior_velocity_planner.param.yaml",
)

with open(behavior_velocity_planner_param_path, "r") as f:
    behavior_velocity_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

# 导入参数：
parameters = [
    behavior_velocity_planner_param,
    ...]
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        设置定时器对象      
    </summary>

```python
from launch.actions import ExecuteProcess, TimerAction

TimerAction(
    period=2.0,
    actions=[ExecuteProcess(...)],
)
```

</details>

## Glossary

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20210807095158418.png ':size=800')

> [!note]
>
> `roslaunch` 在启动节点前，会提前解析 `substitution args` 和导入参数 `param` 到参数服务器

## Tools

### Plugins

- VSCode 插件：ROS snippets（代码块）
- VSCode 插件：Xml Formatter（格式化 XML 文件）

### Scripts

- [ROS2 Launch File Migrator](https://github.com/aws-robotics/ros2-launch-file-migrator)：将 ROS1 的 launch.xml 文件转换为 ROS2 的 launch.py 文件

## Reference

- XML Format for [ROS2](https://design.ros2.org/articles/roslaunch_xml.html)
- [Migration tutorials](https://docs.ros.org/en/humble/How-To-Guides/Launch-files-migration-guide.html)
