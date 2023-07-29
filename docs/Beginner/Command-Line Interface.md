# Command-Line Interface

## Usage

### Daemon

只有 ROS2 才有 daemon 的概念

<details>
    <summary>:wrench: <b>用例 1：</b>
        重启 ROS2
    </summary>

```bash
$ ros2 daemon stop
$ ros2 daemon start 
```

</details>

### Diagnostics

<details>
    <summary>:wrench: <b>用例 1：</b>
        查看哪些主题没有订阅成功        
    </summary>

<!-- tabs:start -->

#### **ROS2**

```bash
$ ros2 doctor
```

#### **ROS1**

```bash
$ roswtf
```

<!-- tabs:end -->

</details>

### Node

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行
    </summary>

<!-- tabs:start -->

#### **ROS2**

```bash
# >>> 查看节点的具体信息（如包含某些订阅器，发布器）>>>
$ ros2 node info <节点名>

# >>> 查看已启动的节点 >>>
$ ros2 node list

# >>> 启动节点 >>>
# 对于 Python 文件，需要添加可执行权限
$ ros2 run <pkg_name> <node_name>
```

#### **ROS1**

```bash
# >>> 查看节点的具体信息（如包含某些订阅器，发布器）>>>
$ rosnode info <节点名>

# >>> 查看已启动的节点 >>>
$ rosnode list

# >>> 启动节点 >>>
# 对于 Python 文件，需要添加可执行权限
$ rosrun <pkg_name> <node_name>
```

<!-- tabs:end -->

</details>

### Package

```bash
# 跳转到某个包的源文件路径
(ROS1) $ roscd <pkg_name>
# ros2没有ros2cd的相关实现，需自行构建
(ROS2) $ ros2cd <pkg_name>

# 返回某个包的绝对路径
(ROS1) $ rospack find <pkg>
```

```bash
# custom bash function
# you could append to ~/.bashrc
ros2cd() {
    pkg_name=$1
    pkg_path=$(colcon list --paths-only --packages-select ${pkg_name})
    cd ${pkg_path}
}
```

- [ROS1 client library](http://docs.ros.org/en/independent/api/rospkg/html/python_api.html)

```cpp
# rospack find <pkg> (C++)
#include <ros/package.h>
std::string path = ros::package::getPath("package_name");

# rospack find <pkg> (Python)
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('package_name')
```

## Reference

- [ROS2 CLI Cheatsheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)
- [ROS2 doctor](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
- [ROS1 client library](http://docs.ros.org/en/hydro/api/rosnode/html)

