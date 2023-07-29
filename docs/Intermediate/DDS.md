# DDS

# Install

```bash
$ sudo apt install ros-${ROS_DISTRO}-rmw-fastrtps-cpp
$ sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
$ sudo apt install ros-${ROS_DISTRO}-rmw-connextdds
$ sudo apt install ros-${ROS_DISTRO}-rmw-gurumdds-cpp
```

# Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        查看当前系统使用的 RMW 实现
    </summary>

```bash
$ ros2 doctor --report | grep middleware
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        使用指定的 RMW
    </summary>

```bash
# rmw_connextdds
$ RWM=rmw_fastrtps_cpp
$ export RMW_IMPLEMENTATION=${RWM}
$ RMW_IMPLEMENTATION={RWM} ros2 run demo_nodes_cpp talker
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        进行 RMW 的相关配置
    </summary>

```bash
# 禁止多播（multi-broadcast）只允许本机通信（localhost-only）
$ export ROS_LOCALHOST_ONLY=1
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        <a href="https://github.com/ros2/rosbag2/issues/1155">[RTPS_MSG_IN Error] (ID:...) Problem reserving CacheChange in reader</a>
    </summary>

目前仅知道该 WARNING 提示与 FastDDS 有关

</details>

# FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        ROS2 下现已支持的 DDS
    </summary>

|                          Product name                           |           License           | RMW implementation   |                                         Status                                          |
|:---------------------------------------------------------------:|:---------------------------:|----------------------|:---------------------------------------------------------------------------------------:|
|    [eProsima Fast DDS](https://github.com/ros2/rmw_fastrtps)    |          Apache 2           | `rmw_fastrtps_cpp`   |                Full support. Default RMW. Packaged with binary releases.                |
|  [Eclipse Cyclone DDS](https://github.com/ros2/rmw_cyclonedds)  | Eclipse Public License v2.0 | `rmw_cyclonedds_cpp` |                      Full support. Packaged with binary releases.                       |
|    [RTI Connext DDS](https://github.com/ros2/rmw_connextdds)    |    commercial, research     | `rmw_connextdds`     |      Full support. Support included in binaries, but Connext installed separately.      |
| [GurumNetworks Gurum DDS](https://github.com/ros2/rmw_gurumdds) |         commercial          | `rmw_gurumdds_cpp`   | Community support. Support included in binaries, but Gurum**DDS** installed separately. |

</details>







