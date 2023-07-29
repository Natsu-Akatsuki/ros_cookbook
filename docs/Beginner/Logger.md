# Logging

# Usage

## Client Library

|       ROS1       |          ROS2           |
|:----------------:|:-----------------------:|
|     ROS_INFO     |       RCLCPP_INFO       |
| ROS_ERROR_STREAM |   RCLCPP_ERROR_STREAM   |
|  ROS_DEBUG_COND  | RCLCPP_DEBUG_EXPRESSION |

- ROS1

```cpp

// >>> ROS1 >>>
// 普通输出
ROS_INFO("Publishing: '%s'", message.data.c_str());
// 流输出
ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
// 定时输出
ROS_DEBUG_THROTTLE(60/*unit: sec*/, "This message will print every 60 seconds");
// 条件输出
ROS_DEBUG_COND(x < 0, "Uh oh, x = %d, this is bad", x);
ROS_DEBUG_STREAM_COND(x < 0, "Uh oh, x = " << x << ", this is bad");

// >>> ROS2 >>>
// 普通输出
RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
// 流输出
RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension.visualization"), __FUNCTION__ << ": marker is null pointer!");
// 定时输出
RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000/*unit: ms*/, "behavior path output is empty! Stop publish."))
// 跳过第一次的输出
RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);
// 条件输出
RCLCPP_DEBUG_EXPRESSION
// 全局日志：
RCLCPP_ERROR(rclcpp::get_logger("data_process"), "clear lidar buffer, only happen at the beginning");
```

## Configuration

<details>
    <summary>:wrench: <b>用例 1：</b>
        ROS2 中使能含颜色的日志输出
    </summary>

```bash
$ export RCUTILS_COLORIZED_OUTPUT=1
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        指定某个节点的日志输出等级
    </summary>

<!-- tabs:start -->

#### **ROS2**

1）方案 1：ros2 run

```bash
# 指定某个节点的输出等级（DEBUG, INFO, WARN, ERROR or FATAL）
(ROS2) $ ros2 run <pkg> <executable> --ros-args --log-level <log-level>
```

#### **ROS1**

1）方案 1：[修改程序](http://wiki.ros.org/roscpp/Overview/Logging)

```cpp
// 程序上的修改
#include <ros/console.h>

if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
```

2）方案 2：`rqt_console` \
3）方案 3：[配置文档](https://wiki.nps.edu/display/MRC/Setting+Logging+Level)

<!-- tabs:end -->

</details>

# Reference

- Logger official tutorial for [ROS1](http://wiki.ros.org/roscpp/Overview/Logging) and [ROS2](https://docs.ros.org/en/humble/Concepts/About-Logging.html)
- Logger API for [ROS2](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html)