# Logging

校验等级：:star::star::star::star:

## API

|                                                                  ROS2                                                                   |                                                                          ROS1                                                                           |
|:---------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------:|
|                        // 普通输出<br />`RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());`                        |                                           // 普通输出<br />`ROS_INFO("Publishing: '%s'", message.data.c_str());`                                            |
| // 流输出<br />`RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension.visualization"), __FUNCTION__ << ": marker is null pointer!");` |                                      // 流输出<br />`ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");`                                       |
|   // 定时输出<br/>`RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000/*unit: ms*/, "behavior path output is empty! Stop publish."))`    |                             // 定时输出<br />`ROS_DEBUG_THROTTLE(60/*unit: sec*/, "This message will print every 60 seconds");`                             |
|                                                 // 条件输出<br />`RCLCPP_DEBUG_EXPRESSION`                                                  | // 条件输出<br />`ROS_DEBUG_COND(x < 0, "Uh oh, x = %d, this is bad", x);`<br/><br />`ROS_DEBUG_STREAM_COND(x < 0, "Uh oh, x = " << x << ", this is bad");` |
|               // 跳过第一次的输出<br/>`RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);`               |                                                                          TODO                                                                           |
|          // 全局日志：<br/>`RCLCPP_ERROR(rclcpp::get_logger("data_process"), "clear lidar buffer, only happen at the beginning");`           |                                                                          TODO                                                                           |

## Usage

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

方案 1：ros2 run

```bash
# 指定某个节点的输出等级（DEBUG, INFO, WARN, ERROR or FATAL）
(ROS2) $ ros2 run <pkg> <executable> --ros-args --log-level <log-level>
```

#### **ROS1**

方案 1：[修改程序](http://wiki.ros.org/roscpp/Overview/Logging)

```cpp
// 程序上的修改
#include <ros/console.h>

if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}
```

方案 2：`rqt_console` \
方案 3：[配置文档](https://wiki.nps.edu/display/MRC/Setting+Logging+Level)

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a href="https://github.com/ros2/rviz/blob/rolling/docs/migration_guide.md">RViz2 中如何使用日志功能</a>
    </summary>

```cpp
#include "rviz_common/logging.hpp"

// 不会输出到 /rosout
RVIZ_COMMON_LOG_INFO("Hello, world!");
RVIZ_COMMON_LOG_INFO_STREAM("Hello" << "world!");

// 会发布到 /rosout
// 其中的节点为 rviz 而非 rviz2
RCLCPP_INFO(rclcpp::get_logger("rviz"), "clicked: (%d, %d)", event.x, event.y);
```

</details>

## Tools

### rqt_console

- 显示 ROS 的日志信息

```bash
$ rosrun rqt_console rqt_console
```

## Reference

| 概要   | ROS2                                                       | ROS1                                        |
|------|------------------------------------------------------------|---------------------------------------------|
| 官方文档 | https://docs.ros.org/en/humble/Concepts/About-Logging.html | http://wiki.ros.org/roscpp/Overview/Logging |
| API  | https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html  | http://wiki.ros.org/rosconsole              |