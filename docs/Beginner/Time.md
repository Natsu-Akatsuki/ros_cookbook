# Time

> [!attention]
>
> 应留意因类型不同而导致的精度丢失

## Clock

如下代码块只适用于

| 功能    | 代码块（ROS2）                                                                                                                                      |
|-------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 设置时钟源 | `rclcpp::Clock ros_clock{RCL_ROS_TIME};`<br/>`rclcpp::Clock system_clock{RCL_SYSTEM_TIME};`<br/>`rclcpp::Clock steady_clock{RCL_STEADY_TIME};` |

## Time

### Second to Time

ROS 2 没有将双精度浮点型（以秒为单位）的时间戳转换为 Time 对象的接口，可以先将其转换为 int64（以 ns 为单位）的时间戳，但该接口会丢失精度。

| ROS2(C++)                                                                   | ROS(C++)                                                                    |
|:----------------------------------------------------------------------------|:----------------------------------------------------------------------------|
| // 近似替代<br/>`rclcpp::Time t(static_cast<int64_t>(seconds * 1e9));`          | // 秒（double）<br />`ros::Time(double t)`<br/>`ros::Time().fromSec(double t)` |
| // 秒（int32）+纳秒（unint32）<br/>`Time(int32_t seconds, uint32_t nanoseconds)`   | // 秒（unint32）+纳秒（unint32）<br />`ros::Time(uint32_t _sec, uint32_t _nsec)`   |
| // 纳秒（int64）<br/>`rclcpp::Time t(int64_t nanoseconds=0);`                   | // 纳秒（uint64）<br/>`ros::Time().fromNSec(uint64_t t)`                        |
| // 自建消息类型<br/>`rclcpp::Time(const builtin_interfaces::msg::Time &time_msg)` | 无                                                                           |

| ROS2(Python) | ROS(Python)                                                |
|:-------------|:-----------------------------------------------------------|
| TODO         | // 秒（整型） + 纳秒（整型）<br />`rospy.Time(secs=0, nsecs=1000000)` |

### Time to Second

| ROS2(C++)                                                                         | ROS(C++)                                                                                                                                            |
|:----------------------------------------------------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------------------|
| // 秒（double）<br />`double timestamp = rclcpp::Time(msg->header.stamp).seconds();` | // 秒（double）<br />`double timestamp = msg->header.stamp.toSec(); `<br />`double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9`; |
| // 纳秒（uint64）<br />`uint64_t rclcpp::Time(msg->header.stamp).nanoseconds();`      | // 纳秒（uint64）<br />`uint64_t timestamp = msg->header.stamp.toNSec();`                                                                               |

| ROS2(Python) | ROS(Python)                                                                             |
|:-------------|:----------------------------------------------------------------------------------------|
| TODO         | // 秒（浮点数）<br />`timestamp = (msg.header.stamp).secs + (msg.header.stamp).nsecs * 1e-9;` |

### Now

| ROS2(C++)                                                                                                      | ROS(C++)                          |
|:---------------------------------------------------------------------------------------------------------------|:----------------------------------|
| `rclcpp::Time t = now();`<br/>`rclcpp::Time t = this->now();`<br/>`rclcpp::Time t = this->get_clock()->now();` | `ros::Time t = ros::Time::now();` |
| // RViz2<br />`context_->getClock()->now();`                                                                   | `ros::Time::now();`               |

| ROS2(Python)                                                                                                                                    | ROS(Python)                                       |
|:------------------------------------------------------------------------------------------------------------------------------------------------|:--------------------------------------------------|
| # 需要 Node 对象<br/>`self.get_clock().now();`<br/><br/># 不需要 Node 对象<br/>`from rclpy.clock import Clock`<br/>`time_stamp = Clock().now().to_msg()` | // 仅支持整型<br />`rospy.Time(secs=0, nsecs=1000000)` |

### TF2

获取最新的 TF，具体参考 [Here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html)

| ROS2(C++)                                                                        | ROS(C++)       |
|:---------------------------------------------------------------------------------|:---------------|
| `tf2::TimePointZero`<br />                                                       | `ros::Time(0)` |
| `rclcpp::Time(0, 0, this->get_clock()->get_clock_type())`<br />`rclcpp::Time(0)` | `ros::Time(0)` |

## Duration

### Second to Duration

| ROS2(C++)                                                                                             | ROS(C++)                                                                      |
|:------------------------------------------------------------------------------------------------------|:------------------------------------------------------------------------------|
| // 秒（int32_t） + 纳秒（uint32_t）<br />`rclcpp::Duration t = rclcpp::Duration(0, 0);`<br />                | // 秒（int32_t） + 纳秒（int32_t）<br />Duration (int32_t _sec, int32_t _nsec);<br/> |
| // 秒（double）版本需大于 foxy<br />`rclcpp::Duration d = rclcpp::Duration::from_seconds(0.5);`               | // 秒（double）<br />`ros::Duration t = ros::Duration(0.5);`                     |
| // 内置消息类型<br />Duration (const builtin_interfaces::msg::Duration &duration_msg)                       | TODO                                                                          |
| Duration (rcl_duration_value_t nanoseconds)<br/>Duration (std::chrono::nanoseconds nanoseconds)<br /> | TODO                                                                          |

| ROS2(Python)                                                                        | ROS(Python)                       |
|:------------------------------------------------------------------------------------|:----------------------------------|
| // 秒<br />`from rclpy.duration import Duration`<br/>`Duration(seconds=0).to_msg()`; | // 秒<br />`rospy.Duration(0.02);` |

### Duration to Second

| ROS2(C++)                                         | ROS(C++) |
|:--------------------------------------------------|:---------|
| `double seconds = d.seconds();`                   | TODO     |
| `rcl_duration_value_t seconds = d.nanoseconds();` | TODO     |

| ROS2(Python) | ROS(Python) |
|:-------------|:------------|
| TODO         | TODO        |

### Sleep

| ROS2(C++)                                                                                                            | ROS(C++)                                |
|:---------------------------------------------------------------------------------------------------------------------|:----------------------------------------|
| `#include <chrono>` <br />`std::this_thread::sleep_for(pause_aniation_duration_.to_chrono<std::chrono::seconds>());` | // 秒<br />`ros::Duration(0.5).sleep();` |

| ROS2(Python) | ROS(Python)                  |
|:-------------|:-----------------------------|
| 使用标准库        | // 秒<br />`rospy.sleep(0.5)` |

### Other

| ROS2(C++) | ROS(C++)   |
|:----------|:-----------|
| 无         | `isZero()` |

## Rate and Timer

推荐使用 [Timer](http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers) 来实现特定频率的循环而不是使用 [rate](http://wiki.ros.org/roscpp/Overview/Time) 机制

### Timer Callback

<!-- tabs:start -->

#### **C++ (ROS 2)**

`ROS2` 的回调函数无形参

```cpp
timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&Node::run, this));

using std::chrono_literals::operator ""ms;
timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&Node::timerCallback, this));

timer_period = 0.2  # 单位：秒
self.timer = self.create_timer(timer_period, self.timer_callback)
```

#### **C++ (ROS)**

```cpp
// 定时器回调函数
ros::Timer timer = nh.createTimer(ros::Duration(1.0 / publish_rate), &onTimer, this);
// 类成员函数，方案一
ros::Timer timer = nh.createTimer(ros::Duration(0.1), &Foo::callback, &foo_object); 
// 类成员函数，方案二
ros::Timer timer = nh.createTimer(ros::Duration(0.1), callback);

// 对应的回调函数需含如下指定形参
void onTimer(const ros::TimerEvent& e);
```

#### **Python (ROS 2)**

`ROS 2` 没有`rospy.Rate(5)`，只能通过定时器来实现

#### **Python (ROS)**

```python
import rospy

rospy.Timer(rospy.Duration(2), on_timer)  # 单位：秒


def on_timer(self, event):
    pass
```

<!-- tabs:end -->

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="http://library.isr.ist.utl.pt/docs/roswiki/Clock.html">如何将录制传感器的时间作为时钟源给节点使用？</a>
    </summary>

回放`rosbag`时添加选项`--clock`来发布`/clock`主题，`ROS` client 库则会将此主题作为时钟源提供时间

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2Fclock_Topic">让一个节点使用仿真时间源？</a>
    </summary>

设置参数：`/use_sim_time=true`

```bash
(ROS) $ rosparam set /use_sim_time true
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        在 /use_sim_clock = True 的情况下依然获取系统时间，而不是仿真时间
    </summary>

| ROS(C++)                                                                                                                   | ROS(C++)                                                                   |
|----------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------|
| `ros::WallTime stime;`<br/>`stime = ros::WallTime::now()`<br/>`ros::WallDuration duration = ros::WallTime::now() - stime;` | // 默认已经是系统时间了<br/>`rclcpp::Time t = rclcpp::Clock{RCL_SYSTEM_TIME}.now();` |

</details>

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        ROS 中默认使用的时钟源是？
    </summary>

s 根据实验，应该为`system clock`，即`wall clock`

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        Field of type 'rclcpp::Duration' has private default constructor 导致无法实例化类
    </summary>

字面意思，rclcpp::Duration 的默认构造函数为私有函数，无法被访问，需使用其他构造函数

```cpp
private: // 私有成员
  rcl_duration_t rcl_duration_;
  Duration() = default;
```

</details>

## Reference

| 概要                          | 链接                                                                                                                                                                                                                                    |
|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ROS API 文档                  | [Time](http://docs.ros.org/en/latest/api/rostime/html/classros_1_1Time.html)，[Duration](https://docs.ros.org/en/latest/api/rostime/html/classros_1_1Duration.html)                                                                    |
| ROS2 API 文档                 | [Time](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Time.html)，[Clock](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Clock.html) ，[Duration](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Duration.html) |
| Design：ROS 2 Clock and Time | https://design.ros2.org/articles/clock_and_time.html                                                                                                                                                                                  |
| ROS Clock                   | https://wiki.ros.org/Clock                                                                                                                                                                                                            |
| TF                          | [ROS2](https://docs.ros.org/en/iron/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html)                                                                                                                                  |
