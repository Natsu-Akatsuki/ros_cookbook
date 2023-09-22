# Time

> [!attention]
>
> 应留意因类型不同而导致的精度丢失

## Usage

### Clock

only for ROS 2

|  版本   |    ROS     |    ROS 2     |
|:-----:|:----------:|:------------:|
| 默认时钟源 | wall clock | system clock |

<!-- tabs:start -->

#### **默认时钟源**

```cpp
rclcpp::Clock ros_clock{RCL_ROS_TIME};
rclcpp::Time ros_clock_stamp{0, 0, RCL_ROS_TIME};
RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", ros_clock_stamp.seconds());
```

#### **wall clock**

```cpp
rclcpp::Clock system_clock{RCL_SYSTEM_TIME};
rclcpp::Time system_clock_stamp{0, 0, RCL_SYSTEM_TIME};
RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", system_clock_stamp.seconds());
```

#### **system clock**

```cpp
rclcpp::Clock steady_clock{RCL_STEADY_TIME};
rclcpp::Time steady_clock_stamp{0, 0, RCL_STEADY_TIME};
RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", steady_clock_stamp.seconds());
```

<!-- tabs:end -->

## Time

<details>
    <summary>:wrench: <b>用例 1：</b>
        创建时间戳对象
    </summary>

<!-- tabs:start -->

#### **C++ (ROS)**

```cpp
// >>> 双形参 >>>
// uint32_t, uint32_t -> Time
Time(uint32_t _sec, uint32_t _nsec)

// >>> 单一形参 >>>
// double -> Time
ros::Time(<data>)
ros::Time().fromSec(<data>)

// uint64/int64_t -> Time
ros::Time().fromNSec(<data>)

// >>> 无参构造 >>>
ros::Time t = ros::Time::now();
```

#### **C++ (ROS 2)**

> [!note]
>
> 双形参用法中：uint32_t (ROS) VS int32_t (ROS 2)

ROS 2 没有将双精度浮点型（以 s 为单位）的时间戳转换为 Time 对象的接口，可以先将其转换为 int64（以 ns 为单位）的时间戳，但该接口会丢失精度

```cpp
// >>> 双形参 >>>
// int32_t, unint32_t -> Time
Time(int32_t seconds, uint32_t nanoseconds, rcl_time_source_type_t clock=RCL_SYSTEM_TIME)

// >>> 单形参 >>>
// int64_t -> Time 
Time(int64_t nanoseconds, rcl_time_source_type_t clock=RCL_SYSTEM_TIME)
rclcpp::Time t(static_cast<int64_t>(seconds * 1e9));

// 自建消息类型 -> Time
Time(const builtin_interfaces::msg::Time &time_msg)

// >>> 无参构造 >>>
rclcpp::Time t = now();
rclcpp::Time t = this->now();
rclcpp::Time t = this->get_clock()->now();
// 显式指明时钟源
rclcpp::Time t = rclcpp::Clock{RCL_ROS_TIME}.now();
```

#### **Python (ROS)**

```python
# 仅支持整型
rospy.Time(secs=0, nsecs=1000000)
```

#### **Python (ROS 2)**

```python
self.get_clock().now();
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        Time 类型 <-> 内建消息类型 <-> 秒/纳秒
    </summary>

<!-- tabs:start -->

#### **ROS (C++)**

```cpp
// Time -> double（单位：second）
double timestamp = msg->header.stamp.toSec();
double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

// Time -> uint64_t（单位：ns）
uint64_t timestamp = msg->header.stamp.toNSec();
```

#### **ROS 2 (C++)**

```cpp
// 内建消息类型 -> Time 对象 -> 秒
double timestamp = rclcpp::Time(msg->header.stamp).seconds();

// 内建消息类型 -> Time 对象 -> 纳秒
uint64_t rclcpp::Time(msg->header.stamp).nanoseconds();

// Time 对象 -> 内建消息类型
msg.header.stamp = timestamp.to_msg()
```

#### **ROS (Python)**

```python
# Time -> double（单位：second）
timestamp = (msg.header.stamp).secs + (msg.header.stamp).nsecs * 1e-9;
```

#### **ROS 2 (Python)**

```cpp
# Time 对象 -> 内建消息类型
self.get_clock().now().to_msg();
```

<!-- tabs:end -->

</details>

## Duration

<details>
    <summary>:wrench: <b>用例 1：</b>
        Sleep
    </summary>

<!-- tabs:start -->

#### **C++ (ROS)**

```cpp
ros::Duration(0.5 /*unit: sec*/).sleep();
```

#### **C++ (ROS 2)**

```cpp
rclcpp::sleep_for(std::chrono::milliseconds(500));
```

#### **Python (ROS)**

```python
rospy.sleep(0.5)
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        浮点数 <-> Duration
    </summary>

<!-- tabs:start -->

#### **C++ (ROS)**

```cpp
// >>> 浮点数 -> Duration >>>
ros::Duration t = ros::Duration(0.5 /*unit: sec*/);
```

#### **C++ (ROS 2)**

```cpp
// >>> 浮点数 -> Duration >>>
// Duration(int32_t seconds, uint32_t nanoseconds)
rclcpp::Duration t = rclcpp::Duration(0, 0);
rclcpp::Duration d = rclcpp::Duration::from_seconds(0.5) // 版本需大于 foxy

// >>> Duration -> 浮点数 >>>
double seconds = d.seconds();
```

#### **Python (ROS)**

```python
rospy.Duration(0.02)  # 0.02s
```

#### **Python (ROS 2)**

```python
from rclpy.duration import Duration

Duration(seconds=0).to_msg()
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        内建消息类型 / C 库消息类型 -> Duration
    </summary>

<!-- tabs:start -->

#### **cpp (ROS2)**

```cpp
// Duration(const builtin_interfaces::msg::Duration &duration_msg)
auto t = rclcpp::Duration(next_pt.time_from_start)
// Duration(rcl_duration_value_t nanoseconds)    
auto t = rclcpp::Duration(time_ns /*rcutils_duration_value_t*/); 
```

<!-- tabs:end -->

</details>

## Rate and Timer

推荐使用 [Timer](http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers) 来实现特定频率的循环而不是使用 [rate](http://wiki.ros.org/roscpp/Overview/Time) 机制

### Timer Callback

<!-- tabs:start -->

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

#### **C++ (ROS 2)**

`ROS 2` 的回调函数无形参

```cpp
timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&Node::run, this));

using std::chrono_literals::operator ""ms;
timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&Node::timerCallback, this));

timer_period = 0.2  # 单位：秒
self.timer = self.create_timer(timer_period, self.timer_callback)
```

#### **Python (ROS)**

```python
import rospy

rospy.Timer(rospy.Duration(2), on_timer)  # 单位：秒


def on_timer(self, event):
    pass
```

#### **Python (ROS 2)**

`ROS 2` 没有`rospy.Rate(5)`，只能通过定时器来实现

<!-- tabs:end -->

## Time Source

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="http://library.isr.ist.utl.pt/docs/roswiki/Clock.html">如何将录制传感器的时间作为时钟源给节点使用？</a>
    </summary>

回放`rosbag`时添加选项`--clock`来发布`/clock`主题，`ROS` client 库则会将此主题作为时钟源提供时间（@[here]()）

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

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        ROS 中默认使用的时钟源是？
    </summary>

根据实验，应该为`system clock`，即`wall clock`

</details>

## Reference

- [Design：ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
- ROS API 文档：[Time](http://docs.ros.org/en/latest/api/rostime/html/classros_1_1Time.html)
- ROS 2 API 文档：[Time](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Time.html)，[Clock](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Clock.html#abe5646eb46910ea5bda2486d082a31ab)  ，[Duration](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Duration.html)
- [ROS 和 Time 的博客](https://nu-msr.github.io/me495_site/time_in_ros.html)
