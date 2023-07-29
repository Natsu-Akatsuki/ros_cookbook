# Interface

ROS 的进程是通过三种`interface`来实现通信。在使用这些 interface 时，需要使用`IDL`文件进行描述，然后触发`ROS`的工具来生成相应编程语言的源代码

![](http://ros.org/images/wiki/ROS_basic_concepts.png ':size=200 通信机制')

## Install

安装第三方自定义消息类型包

|                      Version                       |                                        apt package                                         |            Type             |
|:--------------------------------------------------:|:------------------------------------------------------------------------------------------:|:---------------------------:|
|                        ROS1                        |                                ros-${ROS_DISTRO}-uuid-msgs                                 |     uuid_msgs/UniqueID      |
|                        ROS2                        | [ros-${ROS_DISTRO}-unique-identifier-msgs](https://github.com/ros2/unique_identifier_msgs) | unique_identifier_msgs/UUID |
| [ROS1](http://wiki.ros.org/diagnostic_msgs) / ROS2 |                             ros-${ROS_DISTRO}-diagnostic-msgs                              |       diagnostic_msgs       |

## Usage

### IDL

`ROS2`有逗号，`ROS1`无逗号语法

|                                              ROS1                                              |                                                           ROS2                                                           |                                   备注                                   |
|:----------------------------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------:|
|                                 int32[] foo<br />int32[5] bar                                  |                                                      int32[<=5] bat                                                      | `ROS1`的定长数组在 C++ 中无法进行`resize()`操作<br />ROS2 支持可变长数组（`bounded arrays`） |
|                                           string foo                                           |                                                      string<=5 bar                                                       |                    ROS2 支持可变长字符串（`bounded strings`）                    |
|                                      int32 X=123（不可修改的常量）                                      |                                                   int32 X=123（不可修改的常量）                                                   |                         常量，程序上不能对它进行修改，命名需要大写                          |
|                                               —                                                |                                                     int32 X 123（默认值）                                                     |                               ROS2 支持默认值                               |
|                                         Header header                                          |                                                  std_msgs/Header header                                                  |                                   —                                    |
| [duration](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Duration.html)  time_from_start | [builtin_interfaces::msg::Duration](https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html) time_from_start |                                                                        |
|           [time](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Time.html) time           |          [builtin_interfaces::msg::Time](https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html) time           |                          ROS1 中`time`这个字段为小写                           |

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行（CLI）
    </summary>

```bash
# >>> 查看某个消息类型的数据结构 >>>
(ROS1) $ ros msg show <message_type>
(ROS2) $ ros2 interface show <message_type>
```

</details>

### Type

`ROS2`的消息类型追加了`msg`，`srv`命令空间；`ROS2`的`ConstPtr`（`boost::shared_ptr`）升格为`ConstSharedPtr`（`std::shared_ptr`）

|                ROS1                |                    ROS2                     |
|:----------------------------------:|:-------------------------------------------:|
|     geometry_msgs::PoseStamped     |     geometry_msgs::**msg**::PoseStamped     |
|      nav_msgs::msg::Odometry       |         nav_msgs::**msg**::Odometry         |
|         Odometry::ConstPtr         |        Odometry::Const**Shared**Ptr         |
|           Odometry::Ptr            |           Odometry::**Shared**Ptr           |
|        uuid_msgs::UniqueID         |    unique_identifier_msgs::**msg**::UUID    |
| autoware_adapi_v1_msgs::ClearRoute | autoware_adapi_v1_msgs::**srv**::ClearRoute |

`ROS2`的头文件采用`.hpp`后缀，追加`msg`修饰，命名方式从`大驼峰`改成`下划线`方式

|                     ROS1                     |                        ROS2                         |
|:--------------------------------------------:|:---------------------------------------------------:|
|   \#include <geometry_msgs/PoseStamped.h>    |   \#include <geometry_msgs/msg/pose_stamped.hpp>    |
| \#include <visualization_msgs/MarkerArray.h> | \#include <visualization_msgs/msg/marker_array.hpp> |

### Topic

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行（CLI）
    </summary>

```bash
# >>> 查看现有的主题（包含订阅的和发布的主题）>>>
(ROS1) $ rostopic list
(ROS2) $ ros2 topic list

# >>> 查看主题的类型 >>>
(ROS1) $ rostopic type <topic_name>
(ROS2) $ ros2 topic type <topic_name>

# >>> 查看主题中的数据 >>>
(ROS1) $ rostopic echo <topic_name>
(ROS2) $ ros2 topic echo <topic_name>
```

</details>

#### Publisher and Subscriber

ROS2 中存在一个 `QoS` 的概念，我们可以通过调整 `QoS Profile` 用来配置需要的通讯质量，从而调用不同的通讯服务。比如希望通讯的数据尽量可靠，则会触发 TCP 通讯方式；希望通讯的数据丢包率低，则会触发 UDP 通讯方式。具体参考 [Here](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

[ROS2 QoS](https://github.com/ros2/rmw/blob/foxy/rmw/include/rmw/qos_profiles.h)：

|              ROS1               |               ROS2                |
|:-------------------------------:|:---------------------------------:|
|          queue_size=5           |      rclcpp::SensorDataQoS()      |
|    queue_size=5, latch=true     | rclcpp::QoS(10).transient_local() |
| queue_size=10, latch=false, ... | rmw_qos_profile_services_default  |

在构造函数中存在的 latch 参数，代表是否自动地将最近的一次数据发送给新的订阅器

> [!note]
>
> `ROS2`的实参顺序跟`ROS1`是不一致的

<details>
    <summary>:wrench: <b>用例 1：</b>
        初始化订阅器和发布器（源程序）
    </summary>

类内函数的参数类型建议为 const T::ConstPtr&（ROS1）和 const T::ConstPtr&（ROS2）

<!-- tabs:start -->

#### **ROS1(C++)**

使用类内函数作为回调函数，具体参考 [Here](http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks)

```cpp

// >>> 声明 >>>
using MSG_T = ...;
void callback(const MSG_T::ConstPtr& msg);
ros::Subscriber sub_;
ros::Publisher pub_;

// >>> 定义 >>>
// 成员函数做回调函数
sub_ = pnh_.subscribe(<主题名>, <队列大小>, &类名::函数名, this); 
// 可调用对象做回调函数
sub_ = pnh_.subscribe<消息类型>(<主题名>, <队列大小>, std::bind(&类名::函数名, this, _1));
// 发布器
pub_ = nh_.advertise<主题类型>(<主题名>, 1, <是否 latch>);
```

#### **ROS2(C++)**

rclcpp::QoS(1) 等价于 queue_size 为 1，消息类型不需要使用 Ptr 等进行修饰

```cpp
// >>> 声明 >>>
using MSG_T = ...;
void callback(const MSG_T::ConstSharedPtr& msg);
rclcpp::Publisher<主题类型>::SharedPtr pub_;
rclcpp::Subscription<主题类型>::SharedPtr sub_;

// >>> 定义 >>>
sub_ = this->create_subscription<主题类型>(<主题名>, rclcpp::QoS(1), std::bind(&类名::函数名, this, std::placeholders::_1));
pub_ = this->create_publisher<主题类型>(<主题名>, 队列长度);
```

#### **ROS1(Python)**

[//]: # (@formatter:off)
```python
self.pub = rospy.Publisher(<主题名>, <主题类型>, queue_size=<队列长度>, latch=<True|False>)
self.sub = rospy.Subscriber(<主题名>, <主题类型>, <回调函数>, queue_size=<队列长度>)
```
[//]: # (@formatter:on)

<!-- tabs:end -->

#### **ROS2(Python)**

> [!attention]
>
> ROS1 和 ROS2 的主题类型和主题名的位置是相反的

[//]: # (@formatter:off)
```python
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

# >>> 发布器 >>>
# 案例一：
# PointCloud2, "/rslidar_points", self.pc_callback, 10
self.pub = self.create_publisher(<主题类型>, <主题名>, <队列长度>)
# 案例二：使用 QoSProfile
# latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
self.pub = self.create_publisher(<主题类型>, <主题名>, qos_profile=<profile>)

# >>> 订阅器 >>>
self.sub = self.create_subscription(<主题类型>, <主题名>, <回调函数>, <队列长度>)
```
[//]: # (@formatter:on)

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://github.com/ros/ros_comm/blob/noetic-devel/tools/topic_tools/sample/simple_lazy_transport.py">取消订阅</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        基于命令行发布主题（CLI）
    </summary>

- [发布数据时带时间戳](http://wiki.ros.org/ROS/YAMLCommandLine#Headers.2Ftimestamps)（BUG：`-f`和`-s`同时使用时，只会替换第一帧的数据）

<!-- tabs:start -->

#### **ROS1**

```bash
$ rostopic pub <topic_name> <topic_type> [args...]

# options：
# -r：   指定发布的频率
# -f：   从yaml文件中读取args
# -s：   需配合-r模式使用，可使用auto和now这两个词的替换符（换句话说命令行中需要有auto或now这几个关键词，否则无效）

$ rostopic pub -s -r 4 /clicked_point geometry_msgs/PointStamped "header: auto
    point:
    x: 0.0
    y: 0.0
    z: 0.0"
    
$ rostopic pub -s --use-rostime -r 4 /clicked_point geometry_msgs/PointStamped "header:
     seq: 0
     stamp: now
     frame_id: ''
   point:
     x: 0.0
     y: 0.0
     z: 0.0"
```

#### **ROS2**

```bash

# 发布数据
# ros2 topic pub [options] <topic_name> <topic_type> [args...]
$ ros2 topic pub -r 10 /rosout rcl_interfaces/msg/Log "{stamp:{sec: 0, nanosec: 0}, level: 0, name: 'MyLogger', msg: 'This is an informational message.'}"
```

<!-- tabs:end -->

</details>

#### Message

<details>
    <summary>:wrench: <b>用例 1：</b>
        构建自定义消息类型（CMakeLists.txt）
    </summary>

<!-- tabs:start -->

#### **ROS2**

```cmake
cmake_minimum_required(VERSION 3.11)
project(<包名>)

# ======================= COMPILE OPTION =======================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# ======================= COMPILE OPTION =======================

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 同时支持srv和msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorGPS.msg"
  DEPENDENCIES 
  	geometry_msgs std_msgs
  )

ament_auto_package()
```

> [!attention]
>
> `rosidl_generate_interfaces` 中的目标文件名需`${PROJECT_NAME}`作为 basename，否则会有 [Here](https://github.com/ros2/rosidl/issues/441) 的问题 \
> `ROS2`对 interface 的命名有要求，需要大驼峰

#### **ROS1**

```cmake
cmake_minimum_required(VERSION 3.14)
project(<包名>)

# ======================= COMPILE OPTION =======================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# ======================= COMPILE OPTION =======================

find_package(catkin REQUIRED COMPONENTS
  message_generation
  # 生成消息时涉及的依赖：
  geometry_msgs
  std_msgs
  )

# 默认 msg 目录下的 IDL 文件
add_message_files(
  FILES
  VectorMapPrimitive.msg 
  VectorMapRoute.msg
  VectorMapSegment.msg
)

# 其他目录下的 IDL 文件，暂不支持一个宏中多次使用 DIRECTORY
add_message_files(
  DIRECTORY motion/msg
  FILES MotionState.msg
)

add_service_files(
  FILES
  SetRoute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://natsu-akatsuki.github.io/ros_cookbook/#/Beginner/CMake%20and%20Package?id=packagexml">构建自定义消息类型（package.xml）</a>
    </summary>
</details>

#### Namespace

<details>
    <summary>:wrench: <b>用例 1：</b>
        如何重映射私有主题？
    </summary>

<!-- tabs:start -->

#### **ROS1(C++)**

```cpp
// 若要重映射如下主题，则 launch 需要使用 private namespace
pnh_ = ros::NodeHandle("~");
// 对应的主题名不需要额外加 "~"
pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/map", 1, true);
```

#### **ROS1(launch.xml)**

```xml

<node pkg="map_loader" type="pointcloud_map_loader" name="pointcloud_map_loader">
    <remap from="~output/pointcloud_map" to="/map/pointcloud_map"/>
</node>
```

<!-- tabs:end -->

</details>

#### Subscriber Number

- 得到订阅该主题的订阅器个数（@ref: [here](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1PublisherBase.html)）

```python
# ROS1 C++    
if (pub.getNumSubscribers() > 0) {;}
# ROS1 Python
if pub.get_num_connections() > 0:

# ROS2 C++
if (pub_->get_subscription_count() < 1) {;}
# ROS2 Python
if pub.get_subscription_count() > 0:  
```

#### Statistics

<details>
    <summary>:wrench: <b>用例 1：</b>
        将订阅到的数据进行分析，然后进行发布
    </summary>

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

// manually enable topic statistics via options  
auto options = rclcpp::SubscriptionOptions();  
options.topic_stats_options.state = rclcpp::TopicStatisticsState::NodeDefault;  

// configure the collection window and publish period (default 1s)  
options.topic_stats_options.publish_period = std::chrono::seconds(10);  

// configure the topic name (default '/statistics')  
// options.topic_stats_options.publish_topic = "/topic_statistics"  

auto callback = [this](std_msgs::msg::String::SharedPtr msg) {  
    this->topic_callback(msg);  
  };  

subscription_ = this->create_subscription<std_msgs::msg::String>(  
  "topic", 10, callback, options);
```

```cpp
// 构建订阅器 option（MutallyExclusive）组（使用共享指针）
rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node *node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
```

</details>

#### Sync

<details>
    <summary>:wrench: <b>用例 1：</b>
        使用 TimeSynchronizer 进行消息同步
    </summary>

```cpp
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  // 定义需要同步的主题
  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(std::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        使用 Custom Synchronizer 进行同步
    </summary>

```cpp
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
```

</details>

### Service

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行（CLI）
    </summary>

```bash
# >>> 调用服务 >>>
# ros2 service call /clear std_srvs/srv/Empty
(ros2) ros2 service call <service_name> <service_type> [arguments]
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        初始化服务（C++，Python）
    </summary>

<!-- tabs:start -->

#### **ROS2(C++)**

ROS2 倾向于使用智能指针

```cpp
// >>> 客户端 >>>
rclcpp::Client<包名::srv::服务名>::SharedPtr client_;
client_ = node->create_client<CooperateCommands>(<服务类型>, [rmw_qos_profile_services_default]);

// >>> 服务端 >>>
rclcpp::Service<包名::srv::服务名>::SharedPtr service_;
service_ = node->create_service<服务类型>(<服务名>, &回调函数);
```

#### **ROS1(C++)**

```cpp
// >>> 客户端 >>>
ros::ServiceClient client_;
client_ = pnh_.serviceClient<服务类型>(<服务名>);

// >>> 服务端 >>>
ros::ServiceServer service_;
service_ = pnh_.advertiseService(<服务类型>, <回调函数>);
```

#### **ROS1(Python)**

```python
# >>> 客户端 >>>
rospy.wait_for_service('服务名')
service = rospy.ServiceProxy('服务名', 服务类型)

# >>> 服务端 >>>
rospy.Service('服务名', 服务类型, 回调函数)


def onCallback(self, req):
    pass
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        等待服务（C++，Python）
    </summary>

<!-- tabs:start -->

#### **ROS1(C++)**

```cpp
// 等待 1s
while (!ros::service::waitForService(<服务名>, ros::Duration(1))) {
  if (!ros::ok()) {
    ROS_ERROR("Interrupted while waiting for service.");
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("Waiting for service... [" << module_name << "]");
}
```

#### **ROS2(C++)**

ROS1 的 wait() 是普通函数，而 ROS2 的是成员函数

```cpp
// concise
using std::chrono_literals::operator ""s;
client->wait_for_service(1s)

// complex
while (!client->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for service... [" << module_name << "]");
}
```

#### **ROS1(Python)**

不能仿效 C++的形式，尽管可以指定 timeout，但它是直接抛出异常 ROSException，而非返回 true/false

```python

import rospy

rospy.init_node( < 节点名 >)
rospy.wait_for_service( < 服务名 >)
```

<!-- tabs:end -->

#### **ROS2(Python)**

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    if not rclpy.is_shutdown():
        node.get_logger().error("Interrupted while waiting for service.")
        rclpy.shutdown()
        return
    node.get_logger().info("Waiting for service... [{}]".format(module_name))
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        回调函数（C++，Python）
    </summary>

|  —  |                                                       ROS1                                                       |                                                                     ROS2                                                                     |
|:---:|:----------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------:|
| 返回值 |                                                      `bool`                                                      |                                                                    `void`                                                                    |
| 形参  |                                                 引用（无 const，无指针）                                                  |                                                                 const + 智能指针                                                                 |
| 示例  | bool on_clear_route(<br/>      ClearRoute::Service::Request &req,<br/>      ClearRoute::Service::Response &res); | void on_clear_route(<br/>    const ClearRoute::Service::Request::SharedPtr req,<br/>    const ClearRoute::Service::Response::SharedPtr res); |

<!-- tabs:start -->

#### **ROS1(C++)**

```cpp
// 类成员函数（无模板参数）
srv_auto_mode_ = pnh->advertiseService("服务名", &RTCInterface::onAutoModeService, this);
bool onAutoModeService(AutoMode::Request &request, AutoMode::Response &response);
```

#### **ROS2(C++)**

```cpp
#include <memory>

void callback(const std::shared_ptr<包名::srv::服务类型::Request> request,
          std::shared_ptr<包名::srv::服务类型::Response> response)
{
  response->sum = request->a + request->b; 
}

rclcpp::Service<包名::srv::服务类型>::SharedPtr service =
    node->create_service<包名::srv::服务类型>("服务名", &callback);
```

#### **ROS1(Python)**

TODO

#### **ROS2(Python)**

```python
self.srv = self.create_service( < 服务类型 >, < 服务名 >, self.callback)

def callback(self, request, response):
    return response
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        调用服务（C++，Python）
    </summary>

<!-- tabs:start -->

#### **ROS1(C++)**

`call()` 函数只能接受左值

```cpp
# 步骤一：实例化请求
请求类 srv;
# 步骤二：触发请求
if (client.call(srv)) {
	ROS_INFO("Sum: %ld", (long int)srv.response.sum);
}
else {
	ROS_ERROR("Failed to call service add_two_ints");
	return 1;
}
```

#### **ROS2(C++)**

C++中`ROS1`的服务是同步的，而`ROS2`的则为异步

```cpp
# 步骤一：实例化请求
auto request = std::make_shared<包名::srv::服务类型::Request>();
// TODO
# 步骤二：异步触发请求
client->async_send_request(request);
auto result = client->async_send_request(request);

// Wait for the result.
if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS){  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "...");
} else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service [服务名]");
}
```

#### **ROS1(Python)**

TODO

#### **ROS2(Python)**

TODO

<!-- tabs:end -->

</details>

### Action

TODO

### Others

<details>
    <summary>:wrench: <b>用例 1：</b>
        初始化消息类型（ROS2）
    </summary>

```cpp
#include <rosidl_generator_cpp/message_initialization.hpp>

sensor_msgs::msg::PointCloud2 msg{rosidl_runtime_cpp::MessageInitialization::ALL};
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        生成消息类型的匿名对象（ROS2）
    </summary>

```cpp
# set__data
std_msgs::msg::String().set__data("hello world!"));
std_msgs::msg::Int64().set__data(1000);

# build
geometry_msgs::build<geometry_msgs::Point>().x(e.at(0).x()).y(e.at(0).y()).z(0.0));
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        ROS2 中编译自定义消息类型时：fatal error: autoware_api_msgs/msg/detail/header__struct.hpp: No such file or directory 
    </summary>

将类型 Header header-> std_msgs/Header header

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        基于元编程获取某个消息类型的字符串表示
    </summary>

```cpp
// >>> ROS1 >>>
ros::message_traits::datatype<am_rviz_plugins_msgs::OverlayMenu>

// >>> ROS2 >>>
rosidl_generator_traits::data_type<am_rviz_plugins_msgs::OverlayMenu>
```

</details>

## FAQ

## Supplementary materials

### Topic

- Official demo for [ROS1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and [ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- Topic statistics for [ROS1](https://wiki.ros.org/Topics#Topic_statistics) and [ROS2](https://docs.ros.org/en/humble/Concepts/About-Topic-Statistics.html)
- Sync demo for [ROS1](http://wiki.ros.org/message_filters)

### Message

- Official document for [ROS1](https://wiki.ros.org/msg) and [ROS2](https://docs.ros.org/en/iron/Concepts/About-ROS-Interfaces.html#)
- [Autoware.Universe name guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/message-guidelines/)
- [Autowarw adapting-message-definitions](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/main/docs/developer_guide/knowhow/PortingToROS2.md#adapting-message-definitions)

### Service

- Official C++ demo：[ROS1](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29), [ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- Official Python demo：[ROS1](http://wiki.ros.org/rospy/Overview/Services), [ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Services between ROS1 and ROS2](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/)