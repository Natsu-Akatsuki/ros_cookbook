# Parameter Server

<details>
    <summary>:wrench: <b>用例 1：</b>
        如何读取私有参数？需使用私有句柄（NodeHandle）
    </summary>

```xml
<!-- 若要读取如下的参数，则NodeHandle需要使用private namespace -->
<node pkg="multi_object_tracker" type="multi_object_tracker_node" name="multi_object_tracker">
    <param name="publish_rate" value="$(arg publish_rate)"/>
</node>
```

```cpp
pnh_ = ros::NodeHandle("~")
pnh_.param<float>("publish_rate", publish_rate_, 1.0);
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        常用命令行（CLI）
    </summary>

```bash
# 设置参数
(ROS1) $ rosparam set /use_sim_time true
(ROS2) $ ros2 param set <node_name> <param_name> <value>

# 显示参数值
(ROS1) $ rosparam get <param_name>
(ROS2) $ ros2 param get <node_name> <param_name>

# 显示某个节点的ROS参数
(ROS2) $ ros2 param list <node_name>

(ROS1) $ rosrun rqt_reconfigure rqt_reconfigure
(ROS2) $ ros2 run rqt_reconfigure rqt_reconfigure

# 从文件中导入参数
(ROS1) $ rosparam load <file> [namespace]
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        读取参数服务器中的参数（源程序）
    </summary>

|   ROS1   |               ROS2               |
|:--------:|:--------------------------------:|
| hasParam |          has_parameter           |
| getParam | declare_parameter, get_parameter |
|  param   |        declare_parameter         |

```cpp
// 判断：判断参数服务器是否含有某个参数
bool result = nh_.hasParam("map_frame");

// 取值：ret(bool)
// note：非模板函数
std::string map_frame_;
bool result = nh_.getParam("map_frame", map_frame_);

// 取值：ret(bool) + default
nh_.param("map_frame", map_frame_);
pnh_.param<std::string>("publish_rate", publish_rate_, "base_link");
pnh_.param<bool>("use_height", use_height_, false);
pnh_.param<int>("min_cluster_size", min_cluster_size_, 3);

// 取值：ret(val) + default
p.forward_path_length = pnh_.param("backward_path_length", 5.0);
```

```cpp
// 判断：判断参数服务器是否含有某个参数
bool result = this->has_parameter("map_frame");

// 取值：ret(val) + default
p.forward_path_length = declare_parameter("forward_path_length", 100.0);

// 取值：ret(val)
bool result = declare_parameter<std::string>("map_frame");
auto result = node.get_parameter("参数名").as_double();
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        动态参数服务器（Dynamic Parameter Server）
    </summary>

- 追加描述信息和设置取值范围（Python）

```python
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import IntegerRange, FloatingPointRange

# 设置取值范围
range = FloatingPointRange()  # IntegerRange()

range.from_value = -180.0
range.to_value = 180.0
range.step = 0.01

descriptor = ParameterDescriptor()
descriptor.type = ParameterType.PARAMETER_STRING
descriptor.description = "...."
descriptor.floating_point_range = [range]

self.declare_parameter("参数名", "默认参数值", descriptor)
```

- 追加描述信息和设置取值范围（C++）

```CPP
rcl_interfaces::msg::ParameterDescriptor descriptor;
rcl_interfaces::msg::IntegerRange range;

range.set__from_value(0).set__to_value(100).set__step(1);
descriptor.integer_range= {range};

this->declare_parameter("参数名", 默认参数值, descriptor);
```

- 参数更新的回调函数（Python）

```python
from rcl_interfaces.msg import SetParametersResult


def __init__(self):
    self.add_on_set_parameters_callback(self.param_callback)


def param_callback(self, params):
    result = SetParametersResult()

    # 是否触发对参数的更新


result.successful = True
return result
```

- 参数更新的回调函数（C++）

```cpp
rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p) {

  if (get_param(p, "debug", debug_)) {
    RCLCPP_DEBUG(get_logger(), "Setting debug to: %d.", debug_);
  }

  if (get_param(p, "unit_axis", unit_axis_)) {
    if (unit_axis_ == "x") {
      unit_vec_ = Eigen::Vector3d::UnitX();
    } else if (unit_axis_ == "y") {
      unit_vec_ = Eigen::Vector3d::UnitY();
    } else if (unit_axis_ == "z") {
      unit_vec_ = Eigen::Vector3d::UnitZ();
    } else {
      unit_vec_ = Eigen::Vector3d::UnitZ();
    }
    RCLCPP_DEBUG(get_logger(), "Setting unit_axis to: %s.", unit_axis_.c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
```

</details>

## Reference

- ROS1：[Parameter Server API](http://docs.ros.org/en/indigo/api/roscpp/html/classros_1_1NodeHandle.html)，[Parameter Server](http://wiki.ros.org/roscpp/Overview/Parameter%20Server)
- [ROS1->ROS2 Parameter Server Design](https://design.ros2.org/articles/ros_parameters.html)
- CLI document for [ROS1](https://wiki.ros.org/rosparam#rosparam_list) and [ROS2](https://docs.ros.org/en/foxy/How-To-Guides/Using-ros2-param.html)