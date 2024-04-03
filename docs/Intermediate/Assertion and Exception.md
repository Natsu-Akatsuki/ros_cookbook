# Assertion and Exception

## Assertion

| ROS2                                                         | ROS1                                |
| ------------------------------------------------------------ | ----------------------------------- |
| #include "rcpputils/asserts.hpp"<br/>rcpputils::assert_true(pcl_in.size() < 400000); | ROS_ASSERT(pcl_in.size() < 400000); |

## Type

| ROS2                                                        | ROS1                                                        |
| ----------------------------------------------------------- | ----------------------------------------------------------- |
| `const char *name = rosidl_generator_traits::name<MsgT>();` | `const char *name = ros::message_traits::datatype<MsgT>();` |

## Exception

| ROS2(C++)                      | ROS(C++)       |
|:-------------------------------|:---------------|
| rclcpp::ParameterTypeException | ros::Exception |

## Reference

| 摘要     | ROS2 | ROS1                                           |
| -------- | ---- | ---------------------------------------------- |
| 官方教程 | TODO | http://wiki.ros.org/roscpp/Overview/Exceptions |