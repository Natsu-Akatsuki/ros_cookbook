# Assertion and Exception

# Usage

## Assertion

```cpp
// >>> ROS1 >>>
ROS_ASSERT(pcl_in.size() < 400000);

// >>> ROS2 >>>
#include "rcpputils/asserts.hpp"
rcpputils::assert_true(pcl_in.size() < 400000);
```

## Exception

```cpp
// >>> ROS1 >>>
ros::Exception

// >>> ROS2 >>>
rclcpp::ParameterTypeException
```

# Reference

- cpp exception for [ROS1](http://wiki.ros.org/roscpp/Overview/Exceptions)