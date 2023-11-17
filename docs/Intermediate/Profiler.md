# Profiler

## Memory

<details>
    <summary>:wrench: <b>用例 1：</b>
        程序级监控节点进程使用的物理内存，通过主题的方式发布数据，PlotJuggler 可视化数据
    </summary>


![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20231117165131595.png ':size=850 PlotJuggler' )

<!-- tabs:start -->

#### **cpp naive demo**

```cpp
#include "ros_debug.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "...");
  ros::NodeHandle nh("~");
  // ... 
  auto ros_debug = ros_debug::rosDebug(&nh);
  // ...
  ros::spin();  
  return 0;
}
```

#### **Header**

```cpp
#ifndef ROS_DEBUG_H
#define ROS_DEBUG_H

#include <ros/ros.h>
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/Float64.h>

namespace ros_debug {

  class rosDebug {
  public:
    rosDebug(ros::NodeHandle *nh) : nh_(nh) {
      memoryTimer_ = nh_->createTimer(ros::Duration(0.1), &rosDebug::memoryTimerCallback, this);
      memoryPublisher_ = nh_->advertise<std_msgs::Float64>("/debug/memoryMB_usage", 1);
    }

    /**
    * @brief 获取当前进程的物理内存使用量 (VmRSS)
    * @return 返回物理内存使用量 (单位: KB)
    */
    long getPhysicalMemoryUsage() {
      std::string line;
      std::string procStatusFile = "/proc/" + std::to_string(getpid()) + "/status";
      std::ifstream stream(procStatusFile.c_str());
      long memory = 0;

      if (stream.is_open()) {
        while (std::getline(stream, line)) {
          if (line.substr(0, 6) == "VmRSS:") {
            std::istringstream iss(line);
            std::string temp;
            iss >> temp >> memory;
            break;
          }
        }
      }

      return memory;
    };

    void memoryTimerCallback(const ros::TimerEvent &) {
      long memoryUsage = getPhysicalMemoryUsage();
      double memoryUsageMB = memoryUsage / 1024.0;
      std_msgs::Float64 memoryUsageMsg;
      memoryUsageMsg.data = memoryUsageMB;
      memoryPublisher_.publish(memoryUsageMsg);
    };
    
  private:
    ros::NodeHandle *nh_;
    ros::Timer memoryTimer_;
    ros::Publisher memoryPublisher_;
  };

}
#endif //ROS_DEBUG_H
```

<!-- tabs:end -->

</details>





