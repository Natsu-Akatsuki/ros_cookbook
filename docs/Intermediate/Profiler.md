# Profiler

## Memory

### Usage

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

<details>
    <summary>:wrench: <b>用例 2：</b>
        使用 AddressSanitizer 分析内存泄露（Heap-use-after-free）（多线程、共享变量）
    </summary>

步骤 1：配置 `CMakeLists.txt`

```cmake
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -rdynamic -fsanitize=address")
set(CMAKE_BUILD_TYPE "DEBUG")
```

步骤 2：根据 AddressSanitizer 提供的日志和相关调用栈（用于定位是哪个共享变量），可发现是有关多线程的问题，T1647 线程访问了 T6 现成已经析构的内存

```bash
# READ of size 8 at 0x7fcba0501878 thread T1647
# freed by thread T6 here                                    
# previously allocated by thread T6 here
```

具体情况如下图所示：B 线程执行的时候，相应的内存空间被 A 线程析构了一部分（原本有 1000 个元素大小的空间，现在只有 800 个元素大小的空间），以致 B 线程在访问第 1000 个元素时出现问题。

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20231117212537894.png)

解决方案 1：线程 A 和 线程 B 不使用该共享变量

</details>

## Roadmap

- 根据 [AddressSanitizer](https://github.com/google/sanitizers/wiki/AddressSanitizer) 官方文档补充笔记
