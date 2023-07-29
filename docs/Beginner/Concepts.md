# Concepts

> Someone who is sophomoric about concepts and reasoning may try things randomly, and wastes a lot of time.

|    术语    |                               描述                                |
|:--------:|:---------------------------------------------------------------:|
|  Nodes   |   在 ROS 中进程被称为节点（Node），一般调用了 ROS 的客户端库（e.g., roscpp 或 rospy）    |
| Messages |              消息是一个数据结构。节点通过传递消息（messages）来实现相互的通信。              |
|  Topics  | 消息通过发布和订阅机制进行传输。节点根据对应的主题名（topic）来将数据进行发布。主题名是用来标识 message 的内容。 |
|   Bags   |                       ROS 系统中用 bag 包来存储数据                       |

# Reference

- Concepts in [ROS1](https://wiki.ros.org/ROS/Concepts)