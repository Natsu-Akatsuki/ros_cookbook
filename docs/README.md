<img src="_media/favicon.png" align="right" width="15%">

# ROS cookbook

解决方案索引库：记录基于 ROS（ROS1 和 ROS2）开发过程涉及的相关工具链、代码块，:raised_eyebrow:  欢迎交流（[Issues](https://github.com/Natsu-Akatsuki/ros_cookbook/issues)，[PR](https://github.com/Natsu-Akatsuki/ros_cookbook/pulls)，[Discussions](https://github.com/Natsu-Akatsuki/ros_cookbook/discussions)）

网站见：[ros_cookbook - Docs](https://natsu-akatsuki.github.io/ros_cookbook/)

## Contents

### :child: Beginner

- [Build Tool](Beginner/Build%20Tool.md): How to use Catkin, colcon...
- [CMake and Package](Beginner/CMake%20and%20Package.md): How to write CMake and Package.xml
- [Concepts](Beginner/Concepts.md): ROS concepts
- [Executor and Callback](Beginner/Executor%20and%20Callback.md): How to use multi-thread executor
- [Header and Module](Beginner/Header%20and%20Module.md)：Commonly used header file, module, library (e.g. extension library for Python)
- [Install](Beginner/Install.md): How to install ROS and ROS packages
- [Interface](Beginner/Interface.md): How to use ROS interface (e.g. msg, srv, action)
- [Launch](Beginner/Launch.md): How to use launch file
- [Logger](Beginner/Logger.md): How to use logger
- [Network](Beginner/Network.md): How to implement cross-computer communication
- [Node and Component](Beginner/Node%20and%20Component.md): How to write a basic ROS node and component
- [Parameter Server](Beginner/Parameter%20Server.md): How to use parameter server
- [Record](Beginner/Record.md): How to record and replay ROS interface
- [TF](Beginner/TF.md): How to use the transform
- [Time](Beginner/Time.md): How to use ROS clock sources

### :student: Intermediate

- [DDS](Intermediate/DDS.md)
- [Deployment](Intermediate/Deployment.md)
- [Assertion and Exception](Intermediate/Assertion%20and%20Exception.md)
- [Plugins](Intermediate/Plugins.md)

### :hammer_and_wrench: Software and Tools

- [Gazebo](Software%20and%20Tools/Gazebo.md)
- [RViz](Software%20and%20Tools/RViz.md)
- [Tools](Software%20and%20Tools/Tools.md)

### :memo: Others

- [Code Style](Others/Code%20Style.md)

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        如何在 ROS2 找到对应的 Python API？
    </summary>

根据`C++`的相关代码和 API 进行初筛，然后再查看 [rclpy API 文档](https://docs.ros2.org/latest/api/rclpy/index.html)，找到对应的模块和代码接口，继而实现迁移

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        如何实现 ROS1 和 ROS2 代码的迁移？
    </summary>

- [ ] 过一遍 ChatGPT
- [ ] 经验泛化（谷歌，Stack Overflow，百度，曾经遇到过......）
- [ ] 查看例程

</details>

<details>
    <summary>:question: <b>问题 3：</b>
        <a href="https://design.ros2.org/articles/why_ros2.html">为什么选择 ROS2</a>？
    </summary>
</details>

<details>
    <summary>:question: <b>问题 4：</b>
        如何写 ROS 程序？
    </summary>

- [ ] 明确程序的功能，明确程序的输入和输出
- [ ] 进行快速原型开发，验证程序的功能
- [ ] 优化程序，提高程序的性能

</details>

## Reference

- [ROS2 cookbook](https://github.com/mikeferguson/ros2_cookbook)

## Master

- [ ] 能熟练地在 ROS 框架下进行算法的开发和验证
- [ ] 能熟练地定位和解决 ROS 开发过程中遇到的问题

## Roadmap

- [x] 2023.07.24-2023.07.31 修订所有文档
- [ ] 追加 giscus 评论系统

## Contributors

<a href="https://github.com/Natsu-Akatsuki">
  <img src="https://contrib.rocks/image?repo=Natsu-Akatsuki/ros_cookbook" />
</a>