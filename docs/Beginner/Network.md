# Network Communication

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        实现主机与主机的 ROS 通信
    </summary>

<!-- tabs:start -->

#### **ROS2**

TODO

#### **ROS1**

```bash
# 将如下内容添加到 ~/.bashrc

# >>> 主机配置 >>>
# 获取主机的 IP 地址
ROS_MASTER_IP=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311

# >>> 从机配置 >>>
# ROS_MASTER_IP：对应为主机的 IP 地址
export ROS_MASTER_URI=http://<ROS_MASTER_IP>:11311
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        实现主机和容器的 ROS 通信
    </summary>

<!-- tabs:start -->

#### **ROS2**

```bash
# 已测试如下组合
# 主机 Noetic 和 容器 Noetic
# 主机 Noetic 和 容器 Melodic

（主机）$ docker run -it --net=host --rm <镜像名>

# 用例：主机和容器端可互换
（主机）$ roscore
（主机）$ rosrun roscpp_tutorials talker
（容器）$ rosrun roscpp_tutorials listener
```

#### **ROS1**

```bash
# 测试环境（humble）

# 不需要 --ipc=host --net=host --pid=host -v /dev/shm:/dev/shm
（主机）$ docker run -it --net=host --rm <镜像名>

# 安装依赖（可互相切换）
$ sudo apt install ros-${ROS_DISTRO}-rmw-connextdds
$ RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker
$ RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp listener

# 实测如下 DDS 不起作用
$ sudo apt install ros-${ROS_DISTRO}-rmw-fastrtps-cpp
$ RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_cpp talker
$ RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_cpp listener

$ sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
$ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker
$ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp listener

# 缺乏可用证书
$ RMW_IMPLEMENTATION=rmw_gurumdds_cpp ros2 run demo_nodes_cpp talker
$ RMW_IMPLEMENTATION=rmw_gurumdds_cpp ros2 run demo_nodes_cpp listener
```

<!-- tabs:end -->

实测 ROS1 和 ROS2 都不需要如下 Dockerfile 配置

```dockerfile
# Dockerfile
ARG USER_NAME=<用户名>
RUN useradd ${USER_NAME} -m -G sudo -u 1000 -s /bin/bash && yes ${USER_NAME} | passwd ${USER_NAME}
USER ${USER_NAME}
```

已参考资料：

- [ROS2 topics on Docker detected by host but can't subscribe](https://github.com/eProsima/Fast-DDS/issues/2956)

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a href="http://wiki.ros.org/ROS/NetworkSetup">配置多机时间同步（主从模式， 从机时间跟主机对齐）</a>
    </summary>

```bash
# 主机
$ sudo apt install ntp
# 从机
$ sudo ntpdate <主机的 IP 地址>
```

</details>

## Reference

| 摘要   | ROS2                                                          | ROS1                                 |
|------|---------------------------------------------------------------|--------------------------------------|
| 网络配置 | https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html | http://wiki.ros.org/ROS/NetworkSetup |
