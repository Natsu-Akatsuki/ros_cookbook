# Foxglove Studio

## Feature

- 支持在 ROS2 中可视化 ROS1 包，而无需转换为 .db 包
- 自动识别可显示的主题

## Install

```bash
# 安装 Foxglove bridge 
$ sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        在线可视化 ROS 主题
    </summary>

<!-- tabs:start -->

#### **ROS2**

```bash
# 运行 foxglove_bridge 节点
$ ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

#### **ROS1**

TODO

<!-- tabs:end -->

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20240327103805383.png ':size=800')


</details>

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        <a href="https://kkgithub.com/foxglove/studio/issues/5870#issuecomment-1693580785">为什么无法显示点云？</a>
    </summary>
</details>

## Reference

| 摘要   | 链接                                                                 |
|------|--------------------------------------------------------------------|
| 官方地址 | https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2/ |

