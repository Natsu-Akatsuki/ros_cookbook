# Tools

<details>
    <summary>:wrench: <b>工具 1：</b>
        <a href="https://github.com/eduidl/rtui">TUI for ROS</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>工具 2：</b>
        <a href="https://github.com/awslabs/ros2-migration-tools">Tools for migrating packages from ROS1 to ROS2</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>工具 3：</b>
        <a>Image publisher：将一张图片进行发布</a>      
    </summary>

```bash
# 重映射主题名
(ROS1) $ rosrun image_publisher image_publisher img.jpeg __name:= __ns:= image_raw:=/sensing/camera0/image_raw
```

</details>