# Record

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用 API
    </summary>

```bash
# >>> 回放 >>>
(ROS1) $ rosbag play <包名>
# 串行回放多个 bag
(ROS1) $ rosbag play <包名1> <包名2> <...>
(ROS2) $ ros2 bag play <包目录>

# >>> 只发布特定主题的消息 >>
(ROS1) $ rosbag play school.bag --topics /rslidar_points

# >>> 主题重映射 >>>
(ROS1) $ rosbag play school.bag /rslidar_points:=/velodyne_points
(ROS2) $ ros2 bag play school --remap /rslidar_points:=/velodyne_points

# 只发布特定主题的消息 + 主题重映射（需要分两步：不能直接在 --topics 选项中加入重映射）
(ROS1) $ rosbag play school.bag --topics /rslidar_points /rslidar_points:=/velodyne_points
(ROS1) $ rosbag play flash_light.bag --topics /camera_1/image_raw /imu/data /camera_1/image_raw:=/rgb/image

# >>> 指定位置播放 >>>
(ROS1) $ rosbag play <包名> -s 50
(ROS2) $ ros2 bag play <包名> --start-offset 100

# >>> 基于 rosbag 提供时钟源 >>>
(ROS1) $ rosbag play <包名> --clock

# >>> 录制 >>>
(ROS1) $ rosbag record <主题名>
# ROS2 导出的是一个文件夹
(ROS2) $ ros2 bag record -a

# >>> 裁剪 >>>
# 这种时刻指的是 ROS 时间戳，类似 1576119471.511449 
(ROS1) $ rosbag filter <输入包名> <输出包名> "t.to_sec() < 某个时刻 and t.to_sec() > 某个时刻"

# >>> 压缩和解压 >>>
# --lz4：指定用 lz4 压缩，实测不需要显式解压即能 rosbag play
(ROS1) $ rosbag compress/decompress <待压缩的包名>

# >>> 只播放特定一段时间的数据 >>>
(ROS1) $ rosbag play -u <秒>
(ROS2 iron) $ rosbag play --playback-until-sec <秒>

# >>> 等待所有主题都被订阅时才发布数据 >>>
(ROS1) $ rosbag play <包名> --wait-for-subscribers
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        实现 ROS1 和 ROS2 包的相互转换
    </summary>
Python 版本需 3.8+

1）方案 1：基于 `ros_bridge` ，通过回放 `ROS2` 包，录制 `ROS1` 包 \
2）方案 2：使用 [rosbags](https://gitlab.com/ternaris/rosbags) 提供的 API

```bash
$ pip3 install rosbags

# ROS1 包转换为 ROS2 包
$ rosbags-convert <ROS1包名> --dst <ROS2导出路径>
$ rosbags-convert <ROS2包名> --dst <ROS1导出路径>
```

> [!note]
>
> 暂不支持`ROS2`自定义消息类型->`ROS1`自定义消息类型的转换

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        导入和导出 bag 包（源程序）
    </summary>

> [!attention]
>
> 注意 `read_message` 读取的时间戳是 `rosbag` 获取信息时候的时间戳，而不是传感器发布数据时的时间戳

```python
import rosbag
from tqdm import tqdm


class BagReader:
    def __init__(self):
        pass

    def run(self):
        intput_file = "包名"
        output_file = "包名"
        input_bag = rosbag.Bag(intput_file, 'r')
        output_bag = rosbag.Bag(output_file, 'w')

        for topic, msg, t in tqdm(input_bag.read_messages(), total=input_bag.get_message_count()):
            output_bag.write("主题名", "主题数据", t)

        input_bag.close()
        output_bag.close()


if __name__ == '__main__':
    bag_reader = BagReader()
    bag_reader.run()
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        为 bag 包追加 TF信息（源程序），主题为 `/tf`
    </summary>

```python
from tf2_msgs.msg import TFMessage


def set_transform(header, frame_id, child_frame_id, x, y, z, q):
    gnss_transform = TransformStamped()
    gnss_transform.header.stamp = header.stamp
    gnss_transform.header.frame_id = frame_id
    gnss_transform.child_frame_id = child_frame_id
    gnss_transform.transform.translation.x = x
    gnss_transform.transform.translation.y = y
    gnss_transform.transform.translation.z = z
    gnss_transform.transform.rotation.x = q[0]
    gnss_transform.transform.rotation.y = q[1]
    gnss_transform.transform.rotation.z = q[2]
    gnss_transform.transform.rotation.w = q[3]
    return gnss_transform


tf_msg = TFMessage()
neu_to_imu = set_transform(msg.header, "map", "imu", local_x, local_y, 0, q)
tf_msg.transforms.append(neu_to_imu)

output_bag.write("/tf", tf_msg, msg.header.stamp)
```

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        将当前文件夹下的 bag 文件合并为一个 bag 文件
    </summary>

```bash
# 建议先 merge 完再 compress，因为在合并时，也是需要先隐式 decompress 的

$ pip3 install rosbag-merge
# 合并当前目录下的 bag 文件
$ rosbag-merge --write_bag --outbag_name <包名>
```

</details>

<details>
    <summary>:wrench: <b>用例 6：</b>
        解决 ROS1 中 rosbag 开头丢包的问题
    </summary>
[由于播包开始时订阅器和发布器尚未构成通路，则 rosbag 发的数据或会丢失](https://robotics.stackexchange.com/questions/83136/data-loss-between-publisher-and-subscriber)；可以在 rosbag play 时使用 `--wait-for-subscribers` 选项，等订阅器和发布器初始化后才发布数据。

</details>

## Tools

|                  repository                  |    description    |
|:--------------------------------------------:|:-----------------:|
| [rqt_bag](http://wiki.ros.org/rqt_bag)（ROS1） | rosbag GUI viewer |

## Reference

| 摘要   | ROS2                                                         | ROS1                                |
|------|--------------------------------------------------------------|-------------------------------------|
| 官方用例 | https://github.com/ros2/rosbag2/tree/rolling/rosbag2_py/test | http://wiki.ros.org/rosbag/Cookbook |