# Transform

## Listen

<details>
    <summary>:wrench: <b>用例 1：</b>
        查看 TF 树（CLI）
    </summary>

```bash
# >>> 安装依赖 >>>
(ROS1) $ sudo apt install ros-${ROS_DISTRO}-tf2-tools
(ROS2) $ sudo apt install ros-${ROS_DISTRO}-rqt-tf-tree

# >>> 查看TF树（动态查看）>>>
(ROS1) $ rosrun rqt_tf_tree rqt_tf_tree
(ROS2) $ ros2 run rqt_tf_tree rqt_tf_tree

# >>> 生成相关的 PFD 文件（静态查看）>>>
(ROS1) $ rosrun tf2_tools view_frames.py
(ROS2) $ ros2 run tf2_tools view_frames
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        监听特定的 TF（基坐标变换）（CLI）
    </summary>

```bash
# tf_echo <source_frame> <target_frame>  # 监听source->target的坐标变换（基坐标变换）
(ROS1) $ rosrun tf tf_echo /lidar /camera

# At time 0.000
# - Translation: [0.000, 0.000, 1.000]
# - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
#            in RPY (radian) [-1.571, -0.000, -1.571]
#            in RPY (degree) [-90.000, -0.000, -90.000]

(ROS2) $ ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

# - Translation: [-0.030, -0.030, 0.020]
# - Rotation: in Quaternion [-0.477, 0.476, -0.532, 0.512]
# - Rotation: in RPY (radian) [-1.480, -0.020, -1.590]
# - Rotation: in RPY (degree) [-84.798, -1.146, -91.100]

# 对应的launch文档如下：
# <node pkg="tf2_ros" exec="static_transform_publisher" name="lidar_2_camera" args="-0.03 -0.03  0.02 -1.59 -0.02 -1.48 lidar camera" />
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220312101457181.png ':size=700')


</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        监听特定的 TF（源程序）
    </summary>

<!-- tabs:start -->

#### **ROS1(C++)**

```cpp
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer); // 不能赋值，只能初始化

ros::Rate rate(10.0);
while (node.ok()){
	geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform(toFrameRel, fromFrameRel, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) 
    
    pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
}
```

#### **ROS2(C++)**

> [!note]
>
> `shared_ptr` 封装的 `tf2_ros::TransformListener` 可以赋值

```cpp
// ROS2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; // (this->get_clock());
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // 传引用

// toFrameRel 系->fromFrameRel 系的坐标系变换 或者 fromFrameRel 系->toFrameRel 的坐标变换
transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

try {
    t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
} catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
}
```

#### **ROS2(Python)**

```python
def transfrom_box3d_frame(self, box3d_src, target_frame, src_frame, ):
    """
    将 A 系 (src_frame) 的 box3d 转换到 B 系 (target_frame) 下
    """
    box3d_target = box3d_src.copy()
    xyz = box3d_target[:, :3]
    xyz = np.hstack((xyz, np.ones((xyz.shape[0], 1))))

    try:
        t = self.tf_buffer.lookup_transform(
            target_frame,
            src_frame,
            rclpy.time.Time())  # 只要最新的数据
        # 获得的是基坐标变换
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        rx = t.transform.rotation.x
        ry = t.transform.rotation.y
        rz = t.transform.rotation.z
        rw = t.transform.rotation.w
        r = Rotation.from_quat([rx, ry, rz, rw])
        r = r.as_euler(seq="ZYX", degrees=False)  # 其 TF 变换是 xyz->ypr
        extri_mat = ros_xyzypr_to_tf_mat([x, y, z, r[0], r[1], r[2]], degrees=False, is_basis_change=True)

    except TransformException as ex:
        self.get_logger().info(
            f'Could not transform {src_frame} to {target_frame}: {ex}')
        return

    xyz = np.dot(xyz, extri_mat.T)[:, :3]
    box3d_target[:, :3] = xyz[:, :3]
    return box3d_src
```

<!-- tabs:end -->


</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        监听 TF 
    </summary>

```python
import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf2_echo_rospy')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        # API 对应 target_frame, source_frame
        # 得到激光雷达系到相机系的基变换
        # 等价于将相机下的点转换到激光雷达系下的描述的坐标变换
        trans = tfBuffer.lookup_transform("lidar", 'camera', rospy.Time())
        pass
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue
    rate.sleep()

```

此处 API 所描述的`target_frame`和`source_frame`的描述的是坐标变换的概念，而不是坐标系变换

|     坐标系      |                      描述                       |
|:------------:|:---------------------------------------------:|
| target_frame | The frame to which data should be transformed |
| source_frame |      The frame where the data originated      |

</details>

## Publish

<details>
    <summary>:wrench: <b>用例 1：</b>
        发布动态 TF（Python）
    </summary>

<!-- tabs:start -->

#### **ROS1(Python)**

```python
#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_publisher')
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "lidar"
        t.child_frame_id = "camera"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0
        t.transform.rotation.x = -0.50
        t.transform.rotation.y = 0.50
        t.transform.rotation.z = -0.50
        t.transform.rotation.w = 0.50

        br.sendTransform(t)

    rospy.spin()
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        发布从激光雷达系到相机系的坐标系变换（基变换） TF（Launch）
    </summary>

> [!note]
>
> 机器人操作系统 ROS 的 从左到右的 TF 描述的是基变换，而不是坐标变换

相关参数依次对应：x y z yaw pitch roll 父坐标系（frame_id）子坐标系（child_frame_id），使用的是内旋坐标系

<!-- tabs:start -->

#### **ROS2(XML)**

```xml

<launch>
    <!-- 方案一 -->
    <node pkg="tf2_ros" exec="static_transform_publisher" name="lidar_to_camera" args="0, 0, 1, -1.570795, 0, -1.570795, lidar camera"/> <!-- lidar后面不能加逗号 -->
    <!-- 方案二 -->
    <node pkg="tf2_ros" exec="static_transform_publisher" name="lidar_2_camera" args="--x 0 --y 0 --z 1 --yaw -1.570795 --pitch -1.570795 --roll -1.570795 --frame-id lidar --child-frame-id camera"/>
</launch>
```

#### **ROS2(Python)**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(package="tf2_ros",
                executable="static_transform_publisher",
                arguments=['--x', '1.0',
                           '--frame-id', 'lidar',
                           '--child-frame-id', 'camera'])

    ld.add_action(node)

    return ld
```

#### **ROS1(XML)**

```xml

<launch>
    <!-- static_transform_publisher  -->
    <!-- ZYX: 使用的是内旋坐标系 -->
    <!-- 2：有两种解读，一种是go to，一种是relative to， 此处是go to-->
    <node pkg="tf2_ros" type="static_transform_publisher" name="lidar_to_camera" args="0, 0, 1, -1.570795, 0, -1.570795 lidar camera "/>

</launch>
```

<!-- tabs:end -->

> [!note] 如何快速计算基变换矩阵？
>
> 将旧基在新基下的列向量表征矩阵（基使用一维行向量）

</details>

<details>
    <summary>:wrench:
        <b>用例 3：</b>
        发布从激光雷达系到相机系的坐标系变换（基变换） TF（CLI）
    </summary>

```bash
(ROS1) $ rosrun tf2_ros static_transform_publisher 0 0 1 -1.570795 0 -1.570795 lidar camera
# 方案 1
(ROS2) $ ros2 run tf2_ros static_transform_publisher 0 0 1 -1.570795 0 -1.570795 lidar camera
# 方案 2
(ROS2) $ ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1 --yaw -1.570795 --pitch -1.570795 --roll -1.570795 --frame-id lidar --child-frame-id camera
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        发布静态 TF（C++）
    </summary>

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

tf2_ros::StaticTransformBroadcaster static_broadcaster_ = tf2_ros::StaticTransformBroadcaster();

geometry_msgs::TransformStamped static_transformStamped;
static_transformStamped.header.stamp = ros::Time::now();
static_transformStamped.header.frame_id = map_frame_;
static_transformStamped.child_frame_id = viewer_frame_;
static_transformStamped.transform.translation.x = coordinate_x;
static_transformStamped.transform.translation.y = coordinate_y;
static_transformStamped.transform.translation.z = coordinate_z;
tf2::Quaternion quat;
quat.setRPY(0, 0, 0);
static_transformStamped.transform.rotation.x = quat.x();
static_transformStamped.transform.rotation.y = quat.y();
static_transformStamped.transform.rotation.z = quat.z();
static_transformStamped.transform.rotation.w = quat.w();

static_broadcaster_.sendTransform(static_transformStamped);    
```

</details>	

## Others

<details>
    <summary>:wrench: <b>用例 1：</b>
        将欧拉角转换为四元数（Python）
    </summary>

<!-- tabs:start -->

#### **ROS1**

只有 ROS1 的 TF 库才有这部分转换，[ROS2 中开发者将 transformations 库解耦出来，使 TF 库更加轻量化](https://github.com/ros/geometry2/issues/222)

```python
# 欧拉角转换为四元数
from tf.transformations import quaternion_from_euler

# 平移量和旋转量转换为变换矩阵
import numpy as np
from tf import TransformerROS

mat = TransformerROS().fromTranslationRotation(np.array([0.45, 0, -0.25]), np.array([-0.5, 0.5, -0.5, 0.5]))
with np.printoptions(precision=2, suppress=True):
    print(np.linalg.inv(mat))
```

<!-- tabs:end -->

</details>

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        如何理解 tf_prefix？
    </summary>

[tf_prefix 是 ROS 的参数](https://wiki.ros.org/geometry/CoordinateFrameConventions#geometry.2BAC8-CoordinateFrameConventions.2BAC8-Naming.tf_prefix)（parameter），使用 [tf_prefix](http://wiki.ros.org/geometry/CoordinateFrameConventions) 后，会修饰 frame_name，得到`/[tf_prefix/]frame_name`

参考资料：

1. https://wiki.ros.org/tf/Theory
2. https://wiki.ros.org/tf2
3. https://wiki.ros.org/tf2/Migration: Anyone using the tf::Transformer interface will have the "/" stripped from the frame_id before it is passed to tf2 under the hood.

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        [ WARN] TF_REPEATED_DATA ignoring data with redundant timestamp for frame <...> at time <...> according to authority unknown_publisher
    </summary>

字面意思，同一个 TF（时间戳、坐标系均相同）重复发布了两次，出现这种问题时注意查看发布的 TF 的时间戳

</details>

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://github.com/rkoyama1623/rocky_tf_monitor">找到 frame_id 中含 "/" 的节点</a>
    </summary>

步骤 1：构建测试文件

```xml

<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="lidar1_to_camera1" args="0, 0, 1, -1.570795, 0, -1.570795 lidar1 camera1 "/>
    <node pkg="tf2_ros" type="static_transform_publisher" name="lidar2_to_camera2" args="0, 0, 1, -1.570795, 0, -1.570795 /lidar2 /camera2 "/>
    <node pkg="tf" type="static_transform_publisher" name="lidar3_to_camera3" args="0, 0, 1, -1.570795, 0, -1.570795 lidar3 camera3 100"/>
    <node pkg="tf" type="static_transform_publisher" name="lidar4_to_camera4" args="0, 0, 1, -1.570795, 0, -1.570795 /lidar4 /camera4 100"/>
</launch>
```

步骤 2：执行程序 rocky_tf_monitor.py

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/2024-04-06_15-47.png)

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        使用 rocky_tf_monitor 诊断坐标系树是否错误：一个坐标系是否有多个父坐标系
    </summary>

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20240406181137466.png)

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        Rviz 中 Failed to transform from frame [/A] to frame [A] 或 Failed to transform from frame [/A] to frame [B]
    </summary>

字面意思

情况 1：frame A 和 /A 是不同的坐标系，需移除 frame A 中的前导符号 "/" \
情况 2：已知发布的点云的 frame_id 为 /A，Rviz 中 Fixed frame 为 B，则可视化时需要 /A -> B 的坐标系变换。尽管在程序中提供了 /A -> /B 的坐标系变换，但 TF2 会自动移除第一个前导符 "/"，因此实际上发布的是 A->B 的坐标系变换。可以将点云的 frame_id 改为 A。

</details>

## Reference

| 摘要   | ROS2                                                                                    | ROS1                                   |
|------|-----------------------------------------------------------------------------------------|----------------------------------------|
| 规范   | 无                                                                                       | https://www.ros.org/reps/rep-0103.html |
| 官方教程 | https://docs.ros.org/en/iron/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html | https://wiki.ros.org/tf2/Tutorials     |
| API  | https://docs.ros2.org/latest/api/tf2_ros/                                               | https://docs.ros2.org/latest/api/tf2/  |

- [Blog: Transforms in ROS](https://nu-msr.github.io/me495_site/lecture05_tf.html)
- [ROS TF, Whoever wrote the Python API, F**ked up the concepts](https://www.hepeng.me/ros-tf-whoever-wrote-the-python-tf-api-f-ked-up-the-concept/)

## Coordiniate

| LiDAR to camera                                                                                                       | LiDAR to LOAM                                                                                                                        |
|-----------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| ![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220312094230162.png ':size=150 LiDAR to camera') | ![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20230727222925721.png ':size=100 LiDAR to LOAM（依然是右手系，保证 z 指向前）') |

常用的 TF：

|                                                          场景                                                          | TF 变换（角度） |                                 基变换矩阵                                  |                          备注                          |
|:--------------------------------------------------------------------------------------------------------------------:|:---------:|:----------------------------------------------------------------------:|:----------------------------------------------------:|
|                                                   LiDAR 系到相机系的基体变换                                                   | -90，0，-90 | $$\begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0\end{bmatrix}$$ | LiDAR_to_camera（to-> go to 动态，LiDAR（父/根系），camera（子系） |
| [LiDAR 系到 LOAM  系的基体变换](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/launch/run.launch) |  90，0，90  |                                                                        |      LeGO-LOAM 的 camera 指的是 LOAM 系（x 左，y 上，z 前）      |
| [LOAM 系到 LiDAR 系的基体变换](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/launch/run.launch)  | -90，-90，0 |                                                                        |            LeGO-LOAM 的 camera 指的是 LOAM 系             |
|                                                    相机系到激光雷达系的基体变换                                                    |           | $$\begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & -1 \\ 1 & 0 & 0\end{bmatrix}$$ |                          —                           |

