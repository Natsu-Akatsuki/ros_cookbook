# Header and Module

## cv_bridge

### Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        调用 cv_bridge 拓展库时出现 ImportError: dynamic module does not define module export function (PyInit_xxx)
    </summary>

使用源码编译 cv_bridge 库。这种需求一般出现在 Melodic 等低版本的 ROS 系统中，在该环境下一些 apt 下载的库如 cv_bridge，qt_gui_core，它们的拓展库可在 Python2.7 顺利执行，但在 Python3 下执行时（如在 Conda 虚拟环境下启动）会报错。因为编译拓展库时和运行时的 Python 版本不一致

1）步骤 1：下载源码

```bash
$ git clone -b melodic https://github.com/ros-perception/vision_opencv.git vision_opencv/src && cd vision_opencv
```

2）步骤 2：参考 [Here](https://gitlab.kitware.com/cmake/cmake/-/merge_requests/201/diffs?commit_id=b8b227e90917f9d3ba579c7204d196c7b7a2a46d) 对 /usr/share/cmake-3.10/Modules/FindBoost.cmake 进行修改，否则会出现如下报错

```plain
CMake Warning at /usr/share/cmake-3.10/Modules/FindBoost.cmake:1626 (message):
  No header defined for python3; skipping header check
Call Stack (most recent call first):
  CMakeLists.txt:28 (find_package)
```

> [!note]
>
> Noetic 以上版本则不需要进行修改，it seems fixed since the 3.11 version of cmake, but not in the release delivered with bionic (3.10)

3）步骤 3：编译

```bash
$ env_path='~/.conda/envs/pcdet' \
$ catkin config -DPYTHON_EXECUTABLE=${env_path}/bin/python3.7 \
-DPYTHON_INCLUDE_DIR=${env_path}/include/python3.7m \
-DPYTHON_LIBRARY=${env_path}/lib/libpython3.7m.so
```

4）步骤 4：测试

```bash
$ source devel/setup.bash
$ python -c "from cv_bridge.boost.cv_bridge_boost import getCvType"
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        将 8UC2 编码的图片转换为 OpenCV 类型的图片，并进行 min_max 归一化，转换为单通道灰度图
    </summary>

```cpp
cv::Mat image;
if (image_msg->encoding == "8UC2")
{
    sensor_msgs::Image img;
    img.header = image_msg->header;
    img.height = image_msg->height;
    img.width = image_msg->width;
    img.is_bigendian = image_msg->is_bigendian;
    img.step = image_msg->step;
    img.data = image_msg->data;
    // 查阅源代码，toCVCopy 等函数只能处理 mono, rgb 等类型的编码，则需进行转换
    img.encoding = "mono16";   
    auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    cv::normalize(cv_ptr->image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
}
```

> [!note]
>
> [8UC1 和 mono8 有什么区别？](https://github.com/ros-perception/vision_opencv/pull/180#issuecomment-314384785) \
> mono8 额外地表明该图片是灰度图（包含颜色信息）

</details>

### Reference

- [介绍各种编码模式](http://library.isr.ist.utl.pt/docs/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html)

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用头文件
    </summary>
`ROS2`的内置头文件采用`.hpp`后缀，追加`msg`修饰，命名方式从`大驼峰`改成`下划线`方式

|                                   ROS1                                   |             ROS2             |
|:------------------------------------------------------------------------:|:----------------------------:|
|                           #include "ros/ros.h"                           | #include <rclcpp/rclcpp.hpp> |
| #include <tf2_ros/buffer.h><br/>\#include <tf2_ros/transform_listener.h> |              同左              |
|                     #include <boost/shared_ptr.hpp>                      |      #include \<memory>      |

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        常用模块
    </summary>

|                                                        ROS1                                                         |     ROS2     |
|:-------------------------------------------------------------------------------------------------------------------:|:------------:|
|                                                    import rospy                                                     | import rclpy |
|                                           from cv_bridge import CvBridge                                            |      同左      |
| from sensor_msgs.msg import Image<br />from sensor_msgs.msg import PointCloud2<br />from std_msgs.msg import Header |      同左      |

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        <a href="https://blog.csdn.net/xiangxianghehe/article/details/78660275">module 'em' has no attribute 'Interpreter'</a>
    </summary>

```bash
$ pip uninstall em
$ pip install empy
```

</details>

## Roadmap

- [ ] 追加图片压缩的代码块