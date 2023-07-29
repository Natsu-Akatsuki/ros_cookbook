# Marker

- 一般会单独构建一个节点来处理图形的渲染，而不是像[here](https://github.com/Livox-SDK/livox_detection/blob/master/livox_rosdetection.py)直接将可视化和算法放在同一节点
- marker也只是个ros topic

## 用例

### marker_example

marker_ex.py：使用marker显示文本和物体形状

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220425155918574.png" alt="image-20220425155918574" style="zoom:50%;" />

### distance_marker

参考livox的可视化效果和autoware的程序，在rviz上显示带距离信息的极坐标网格图

![image-20220425192508852](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220425192508852.png)

步骤一：编译polar grid，然后source环境

```bash
$ cd polar_grid
$ catkin build
$ source devel/setup.bash
$ rviz
```

步骤二：发布distance距离信息

```bash
$ python3 distance_marker.py
```

步骤三：rviz中添加相关插件和订阅信息

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220425193856669.png" alt="image-20220425193856669" style="zoom:50%;" />

## 参考资料

- [marker ros wiki](http://wiki.ros.org/rviz/DisplayTypes/Marker)
- Autoware Polar Grid
- Livox Detection