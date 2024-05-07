# Node and Component

ROS1 中既可以将调用了 ROS 客户端库的源程序封装为 [node](https://wiki.ros.org/Nodes) 也可以封装为 [ROS nodelet](https://wiki.ros.org/nodelet)；ROS 1 nodes 将会编译成可执行文件；而后者则被编译为动态库，在运行期时被 `container process` 所加载（load）；ROS2 中的 `nodelet` 称为 `component`，其一般为 `rclcpp::Node`的子类，ROS2 推荐将代码组织成 `component`

相比于 ROS1，写一个 component/nodelet 和 node 所调用的 API 区别不大，改起来更加方便，具体参考 [Here](http://design.ros2.org/articles/changes.html)

ROS1 的节点通信是基于 TCP/UDP 的进程通信。当节点间传输的数据体量较大，通信（比如要反序列和序列化）的开销将比较大。因此若希望减少节点间通讯的开销来提高实时性，这就需要用到`nodelet`技术。具体来说，比如跑一些点云的预处理模块，涉及到点云直通滤波节点，去地面节点，然后希望将这些算法都放到同一个进程中进行计算而通过零拷贝来减少通信开销。

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用 API
    </summary>

```bash
# 显示当前工作空间中的 nodelets（不一定是正在运行的 nodelet）
(ROS1) $ rosrun nodelet declared_nodelets

# 显示 XML 文件
(ROS1)$ rospack plugins --attrib=plugin nodelet

# 启动节点管理器并重命名
(ROS1)$ rosrun nodelet nodelet manager __name:=nodelet_manager

# 将一个节点导入到节点管理器 [type pkg/type(包名/xml文件中的class name)]
(ROS1)$ rosrun nodelet nodelet load nodelet_tutorial_math/Plus nodelet_manager
```

```bash
# 查看当前工作空间现有的component
$ ros2 component types

# 运行一个component container进程，用于管理component
$ ros2 run rclcpp_components component_container
# 查看已启动的container
$ ros2 component list
# /ComponentManager

# Load a component into a container node
# container_node_name package_name plugin_name
$ ros2 component load /ComponentManager composition composition::Talker
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20210810223516109.png)

</details>


<details>
    <summary>:wrench: <b>用例 2：</b>
        启动 Nodelet
    </summary>

<!-- tabs:start -->

#### **ROS1**

```xml
<!--都需要启动nodelet包的nodelet可执行文件，不过相应的启动参数不一样-->
<node pkg="nodelet" type="nodelet" name="euclidean_cluster_manager" args="manager" output="screen"/>

<node pkg="nodelet" type="nodelet" name="$(anon voxel_grid_filter)" args="load pcl/VoxelGrid euclidean_cluster_manager" output="screen">
</node>

<node pkg="nodelet" type="nodelet" name="$(anon euclidean_cluster)" args="load euclidean_cluster/voxel_grid_based_euclidean_cluster_nodelet euclidean_cluster_manager" output="screen">
</node>

        <!--standalone nodelet，不需要加载到nodelet manager，相关于启动一个普通node-->
<node pkg="nodelet" type="nodelet" name="Plus3" args="standalone nodelet_tutorial_math/Plus">
</node>
```

#### **ROS2**

- 可以通过命令行启动`container`进程后，在使用服务时导入`componet`
- 可以通过编写可执行文件来调用[component](https://github.com/ros2/demos/blob/foxy/composition/src/manual_composition.cpp)
- 编写一个`launch`文档，来调用`component`具体可参考[here](https://github.com/ros2/demos/blob/humble/composition/launch/composition_demo.launch.py)

```xml
import launch
        from launch_ros.actions import ComposableNodeContainer
        from launch_ros.descriptions import ComposableNode


        def generate_launch_description():
        """Generate launch description with multiple components."""
        container = ComposableNodeContainer(
        name='进程名(component container name)',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
        ComposableNode(
        package='包名',
        plugin='包名::类名',
        name='线程名
<component>'),
    ComposableNode(
    package='包名',
    plugin='包名::类名',
    name='线程名
    <component>')
        ],
        output='screen',
        )

        return launch.LaunchDescription([container])
```

<!-- tabs:end -->

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        Nodelet 样例（源程序）
    </summary>

<!-- tabs:start -->

#### **ROS1**

```cpp
// from tier4@euclidean_cluster_node.cpp
#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "euclidean_cluster_node");
  ros::NodeHandle private_nh("~");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "euclidean_cluster/euclidean_cluster_nodelet", remap, nargv);
  ros::spin();
  return 0;
}
```

<!-- tabs:end -->


</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
        Node 样例（源程序）
    </summary>


<!-- tabs:start -->

#### **OOP C++(ROS1)**

```cpp
class Node {
public:
  explicit Node(ros::NodeHandle *pnh);

private:
  /* ros system */
  ros::NodeHandle *pnh_;
  ros::Publisher pub_velocity_;
  ros::Publisher pub_odom_;

  ros::Subscriber sub_gear_cmd_;
  ros::Subscriber sub_manual_gear_cmd_;
  
  ros::Timer on_timer_; // 定时器
    
  /* tf */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* received & published topics */
  VelocityReport current_velocity_;
  Odometry current_odometry_;
  SteeringReport current_steer_;

  /* frame_id */
  std::string simulated_frame_id_;
  std::string origin_frame_id_;
  
  void onEngage(const Engage::ConstPtr msg);
  void onTimer(const ros::TimerEvent &e);
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "node_name");
  ros::NodeHandle pnh("~");
  Node node(&pnh);
  ros::spin();
  return 0;
}
```

#### **Naive C++(ROS2)**

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared(<节点名>);
    
    // Run the executor.
    rclcpp::spin(node);
    
    // Shutdown and exit.
    return 0;  
}
```

`spin()`部分可等价于：

```cpp
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

#### **Naive Python (ROS2)**

```python
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__( < 节点名 >)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

<!-- tabs:end -->

- 继承节点类

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node(<节点名>) {}
private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

- Client Library

|      ROS1       |        ROS2        |
|:---------------:|:------------------:|
|    ros::ok()    |    rclcpp::ok()    |
| ros::shutdown() | rclcpp::shutdown() |

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        分析 ROS1 roslaunch 启动的进程
    </summary>

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220624114324096.png ':size=900 roslaunch 为父进程，其启动的节点为子进程' )

</details>

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        rospy.init_node() 为什么在主线程才能调用？
    </summary>

因为`rospy.init_node()`时会导入（ `register` ）信号回调函数（`signal handlers` ），而 Python 中引入信号回调函数需要在主线程中完成（Python 特性）；不引入信号回调函数则可以在非主线程中调用`rospy.init_node()`

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20210909214309037.png)

```python
import rospy
import threading


class Node(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        rospy.init_node('node')
        rospy.spin()


if __name__ == '__main__':
    node = Node()
    node.start()
    node.join()

# >>> ValueError: signal only works in main thread
```

官方信号回调函数实现：

```python
# File "/opt/ros/noetic/lib/python3/dist-packages/rospy/core.py", line 623, in register_signals #687
def register_signals():
    """
    register system signal handlers for SIGTERM and SIGINT
    """
    _signalChain[signal.SIGTERM] = signal.signal(signal.SIGTERM, _ros_signal)
    _signalChain[signal.SIGINT] = signal.signal(signal.SIGINT, _ros_signal)
```

</details>

## Reference

| 摘要   | ROS2                                                                                    | ROS1                                                                      |
|------|-----------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| 官方教程 | https://docs.ros.org/en/humble/Concepts/About-Executors.html                            | http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 |
| CLI  | https://docs.ros.org/en/iron/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html | http://wiki.ros.org/nodelet#Helper_tools                                  |

| 摘要   | 链接                                                                                                                                             |
|------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 示例代码 | https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.cpp<br />https://github.com/cryborg21/sample_nodelet |