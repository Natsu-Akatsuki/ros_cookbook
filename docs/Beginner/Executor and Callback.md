# Executor and Callback

- 根据[官方设计说明](http://design.ros2.org/articles/changes.html)，ROS1 和 ROS2 的 C++实现中均有单线程模型和多线程模型，而 ROS2 能自行设计颗粒度更细的线程模型

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        使用多线程执行回调函数
    </summary>

<!-- tabs:start -->

#### **ROS1**

- （`multi-single`）**多线程**执行**多个**不同的回调函数（比如同时执行`A`，`B`两个回调函数）

```cpp
// 方案一：
ros::MultiThreadedSpinner spinner(4); // Use 4 threads
spinner.spin(); // spin() will not return until the node has been shutdown

// 方案二：（颗粒度更细，需要自行控制开关 start/stop）
ros::AsyncSpinner spinner(4); // Use 4 threads
spinner.start();
ros::waitForShutdown();
```

- （`multi-single`）**多线程**执行**多个**相同的回调函数（比如同时执行两个`A`回调函数）

```cpp
void ChatterCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
  std::this_thread::sleep_for(0.02s);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
    // 默认情况下是对相同的回调函数上互斥锁而不能同时执行
  ros::SubscribeOptions ops;
  ops.template init<std_msgs::String>("chatter", 1, ChatterCallback);
  ops.allow_concurrent_callbacks = true;
  ros::Subscriber sub1 = n.subscribe(ops);
    
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
```

#### **ROS2**

（[Callback Group](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html#about-callbacks)）订阅器的回调函数若没有指定 group 的从属，则使用默认的 callback group（`Mutually Exclusive Callback Group`）

```cpp
// 创建 option->配置 callback group
rclcpp::SubscriptionOptions options;
options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
my_subscription = this->create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(), callback, options);
```

<!-- tabs:end -->

</details>

## FAQ

暂未找到权威的资料，部分回答为自己的推理

<details>
    <summary>:question: <b>问题 1：</b>
        queue size 的含义？
    </summary>

`queue_size` 对应的是`publisher queue size`（待发布数据的缓存队列）和 `subscriber queue size`（待处理的接收数据的缓存队列）

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        <a href="https://get-help.robotigniteacademy.com/t/what-is-rospy-spin-ros-spin-ros-spinonce-and-what-are-they-for/58">rospy 和 roscpp 中 spin 的区别？</a>
    </summary>

`rospy.spin()` 只是起阻塞作用（自旋锁/忙等），防止主进程结束；而`roscpp`中的 `spin` 和`spinonce`一方面起阻塞作用，另一方面用于触发回调函数的执行

</details>

<details>
    <summary>:question: <b>问题 3：</b>
        发布器的数据处理逻辑？（猜测）
    </summary>

调用`pubish()`时，发布器线程（`publisher thread`）会将相关的原始数据放到对应的发布器队列中（`publisher queue` ），如果队列已满则丢弃旧的数据；自旋线程（`spinner thread`）再根据发布器队列中对应的数据，对数据进行**序列化**和进行**发布**

</details>

<details>
    <summary>:question: <b>问题 4：</b>
        订阅器的数据处理逻辑？（猜测）
    </summary>

接收器线程（`receiver thread`）将接收到的**序列化**数据放到各自的订阅器队列中（`subscriber queue` ）中，如果队列已满则丢弃旧的数据；自旋线程（`spinner thread` ）根据订阅器队列中对应的数据，对数据进行反序列化和调用相关的回调函数

</details>

<details>
    <summary>:question: <b>问题 5：</b>
        如何只处理最新的数据？
    </summary>

在 ros 中，可能会遇到一些很耗时的操作，比如点云配准，图像特征提取。这样的话，回调函数的处理时间就会变得很长。如果发布端发布数据的频率高于订阅端处理的速度，同时订阅端没有限制地处理所有的数据的话，就会使订阅端一直处理较旧的数据。最终的数据和数据的处理之间的时延将会很高。希望处理最新的数据的话，就需要将发布器和订阅器的队列长度设置为 1。如下为图像处理时队列长度不为 1
的效果图（左为输出效果，右为输入图像，可看出有较大的时延）（实测：`inference`时间和`ros image`数据传输耗时为 ms 级别）

![img](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/latency.gif)
</details>

<details>
    <summary>:question: <b>问题 6：</b>
        rospy 回调函数的多线程处理机制
    </summary>

``rospy`` 中处理回调函数时会派生出一个新的线程去执行（线程名与主题名相同）；如果有 n 个回调函数（处理的是不同的 topic）则会派生出 n 个线程；如果有回调函数处理的是相同 topic 则共用一个线程

![img](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/rospy-cb-multithread.png)

</details>

<details>
    <summary>:question: <b>问题 7：</b>
        各种队列
    </summary>

|        类型        |                                                                                                                                                                       作用                                                                                                                                                                       |
|:----------------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| subscriber queue | When new messages arrive, they are stored in a queue until ROS gets a chance to execute your callback function. This parameter establishes a maximum number of messages that ROS will store in that queue at one time. If new messages arrive when the queue is full, the oldest unprocessed messages will be dropped to make room. （估计暂未反序列化） |
| publisher queue  |                                                                               the publisher queue is another queue like callback queue, but the queue is for queuing published message which is filled every time ``publish()`` function is called. （估计暂未进行序列化）                                                                                |
|  callback queue  |                                                                                                                                                                       —                                                                                                                                                                        |

</details>

## Reference

- [精讲多线程之 MultiThreadedSpinner](https://zhuanlan.zhihu.com/p/375418691)
- [ROS Spinning, Threading, Queuing](https://levelup.gitconnected.com/ros-spinning-threading-queuing-aac9c0a793f)
- ROSCON: [concurrency (2019)](https://roscon.ros.org/2019/talks/roscon2019_concurrency.pdf)
