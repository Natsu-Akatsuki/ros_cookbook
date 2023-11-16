# Signal

## FAQ

<details>
    <summary>:question: <b>问题 1：</b>
        <a href="https://answers.ros.org/question/10252/what-escalating-to-sigterm-mean/">escalating to SIGTERM 是什么问题？</a>
    </summary>

ROS 尝试使用 `SIGINT` 来释放进程，但如果一段时间后，进程仍未结束，则使用 `SIGTERM` 信号来结束进程。

</details>

<details>
    <summary>:question: <b>问题 2：</b>
        为什么进程无法使用 SIGINIT 正常结束？
    </summary>

进程中的线程可能挂起，而无法接收到 `SIGINIT` 信号。如何使用日志进行调试，判断线程是否的确在挂起？

```cpp
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex mtx;
// 该条件变量不需要赋值
std::condition_variable cv;
bool ready = false;

// waitForEvent 函数等待一个条件变量。它在开始和结束等待时记录日志信息。
void waitForEvent() {
  std::unique_lock<std::mutex> lock(mtx);
  ROS_INFO("Waiting for the event to be set...");    
  // 阻塞当前线程，在这种情况下线程会调用 wait() 进行挂起；
  // 当被 notify_one() 唤醒后，如果 ready 为 True 则停止阻塞
  cv.wait(lock, [] { return ready; });
  // 如果使用 CTRL + C 后，该日志没有打印，则证明的确是挂起了
  ROS_INFO("Finished waiting. Event was set.");
}

// 设置事件的函数
void setEvent() {
  {
    // 给互斥锁上锁，其作用域下的变量都在临界区
    std::lock_guard<std::mutex> lock(mtx);
    ROS_INFO("Setting the event...");
    ready = true;
  }
  // 条件变量 std::condition_variable 的一个成员函数 notify_one() 通知一个等待的线程
  cv.notify_one();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_threaded_node");
  ros::NodeHandle nh;

  ROS_INFO("Starting threads...");

  std::thread worker(waitForEvent);
  std::thread setter(setEvent);

  worker.join();
  setter.join();

  ROS_INFO("All threads finished.");

  return 0;
}
```

</details>

<details>
    <summary>:question: <b>问题 3：</b>
        如何修改 Timeout 触发 SIGTERM 的时间        
    </summary>

具体可修改如下配置：

```bash
# DEFAULT_TIMEOUT_SIGINT  = 15.0 #seconds
# DEFAULT_TIMEOUT_SIGTERM = 2.0 #seconds

# Melodic
$ sudo vim /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
# Noetic
$ sudo vim /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py
```

</details>

<details>
    <summary>:question: <b>问题 4：</b>
        一个 ROS 进程怎样才算正常的退出
    </summary>

使用 CTRL + C 的退出：触发 `SIGINIT` 信号，`ros::spin()` 或 `ros::waitForShutdown()` 接收到 `SIGTINT` 信号后，则停止阻塞以执行后面的语句，从而顺利地执行 main() 函数

```cpp
int main() {
	
    // ...
    ros::waitForShutdown();
    // ros::spin();

    printf("\033[1;32m[Signal] Receive signal to shutdown\033[0m \n");
    return 0;
}
```

</details>

<details>
    <summary>:question: <b>问题 5：</b>
        分析 DLIO 进程无法正确退出的原因？
    </summary>

DLIO 会在 `cv_imu_stamp.wait` 中挂起，因此会屏蔽 SIGINT 信号，可使用上 `ros::ok()` + 使用 `SIGINT` 中断函数解决

```cpp
if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
  // Wait for the latest IMU data
  std::unique_lock<decltype(mtx_imu)> lock(mtx_imu);
  cv_imu_stamp.wait(lock, [this, &end_time] {
    // ros::ok() is used to prevent the thread from waiting and not exiting
    bool status = this->imu_buffer.front().stamp >= end_time or !ros::ok();
    return status; }
    );
}

void signalINTHandler(int signum) {
  printf("\033[1;32m[Signal] Receive SIGINT signal to shutdown\033[0m \n");
  ros::shutdown();
  cv_imu_stamp.notify_one();
}

int main() {
  // ...
  signal(SIGINT, signalINTHandler);
  // ...
}

```

</details>
