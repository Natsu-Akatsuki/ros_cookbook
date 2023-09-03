# Deployment

## Usage

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://docs.ros.org/en/iron/How-To-Guides/Releasing/Releasing-a-Package.html#">在 ROS2 中如何将一个包发布到 ROS2 的包仓库中？</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        <a href="https://answers.ros.org/question/226581/deploying-a-catkin-package/">如何将一个包安装到系统中</a>
    </summary>
</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
      使用元包（meta-package）来分发和管理一系列的包，以方便同时安装一系列的包（即可以在 apt 安装时用元包名来替换一连串的包名）
    </summary>

```bash
$ packages_name="ros-${ROS_DISTRO}-velodyne-driver ros-${ROS_DISTRO}-velodyne-laserscan ros-${ROS_DISTRO}-velodyne-msgs ros-${ROS_DISTRO}-velodyne-pointcloud"
$ meta_packages_name="ros-${ROS_DISTRO}-velodyne"
# 安装元包
$ sudo apt install meta_packages_name
# 等价于安装元包管理的一系列包
$ sudo apt install packages_name

# 创建 meta 包
$ catkin_create_pkg <MY_META_PACKAGE> --meta
```

具体使用方法可参考 [Here](http://wiki.ros.org/catkin/package.xml#Metapackages)，只是需要添加一些代码块到 `CMakeLists.txt` 和 `package.xml` 文件

</details>


