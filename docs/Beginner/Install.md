# Install

## ROS1

具体参考 [Here](http://wiki.ros.org/noetic/Installation)

## ROS2

适用于 Humble (Ubuntu 22.04)，具体参考 [Here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)；
若要安装尝鲜版可参考 [Here](https://docs.ros.org/en/humble/Installation/Testing.html)

<!-- tabs:start -->

#### **:star:apt**

步骤 1：安装

```bash
$ sudo apt update && sudo apt install curl gnupg lsb-release
$ sudo apt upgrade
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# URL 可改为：https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

$ sudo apt update
$ sudo apt install ros-humble-desktop

# 添加环境变量到~/.bashrc
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

步骤 2：验证是否安装成功。

```bash
$ ros2 run demo_nodes_cpp talker
```

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/L3PpIaqrMi15ctrj.png ':size=850')

#### **source**

缺点是不能使用从 apt 安装的 ROS 包

```bash
# >>> 安装相关开发工具 >>>
$ sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-docstrings \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# >>> 获取源代码 >>>
$ mkdir -p ~/ros2_humble/src
$ cd ~/ros2_humble
$ wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
$ vcs import src < ros2.repos

# >>> 安装相关依赖 >>>
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# >>> 编译工程 >>>
$ cd ~/ros2_humble/
$ colcon build --symlink-install

# 添加环境变量
$ echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
```

#### **Conda**

具体参考 [Here](https://github.com/RoboStack/ros-noetic)，同源码编译一样，一旦用到其他 ROS 依赖库就需要源码编译，不能用 apt 包。暂没感受到什么优点。

<!-- tabs:end -->

## Package

### rosdep

`rosdep`相关于`apt`的拓展包，用于下载依赖包。

```bash
# >>> 方案一 >>>
# 初始化安装源
$ sudo rosdep init
# 更新源
$ rosdep update

# >>> 方案二 >>>
# 手动模拟 rosdep init
$ sudo mkdir -p /etc/ros/rosdep/sources.list.d/
$ sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
$ echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
$ source ~/.bashrc
$ rosdep update

$ rosdep install --from-paths src --ignore-src -r -y
# -i, --ignore-packages-from-source, --ignore-src：若ROS_PACKAGE_PATH有这个包/当前工作空间有该依赖包 ，则不rosdep安装
# --from-paths：搜索路径
# -r：Continue installing despite errors.
# -y：Tell the package manager to default to y
```

### rosws

一般配合.rosinstall 文件使用，具体参考 [Here](https://github.com/oceansystemslab/rosinstall)

```bash
$ rosws update
```