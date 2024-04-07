# Deployment

## Experience

因为将多个 ROS 功能包打包成 debian 包比较麻烦，所以在不考虑复用性的情况下，倾向于将多个包集成为一个功能包来打包（如将消息类型包写在功能包中）

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

<details>
    <summary>:wrench: <b>用例 4：</b>
        将 ROS 包打包成 debian 包，然后安装到 ROS 的系统路径（/opt/ros）（基础版）
    </summary>

步骤 1：安装依赖

```bash
# 安装相关
$ sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python

# 如果以前没有初始化过 rosdep
$ sudo rosdep init

# 更新 rosdep
$ rosdep update
```

步骤 2：修改 CMakeLists.txt

<!-- tabs:start -->

#### **[ROS2](https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html)**

#### **ROS1**

```cmake
# 如果需要打包 C++ 目标文件
install(TARGETS 目标文件名
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 如果需要打包 Python 脚本
catkin_install_python(PROGRAMS scripts/<脚本名>
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}	 # 安装到 /opt/ros/lib/包名/bin
  # DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} # 安装到 /opt/ros/版本号/bin
)
```

<!-- tabs:end -->


步骤 3：打包

```bash
$ cd <含 package.xml 的目录>

# 生成 deb 配置文件
# 等价于：bloom-generate rosdebian --os-name ubuntu --os-version $(lsb_release -cs) --ros-distro $ROS_DISTRO .
$ bloom-generate rosdebian 

# 进行编译和将产物和 deb 配置文件打包，最后在上层目录生成 deb 文件
$ fakeroot debian/rules binary
```

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        <a href="https://answers.ros.org/question/280213/generate-deb-from-dependent-res-package-locally/">如何打包含ROS 包（该 ROS 包含不在 ROS 仓库的依赖）</a>
    </summary>

参考 [4. 功能包相互依赖关系解决](https://zhuanlan.zhihu.com/p/578951132)

</details>

<details>
    <summary>:wrench: <b>用例 6：</b>
        <a href="https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml">如何查找填写在 package.xml 的 rosdep key？</a>
    </summary>
基于 https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml 中的信息，将对应的 key 填写到 package.xml 中

| rosdep key    | Ubuntu        |
|---------------|---------------|
| eigen         | libeigen3-dev |
| libopencv-dev | libopencv-dev |

</details>

## Reference

| 摘要                                  | 链接                                                                               |
|-------------------------------------|----------------------------------------------------------------------------------|
| 基于 ROS 生成 debian 包（适用于 ROS2 和 ROS1） | https://docs.ros.org/en/iron/How-To-Guides/Building-a-Custom-Debian-Package.html |
| ROS1 & ROS2 程序打包                    | https://zhuanlan.zhihu.com/p/578951132                                           |

