# Build Tool

ROS 编译工具根据迭代顺序（时间发展的顺序）依次有： `catkin_make`，`catkin_make_isolated`， `catkin_tools` ， `ament_tools`，`colon`

## Usage

### catkin_make

<details>
    <summary>:wrench: <b>用例 1：</b>
        常用命令行
    </summary>

```bash
# 单独编译某些package
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"
# 等价于：
$ catkin_make --only-pkg-with-deps
# 撤销白名单设置
$ catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# 使用ninja进行编译（编译速度会更快，但报错信息无高亮，日志可读性差）
$ catkin_make --use-ninja
```

> [!tip]
>
>  要屏蔽某些包被编译，也可以创建一个名为 `CATKIN_IGNORE`的文件到这些包所在的目录下

</details>

### catkin-tools

具体参考 [Here](https://catkin-tools.readthedocs.io/en/latest/index.html)

<details>
    <summary>:wrench: <b>用例 1：</b>
        <a href="https://catkin-tools.readthedocs.io/en/latest/installing.html">安装</a>
    </summary>

```bash
$ sudo apt-get update
$ sudo apt-get install python3-catkin-tools
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        常用命令行（编译，build）
    </summary>

```bash
# 跳过对某些已编译包的编译（实际上只是检查）
$ catkin build --start-with <pkg>
# 编译当前所处的包
$ catkin build --this
```

> [!note]
>
>  catkin build 的可查看日志 build.make.log

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        常用命令行（配置，config）
    </summary>

```bash
# 配置编译参数
$ catkin config -DPYTHON_EXECUTABLE=/opt/conda/bin/python3 \
-DPYTHON_INCLUDE_DIR=/opt/conda/include/python3.8 \
-DPYTHON_LIBRARY=/opt/conda/lib/libpython3.8.so
# 追加配置参数
$ catkin config -a <配置参数>
# 移除配置参数
$ catkin config -r <配置参数>

# 使用catkin_make参数
$ catkin config --catkin-make-args [args]

# 配置白名单（或黑名单）
$ catkin config --whitelist/blacklist <pkg>
# 取消白名单配置
$ catkin config --no-whitelist  
```

</details>

<details>
    <summary>:wrench: <b>用例 4：</b>
         常用命令行（清理中间文件，clean）
    </summary>

```bash
# 指定删除某个package
$ catkin clean <package_name>
# 删除所有 product 
$ catkin clean --deinit
# 移除非src文件夹下的包的编译产物 
$ catkin clean --orphans
```

> [!note]
>
>  catkin clean 默认删除 ``devel`` , ``log`` 等目录，但隐藏目录 ``.catkin_tools`` , ``.catkin_workspace`` 不会清除

</details>

<details>
    <summary>:wrench: <b>用例 5：</b>
        常用命令行（<a href="https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html#profile-cookbook">profile</a>）
    </summary>

```bash
$ catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug
$ catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build --profile debug
$ catkin build --profile release

$ alias catkin_debug="catkin build --profile debug"
$ alias catkin_release="catkin build --profile release"

# -x: 文件夹后缀
```

</details>

### colcon

具体参考 [Here](https://colcon.readthedocs.io/en/released/user/quick-start.html)

<details>
    <summary>:wrench: <b>用例 1：</b>
       <a href="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#">安装</a> 
    </summary>

```bash
# 安装
$ sudo apt install python3-colcon-common-extensions

# 配置跳转
$ echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
&& echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# 配置命令行Tab补全
$ echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# 配置clean拓展插件
$ git clone https://github.com/ruffsl/colcon-clean
$ python3 setup.py install --user
# colcon clean
$ colcon clean packages --base-select build install
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        常用命令行（编译，<a href="https://colcon.readthedocs.io/en/released/user/how-to.html">build</a>）
    </summary>

暂未发现其支持像 `catkin build` 中的 `context-aware` 功能

```bash
# 编译工作空间的所有pkg
$ colcon build
# 只编译部分包
$ colcon build --packages-select <pkg_name>
# 使用符号链接而不是复制文件进行安装
$ colon build --symlink-install

# options:
# --cmake-args -DCMAKE_BUILD_TYPE=Debug
# --event-handlers console_direct+   编译时显示所有编译信息
# --event-handlers console_cohesion+  编译完一个包后才显示它的编译信息
# --packages-select <name-of-pkg>  编译某个特定的包（不包含其依赖）
# --packages-up-to <name-of-pkg>   编译某个特定的包（包含其依赖）
# --packages-above <name-of-pkg>  重新编译某个包（和依赖这个包的相关包）

# source devel/setup.bash的等价命令
$ source install/local_setup

# 使用软链接进行安装
# 使用Ninja；
# 设置并行个数
# 日志及时输出到终端
$ colcon build --symlink-install \
--cmake-args -G Ninja \
--parallel-workers 8 \
--event-handlers console_direct+
```

</details>

<details>
    <summary>:wrench: <b>用例 3：</b>
        常用命令行（显示相关信息）
    </summary>

```bash
# 显示当前工作空间的所有包的信息
$ colcon list
# List all packages in the workspace in topological order and visualize their dependencies
$ colcon graph           
```

</details>

### Others

<details>
    <summary>:wrench: <b>用例 1：</b>
        could not find a package configuration file (catkin build)
    </summary>

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20210912141918386.png ':size=600')

字面意思，即找不到某个包的配置文件（e.g. .cmake 文件），一般是因为没有安装或编译该依赖包，或者没有定义 CMAKE_PREFIX_PATH，相关 CheckList 如下：

|                             案例                             |                           解决方案                           |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| apt 依赖包尚未安装<br />->依赖包缺失<br />->没有依赖包配置文件 |                          安装依赖包                          |
| 自建依赖包尚未编译<br />->依赖包缺失<br />->没有依赖包配置文件 | 编译依赖包。其中或存在编译顺序问题，检查 package.xml 文件是否指明了依赖包，以便 catkin tool 编译时能先编译特定的依赖包才编译当前包 |
| 依赖包存在<br />->依赖包配置文件存在<br />->环境变量尚未导入 | 检查是否导入了 ROS 环境变量（e.g. source /opt/ros/noetic/setup.bash）；需根据不同的终端模拟器导入不同的文件，<br />bash 对应 setup.bash，sh 对应 setup.sh，zsh 对应 setup.zsh |
| 依赖包存在<br />->依赖包配置文件存在<br />->环境变量已导入<br />->CMAKE_PREFIX_PATH 为空 | 配置文件存在，导入了 ROS 环境变量，检查是否在 CMakeLists.txt 中定义了 CMAKE_PREFIX_PATH（可能通过调用 find_package() 会导入）。在 ROS1 中在 find_package(catkin) 前不能定义 CMAKE_PREFIX_PATH，否则 find_package(catkin) 中不会将环境变量 CMAKE_PREFIX_PATH（e.g. /opt/ros/noetic）添加到 CMAKE 变量 CMAKE_PREFIX_PATH 中（相关处理逻辑参考 [Here](https://github.com/ros/catkin/issues/1161)），导致 ROS 的一系列配置文件无法被找到（一种解决方案是进行 [reorder](https://github.com/ros-perception/perception_pcl/pull/352/commits/3792938b53592422c324df734e328837107ae7e3)，即调整顺序） |

对于安装依赖包可尝试：

```bash
# 如提示 Could not find a package configuration file provided by “gazebo_plugins”
# 一般是追加 ros-$ROS_DISTRO- 前缀，下划线替换为中划线
$ sudo apt install ros-$ROS_DISTRO-gazebo-plugins
```

</details>

<details>
    <summary>:wrench: <b>用例 2：</b>
        目标文件命名冲突（for catkin）
    </summary>

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/M5KhRzVvmtcWapDQ.png!thumbnail 'rslidar 和 velodyne 包的目标文件重名')

</details>