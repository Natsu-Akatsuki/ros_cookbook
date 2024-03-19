## NoeticTrt

### 环境

- ubuntu 20.04
- ros noetic
- cudnn 11.1, cudnn8, trt 7.2.3.4, pytorch1.12
- pcl 1.10
- ceres 2.1
- osqp (github latest)
- gtsam (apt) 
- display manager: lxde

### 创建镜像

根据当前的地址修改 Dockerfile 的镜像源

|              |                     |      |
| ------------ | ------------------- | ---- |
| 广东工业大学 | mirrors.gdut.edu.cn |      |
| 西安交通大学 | mirrors.xjtu.edu.cn |      |
|              |                     |      |



```bash
# 搭建基础镜像
$ bash build_image.sh
# 镜像改名
$ docker tag helios:noetic-trt1.0 877381/helios:noetic-trt1.0

# 导入trt模块
# 从官网下载deb包，然后在容器中下载
$ sudo dpkg -i nv-tensorrt-repo-ubuntu1804-cuda11.1-trt7.2.3.4-ga-20210226_1-1_amd64.deb
$ sudo apt update && sudo apt install tensorrt
```

### 用例

- 构建容器

```bash
$ bash build_container.sh
# 在容器中启动vncserver
(helios) $ vncserver
# 若容器关闭再启动，则需要移除原先的配置再执行一次vncserver
$ vncserver --kill :0
$ vncserver
```

- 主机端启动vncviewer

```bash
# 需要依赖tigervnc-viewer
$ sudo apt install tigervnc-viewer
$ vncviewer localhost:15900
# 登录密码为admin
# helios用户密码为（即使用sudo权限时的密码）helios
```

### BUG

- 基于X11的ros rviz(noetic)无法修改此处的值

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220329145821156.png" alt="image-20220329145821156" style="zoom: 50%;" />

### 日志

- 2022.07.05：Sleipnir工程基础环境搭建
