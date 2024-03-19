# >>> 基础镜像 >>>
FROM osrf/ros:melodic-desktop-full

# >>> 修改源，并安装apt包 >>>
# rm /etc/apt/sources.list.d/*: 删除cuda sources，避免无法update
RUN sed -i s@/archive.ubuntu.com/@/mirrors.gdut.edu.cn/@g /etc/apt/sources.list \
    && sed -i '37,49d' /etc/apt/sources.list \
    && rm /etc/apt/sources.list.d/* \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        apt-utils \
        bash-completion \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        gdb \
        git \
        htop \
        keyboard-configuration \
        libx11-dev \
        lldb \
        locales \
        lsb-core \
        mlocate \
        nano \
        net-tools \
        openssh-server \
        software-properties-common \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

# >>> 安装ROS >>>
RUN DEBIAN_FRONTEND=noninteractive  sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y ros-melodic-desktop-full \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        python-pip \
        && rm -rf /var/lib/apt/lists/*

# >>> 安装其他ros包依赖 >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        libboost-all-dev \
        libboost-python-dev \
        libeigen3-dev \
        libfmt-dev \
        libgeographic-dev \
        libglfw3-dev \
        libglm-dev \
        libgtest-dev \
        libpugixml-dev \
        libpcap-dev \
        ros-melodic-angles \
        ros-melodic-camera-info-manager \
        ros-melodic-ddynamic-reconfigure \
        ros-melodic-diagnostic-updater \
        ros-melodic-geodesy \
        ros-melodic-jsk-recognition-msgs ros-melodic-visualization-msgs \
        ros-melodic-lanelet2 \
        ros-melodic-nav-msgs \
        ros-melodic-nmea-msgs \
        ros-melodic-tf2-sensor-msgs \
    # catkin build的python依赖
	python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# >>> 安装其他apt包依赖 >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        htop \
    && rm -rf /var/lib/apt/lists/*

# >>> 安装gtsam,ceres,ospq,Sophus,g2o,protobuf >>>
RUN  DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:borglab/gtsam-release-4.0 -y \
    && apt update \
    && apt install libgtsam-dev libgtsam-unstable-dev -y \
    && cd /tmp \
    && apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    && wget -c http://ceres-solver.org/ceres-solver-2.1.0.tar.gz -O /tmp/ceres-solver-2.1.0.tar.gz \
    && cd /tmp \
    && tar -xzvf ceres-solver-2.1.0.tar.gz \
    && cd /tmp/ceres-solver-2.1.0 \
    && mkdir build && cd build && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp/ \
    && git clone --depth=1 --recursive https://github.com/osqp/osqp osqp \
    && cd osqp && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp \
    && git clone https://github.com/strasdat/Sophus.git --depth=1 \
    && cd Sophus \
    && mkdir build && cd build && cmake .. \
    && make -j$(nproc) && make install \
    && cd /tmp \
    && git clone https://github.com/RainerKuemmerle/g2o --depth=1 --branch=20200410_git \
    && cd g2o \
    && mkdir build && cd build && cmake .. \
    && make -j$(nproc) && make install \
    && cd /tmp \
    && git clone https://github.com/protocolbuffers/protobuf --depth=1 --branch=3.15.x \
    && cd protobuf \
    && cd cmake && mkdir build && cd build && cmake -Dprotobuf_BUILD_TESTS=OFF .. \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/* \
    && rm -rf /var/lib/apt/lists/*

# >>> 解决中文显示乱码问题 >>>
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# >>> 安装vnc server >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        tigervnc-common \
        tigervnc-standalone-server \
        tigervnc-xorg-extension \
    && rm -rf /var/lib/apt/lists/*

# >>> 安装lxde display manager >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --fix-missing \
        dolphin \
        konsole \
        lxde \
    && rm -rf /var/lib/apt/lists/*

# >>> 解决中文显示乱码问题 >>>
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# >>> 入口点函数 >>>
ENTRYPOINT ["/bin/bash"]

# >>> 修改用户权限 >>>
ARG USER_NAME=helios
RUN useradd ${USER_NAME} -m -G sudo -u 1000 -s /bin/bash && yes ${USER_NAME} | passwd ${USER_NAME}
USER ${USER_NAME}

# >>> 配置vnc server和ros >>>
RUN mkdir -p $HOME/.vnc \
    && echo "#!/bin/bash" >> $HOME/.vnc/xstartup \
    && echo "startlxde &" >> $HOME/.vnc/xstartup \
    && echo "admin" | vncpasswd -f >> $HOME/.vnc/passwd && chmod 600 $HOME/.vnc/passwd \
    && echo "geometry=1920x1080" >> $HOME/.vnc/config \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "alias vncserver='vncserver --localhost no :0'" >> ~/.bashrc

WORKDIR /home/${USER_NAME}