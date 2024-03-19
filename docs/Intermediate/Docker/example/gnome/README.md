# VNC

- for gnome

```dockerfile
# 如下为比较重要的代码块
# >>> 安装和配置vnc server >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        tigervnc-common \
        tigervnc-standalone-server \
        tigervnc-xorg-extension \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p $HOME/.vnc \
    && echo "admin" | vncpasswd -f >> $HOME/.vnc/passwd && chmod 600 $HOME/.vnc/passwd
    
COPY tigervnc@.service /etc/systemd/system/tigervnc@.service
RUN systemctl enable tigervnc@:1
COPY xstartup /root/.vnc/xstartup

# >>> 安装gnome display manager >>>
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --fix-missing \
        ubuntu-desktop \
        gnome-shell-extensions \
        gnome-tweaks \
        gnome-tweak-tool \
        gnome-panel \
    && rm -rf /var/lib/apt/lists/*

# >>> 入口点函数 >>>
ENTRYPOINT ["/usr/sbin/init"]
```

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220331171241039.png" alt="image-20220331171241039" style="zoom:67%;" />

- 实测需要较高的管理员权限，容易影响主机系统

## Reference

[gnome vnc 实例](https://github.com/RavenKyu/docker-ubuntu-desktop-vnc/blob/main/Dockerfile)
