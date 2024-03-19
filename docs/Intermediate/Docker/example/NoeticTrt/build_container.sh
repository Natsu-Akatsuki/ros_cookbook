#!/bin/bash

test -d ${HOME}/NoeticTrt || mkdir ${HOME}/NoeticTrt

# 参数配置
set_container_name="--name=NoeticTrt"
image_name="877381/helios:noetic-trt1.0"
user_name="helios"

# 文件挂载
projectVolume="--volume ${HOME}/NoeticTrt:/home/${user_name}/NoeticTrt:rw"
datasetVolume="--volume ${HOME}/mnt/rosbag:/home/${user_name}/Data/rosbag:rw"
set_volumes="${projectVolume} ${datasetVolume}"

# 开启端口（host:container）
vncPORT="-p 15900:5900"
pycharmPORT="-p 31111:22"
jupyterPORT="-p 8888:8888"
tensorboardPORT="-p 6006:6006"
set_network="${vncPORT} ${pycharmPORT} ${jupyterPORT} ${tensorboardPORT}"

# 设备限制
set_shm="--shm-size=8G"

# --device=/dev/dri:/dev/dri：支持集显
docker run -it --rm --gpus all -u 1001:1001 \
    --device=/dev/dri:/dev/dri
    ${set_volumes} \
    ${set_network} \
    ${set_shm} \
    ${set_container_name} \
    ${image_name}
