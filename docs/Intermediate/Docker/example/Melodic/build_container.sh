#!/bin/bash

test -d ${HOME}/Melodic || mkdir ${HOME}/Melodic

# 参数配置
set_container_name="--name=Melodic"
image_name="877381/helios:melodic-1.0"
user_name="helios"

# 文件挂载
projectVolume="--volume ${HOME}/Melodic:/home/${user_name}/Melodic:rw"
datasetVolume="--volume ${HOME}/mnt/rosbag:/home/${user_name}/Data/rosbag:rw"
set_volumes="${projectVolume} ${datasetVolume}"

# 开启端口（host:container）
vncPORT="-p 25900:5900"
pycharmPORT="-p 31112:22"
set_network="${vncPORT} ${pycharmPORT}"

# 设备限制
set_shm="--shm-size=8G"

docker run -it \
    ${set_volumes} \
    ${set_network} \
    ${set_shm} \
    ${set_container_name} \
    ${image_name}

    
    


