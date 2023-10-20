#!/bin/bash

# Written by Nikolay Dema <ndema2301@gmail.com>, September 2022

KOB_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true

IMG_NAME="hsl_2023_kobuki_solution"
CTR_NAME="hsl_2023_kobuki_solution"


### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -e XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v ${KOB_ROOT}/workspace:/workspace \
            -v /dev:/dev \
            -e "ROS_DOMAIN_ID=100" \
            --net=host \
            --privileged \
            --name ${CTR_NAME} ${IMG_NAME} \
            > /dev/null
