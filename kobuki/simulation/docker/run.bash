#!/bin/bash


name=$(cat ./name.txt)

if [[ $name == "" ]]
then
    echo "container name in name.txt is empty"
    exit 1
fi


SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true


### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm \
            -e DISPLAY=$DISPLAY \
            -e "QT_X11_NO_MITSHM=1" \
            -e XAUTHORITY \
            -v /dev:/dev \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v ${SIM_ROOT}/workspace:/workspace \
            --device /dev/bus/usb \
            --net=host \
            --privileged \
            --name $name \
            $name \
            > /dev/null
