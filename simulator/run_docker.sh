#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

docker build -t georgno/drone .

if [ $? -ne 0 ]; then
    exit 1
fi


xhost +local:docker
docker run \
    -it --rm \
    $VOLUMES \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --name="drone" \
    georgno/drone
xhost -local:docker
