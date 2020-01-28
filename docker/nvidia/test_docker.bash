#!/usr/bin/env bash
xhost +local:

docker run -it --rm --privileged  --net=host --ipc=host \
    --gpus all \
    --runtime=nvidia \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "/dev/input:/dev/input" \
    --volume="/home/human:/home/$USER" \
    -e ROS_IP=localhost \
    --name yaprofi-task-base \
    yaprofi-task-base \
    glxgears
