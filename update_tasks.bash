#!/bin/bash

echo "[HARD] Update local git repository..."
git fetch --all
git reset --hard origin/master
git pull origin master

echo "Copy updated ros packages..."
cd ~/catkin_ws/src
rm -rf ./*
cp -rf ~/yaprofi_robotics_2020/*task ./
cp -rf ~/yaprofi_robotics_2020/test ./

#xhost +local:
#sudo docker run -it --rm --privileged --net=host --ipc=host \
#    --env="DISPLAY=$DISPLAY" \
#    --env="QT_X11_NO_MITSHM=1" \
#    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#    --volume "/dev/input:/dev/input" \
#    --volume="/home/human:/home/root" \
#    --name yaprofi-task-base-two \
#    yaprofi-task-base-two \
#    bash -c "cd /home/root/catkin_ws && catkin build"
