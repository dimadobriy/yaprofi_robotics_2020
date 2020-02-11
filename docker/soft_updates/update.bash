#!/bin/bash

echo "Updating..."
cd ~/catkin_ws/src
rm -rf ./*task
cp -rf ~/yaprofi_robotics_2020/*task ./

cd ~/yaprofi_robotics_2020/docker/soft_updates/
cp -rf run_docker.bash ~/Desktop/
cp -rf exec_docker.bash ~/Desktop/

echo "Update docker image"
sudo bash build_docker.bash
