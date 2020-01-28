#!/bin/bash

echo "Some preparations..."
cd ~ && mkdir -p catkin_ws/src && cd catkin_ws/src
cp -r ~/yaprofi_robotics_2020/*task ./
cd ~/yaprofi_robotics_2020

echo "Installing docker dependencies"
cd ./docker/nvidia && sudo bash install_docker.bash

echo "Build docker image"
sudo bash build_docker.bash

echo "Test docker's 3D acceleration"
sudo bash test_docker.bash











