#!/bin/bash

echo "Some preparations..."
# cd ~ && git clone https://github.com/be2rlab/yaprofi_robotics_2020.git
mkdir -p catkin_ws/src && cd catkin_ws/src
cd ~/yaprofi_robotics_2020 && mv ./*task ./

echo "Installing docker dependencies"
cd ./docker/nvidia && sudo bash install_docker.bash

echo "Build docker image"
sudo bash build_docker.bash

echo "Test docker's 3D acceleration"
sudo bash test_docker.bash&







