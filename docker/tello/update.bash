#!/bin/bash

echo "Updating..."

cd ~/yaprofi_robotics_2020/docker/tello/
cp -rf run_docker.bash ~/Desktop/
cp -rf exec_docker.bash ~/Desktop/

echo "Update docker image"
sudo bash build_docker.bash
