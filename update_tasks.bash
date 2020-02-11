#!/bin/bash

echo "[HARD] Update local git repository..."
git fetch --all
git reset --hard origin/master
git pull origin master


echo "Copy updated ros packages..."
cd ~/catkin_ws/src
cp -rf ~/yaprofi_robotics_2020/*task ./
cp -rf ~/yaprofi_robotics_2020/test ./


