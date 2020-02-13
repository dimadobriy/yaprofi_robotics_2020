#!/usr/bin/env bash

source /home/root/catkin_ws/devel/setup.bash

cd /home/root/catkin_ws
catkin build

roslaunch bac_task init_scene.launch | /opt/csim/coppeliaSim.sh -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE /home/root/catkin_ws/src/bac_task/scenes/bac_scene.ttt
