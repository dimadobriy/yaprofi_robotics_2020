#!/usr/bin/env bash

source /home/root/catkin_ws/devel/setup.bash
roslaunch test run_test.launch | rosrun test test_node.py
