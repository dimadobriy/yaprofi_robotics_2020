#!/bin/bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCroscore | E[0]}" )/" && pwd )"

#!!! starts simulation for 600 seconds
# then exit
roscore | /opt/csim/coppeliaSim.sh -h -s600000 -q -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE ${ROOT_DIR}/scenes/bac_scene.ttt


