#!/bin/bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"

roscore | /opt/csim/coppeliaSim.sh -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE ${ROOT_DIR}/scenes/bac_scene.ttt
#/opt/csim/coppeliaSim.sh ${ROOT_DIR}/scenes/bac_scene.ttt


