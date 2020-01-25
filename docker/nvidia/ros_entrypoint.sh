#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetics/setup.bash"
source "/home/catkin_ws/devel/setup.bash"
exec "$@"

