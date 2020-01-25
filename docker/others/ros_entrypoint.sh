#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/home/catkin_ws/devel/setup.bash"
exec "$@"

