#!/bin/bash
set -e

source "/opt/ros/humble/setup.bash"
source "/rusmed_ws/install/setup.bash"

pigpiod

ros2 launch rusmed bringup.launch.py

exec "$@"
