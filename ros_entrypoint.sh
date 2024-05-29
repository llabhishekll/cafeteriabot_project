#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"

# execute user cmd
exec "$@"