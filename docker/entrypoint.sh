#!/bin/bash
# Source ROS 2 and the workspace, then run the given command.
set -e

source /opt/ros/humble/setup.bash
source "${WS:-/turtlebot_3}/install/setup.bash"

exec "$@"
