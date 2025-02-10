#!/bin/bash

set -e

source "/usr/share/gazebo/setup.bash"
source "/opt/ros/humble/setup.bash"
source "/navigation_ws/install/setup.bash"

exec "$@"
