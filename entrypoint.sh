#!/bin/bash

set -e

source "/opt/ros/humble/setup.bash"
source "/navigation_ws/install/setup.bash"

exec "$@"
