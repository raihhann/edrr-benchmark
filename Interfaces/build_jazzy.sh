#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source /opt/ros/jazzy/setup.bash
cmake -S "$root/turtlebot3_msgs" -B "$root/Build" -DCMAKE_INSTALL_PREFIX="$root/Install" -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
cmake --build "$root/Build" --parallel 2
cmake --install "$root/Build"
source "$root/setup.bash"
python3 -c 'from turtlebot3_msgs.msg import SensorState; print("Matching SensorState interface ready")'
