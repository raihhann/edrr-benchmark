#!/bin/bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/ros_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311
role=$(cat "$root/role")
[[ "$role" == pi ]] && export ROS_IP=192.168.123.161 || export ROS_IP="192.168.123.$role"
unset ROS_HOSTNAME
export ROS_LOG_DIR="$root/logs/ros"
exec python2 -u "$root/scripts/local_capture.py" --role "$role" "$@"
