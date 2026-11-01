#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
source "$root/env.sh"
if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
 echo 'RealSense driver is not installed in this Humble environment. Connect the camera and prepare its compatible driver first.' >&2
 exit 1
fi
exec ros2 launch realsense2_camera rs_launch.py "$@"
