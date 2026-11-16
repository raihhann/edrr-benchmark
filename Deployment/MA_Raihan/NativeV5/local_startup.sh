#!/bin/bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
source "$root/ros_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311
role=$(cat "$root/role")
[[ "$role" == pi ]] && export ROS_IP=192.168.123.161 || export ROS_IP="192.168.123.$role"
unset ROS_HOSTNAME
command -v rosbag >/dev/null
mkdir -p "$root/pids" "$root/logs" "$root/captures"
if [[ "$role" != pi ]];then
  running=false
  if [[ -s "$root/pids/camera_recovery" ]];then
    pid=$(cat "$root/pids/camera_recovery")
    if [[ "$pid" =~ ^[0-9]+$ && -r /proc/$pid/cmdline ]];then
      command=$(tr '\0' ' ' < "/proc/$pid/cmdline")
      [[ "$command" == *"$root/scripts/camera_recovery.py"* ]] && running=true
    fi
  fi
  if [[ "$running" == false ]];then
    nohup python2 -u "$root/scripts/camera_recovery.py" >> "$root/logs/camera_recovery.log" 2>&1 < /dev/null &
    echo $! > "$root/pids/camera_recovery"
  fi
fi
echo "Native local ROS1 capture ready on role $role. Laptop V5 coordinates recording and verified download."
