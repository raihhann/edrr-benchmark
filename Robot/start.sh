#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/env.sh"
mkdir -p "$root/Logs" "$root/Runs" "$root/Bags" "$root/Inventory"
export MA_LIDAR=off
[[ -e /dev/ttyUSB0 ]] && export MA_LIDAR=on
camera=auto
for arg in "$@"; do
 case "$arg" in
  --lidar) [[ -e /dev/ttyUSB0 ]] || { echo 'No /dev/ttyUSB0: laser unavailable'; exit 1; }; export MA_LIDAR=on;;
  --no-lidar) export MA_LIDAR=off;;
  --no-camera) camera=off;;
  --camera) camera=on;;
  *) echo "Unknown start option: $arg"; exit 2;;
 esac
done
launch() {
 local name=$1; shift
 if [[ -f "$root/Runs/$name.pid" ]]; then
  local pid; pid=$(cat "$root/Runs/$name.pid")
  if kill -0 "$pid" 2>/dev/null && [[ -r /proc/$pid/cmdline ]] && tr '\0' ' ' < /proc/$pid/cmdline | grep -Fq "$root"; then
   echo "$name already running"; return
  fi
 fi
 nohup setsid "$@" > "$root/Logs/$name.log" 2>&1 < /dev/null &
 echo $! > "$root/Runs/$name.pid"
}
if [[ ! -e /dev/ttyACM0 ]]; then echo 'OpenCR /dev/ttyACM0 is missing'; exit 1; fi
camera_flag=false
if [[ "$camera" != off ]]; then
 if [[ -x "$root/Dependencies/root/opt/ros/humble/lib/camera_ros/camera_node" ]]; then
  camera_flag=true
 elif [[ "$camera" == on ]]; then
  echo 'Camera dependency bundle is missing; see README'; exit 1
 else
  echo 'Camera detected but driver bundle not prepared yet; see Inventory/availability.json'
 fi
fi
lidar_flag=false
[[ "$MA_LIDAR" == on ]] && lidar_flag=true
launch bringup ros2 launch "$root/Scripts/bringup.launch.py" "lidar:=$lidar_flag" "camera:=$camera_flag"
echo 'MA_Raihan bringup started; no navigation commands sent. Check Logs/ and inspect.sh.'
