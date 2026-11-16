#!/bin/bash
# Add the saved LiDAR receiving address and verify actual point-cloud arrival.
set -eo pipefail
root="$HOME/MA_Raihan"
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.123.161
unset ROS_HOSTNAME
workspace=''
for candidate in "$root/optional_ws" "$HOME/robosense_ws"; do
  if [[ -f "$candidate/devel/setup.bash" && -x "$candidate/devel/lib/rslidar_sdk/rslidar_sdk_node" ]]; then
    workspace="$candidate"; break
  fi
done
if [[ -z "$workspace" ]]; then
  echo 'LiDAR driver is not built. Run: bash ~/MA_Raihan/scripts/prepare_optional.sh' >&2
  exit 1
fi
source "$workspace/devel/setup.bash"
package=$(rospack find rslidar_sdk)
mkdir -p "$root/config" "$root/logs" "$root/pids" "$root/state"
config="$root/config/lidar_startup.yaml"
settings=$(python2 "$root/scripts/lidar_settings.py" "$package/config/config.yaml" "$config")
read -r address topic model <<< "$settings"
echo "LiDAR: $model, Pi receiving address $address/24 on eth0, topic $topic"
echo "Configuration: $config (controller-clock timestamps; no hardware clock sync asserted)"
if ! ip -4 -o addr show dev eth0 | awk '{print $4}' | cut -d / -f 1 | grep -Fxq "$address"; then
  sudo ip address add "$address/24" dev eth0 label eth0:ma_lidar
  # Record only the address this helper actually added, for conservative rollback.
  printf '%s\n' "$address/24" > "$root/state/lidar_added_address"
fi
# Check the ROS node as well as the PID file, avoiding a second socket listener.
if python2 -c 'import rosnode,sys;sys.exit(0 if rosnode.rosnode_ping("/rslidar_sdk_node",max_count=1,verbose=False) else 1)' >/dev/null 2>&1; then
  echo 'LiDAR node already running; checking actual messages.'
else
  if [[ -s "$root/pids/lidar" ]]; then
    pid=$(cat "$root/pids/lidar")
    if [[ "$pid" =~ ^[0-9]+$ && -r /proc/$pid/cmdline ]]; then
      command=$(tr '\0' ' ' < "/proc/$pid/cmdline")
      if [[ "$command" == *rslidar_sdk_node*"$config"* ]]; then
        echo 'A managed LiDAR process is still starting or unresponsive; not launching a duplicate.' >&2
        exit 1
      fi
    fi
  fi
  nohup "$workspace/devel/lib/rslidar_sdk/rslidar_sdk_node" \
    "_config_path:=$config" __name:=rslidar_sdk_node \
    >> "$root/logs/lidar.log" 2>&1 < /dev/null &
  printf '%s\n' "$!" > "$root/pids/lidar"
fi
export ROS_LOG_DIR="$root/logs/ros"
if ! python2 - "$topic" <<'PY'
from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import PointCloud2
rospy.init_node('ma_lidar_startup_check', anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message(sys.argv[1], PointCloud2, timeout=15.)
    if msg.width * msg.height == 0 or not msg.data:
        raise ValueError('Point cloud arrived but has no points')
    print('LiDAR verified: received %d points. Add --include-lidar to laptop V4 capture.' % (msg.width * msg.height))
except Exception as error:
    print('LiDAR not ready: %s' % error, file=sys.stderr)
    sys.exit(1)
PY
then
  echo "No usable point cloud verified. Inspect $root/logs/lidar.log and LiDAR power/model/destination IP/ports." >&2
  echo 'The driver is left listening; no motion commands were sent. Stop only LiDAR with: rosnode kill /rslidar_sdk_node' >&2
  exit 1
fi
