#!/bin/bash
# Camera only: never starts the robot base driver or requests a mode change.
set -eo pipefail
root="$HOME/MA_Raihan"
role=$(cat "$root/role")
workspace=''
for candidate in "$HOME/d455_ros_ws" "$root/optional_ws";do
 if [[ -f "$candidate/devel/setup.bash" && -f "$candidate/devel/lib/librealsense2_camera.so" ]];then workspace=$candidate;break;fi
done
[[ -n "$workspace" ]] || { echo 'No built RealSense driver workspace found.' >&2;exit 1; }
source "$workspace/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311 ROS_LOG_DIR="$root/logs/ros"
if [[ "$role" == pi ]];then export ROS_IP=192.168.123.161;else export ROS_IP="192.168.123.$role";fi
unset ROS_HOSTNAME
mkdir -p "$root/logs" "$root/pids" "$root/config"
speed=$(python2 - <<'PYUSB'
import glob,os
for path in glob.glob('/sys/bus/usb/devices/*'):
 try:
  if open(path+'/idVendor').read().strip()=='8086' and open(path+'/idProduct').read().strip().lower()=='0b5c':
   print(open(path+'/speed').read().strip());break
 except IOError:pass
PYUSB
)
[[ -n "$speed" ]] || { echo 'D455 is not attached to this controller.' >&2;exit 1; }
config="$root/config/realsense.launch"
if [[ "${speed%%.*}" -lt 5000 ]];then
 config="$root/config/realsense_usb2.launch"
 echo "D455 USB link is ${speed} Mb/s; using reduced depth profile. Full USB3 performance is unverified."
fi
started=no
if python2 -c 'import rosnode,sys;sys.exit(0 if rosnode.rosnode_ping("/camera/realsense2_camera",max_count=1,verbose=False) else 1)' >/dev/null 2>&1;then
 echo 'RealSense node already running; checking actual images and IMU.'
else
 if [[ -s "$root/pids/realsense" ]];then
  pid=$(cat "$root/pids/realsense")
  if [[ "$pid" =~ ^[0-9]+$ && -r /proc/$pid/cmdline ]] && tr '\0' ' ' < /proc/$pid/cmdline | grep -Fq "$root/config/realsense";then
   echo 'Managed RealSense launch still active; not starting a duplicate.' >&2;exit 1
  fi
 fi
 if ! lsusb | grep -qi '8086:0b5c';then echo 'D455 USB device is not connected to this controller.' >&2;exit 1;fi
 nohup roslaunch "$config" >> "$root/logs/realsense.log" 2>&1 < /dev/null &
 printf '%s\n' "$!" > "$root/pids/realsense"
 started=yes
fi
timeout_s=8
[[ "$started" == yes ]] && timeout_s=45
if python2 "$root/scripts/realsense_readiness.py" --timeout "$timeout_s";then exit 0;fi
# Do not reset a camera while a native recording session is active.
python2 - <<'PYACTIVE'
import glob,os,sys
root=os.path.expanduser('~/MA_Raihan')
for name in glob.glob(root+'/pids/native_*'):
 try:
  pid=open(name).read().strip()
  if not pid.isdigit():continue
  command=open('/proc/'+pid+'/cmdline','rb').read()
  if (root+'/scripts/local_capture.py').encode() in command or b'/capture.bag' in command:
   sys.stderr.write('Native capture is active; refusing camera reset.\n');sys.exit(1)
 except IOError:pass
PYACTIVE
echo 'RealSense stream stalled; trying one camera-only reset (ROS master and robot control stay running).'
if ! timeout 15 rosservice call /camera/realsense2_camera/reset '{}';then
 echo 'RealSense reset failed; no recording/navigation started.' >&2;exit 1
fi
if ! python2 "$root/scripts/realsense_readiness.py" --timeout 45;then
 echo "RealSense still not ready. Inspect $root/logs/realsense.log and the USB cable/power." >&2;exit 1
fi
