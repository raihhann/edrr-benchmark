#!/bin/bash
echo "Starting Jetson Nodes..."
killall -9 point_cloud_node 2>/dev/null
echo "123" | sudo -S systemctl restart nvargus-daemon 2>/dev/null

source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

echo "Starting Native Cameras..."
source ~/ros_ws/devel/setup.sh
nohup roslaunch go1_camera cameras_15.launch > /tmp/camera.log 2>&1 &
echo "Waiting for native cameras to register..."
while ! rostopic list 2>/dev/null | grep -q "image"; do sleep 1; done

echo "Starting compressed camera publishers..."
if ! bash "$HOME/go1_sync_tools/restart_compression.sh"; then
    echo "Compression startup failed; inspect ~/go1_sync_tools/*.log" >&2
    exit 1
fi

echo "Starting Bridge & API..."
source /opt/ros/melodic/setup.bash
nohup roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 > /tmp/bridge.log 2>&1 &
echo "Jetson Nodes Started Successfully."
