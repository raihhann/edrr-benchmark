#!/bin/bash
echo "Starting Jetson .14 Nodes..."
killall -9 point_cloud_node 2>/dev/null
echo "123" | sudo -S systemctl restart nvargus-daemon 2>/dev/null

source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

echo "Starting Native Cameras..."
source ~/ros_ws/devel/setup.bash
nohup roslaunch go1_camera cameras_14.launch > /tmp/14_cameras.log 2>&1 &
echo "Waiting for native cameras to register..."
while ! rostopic list 2>/dev/null | grep -q "/camera/left/image"; do sleep 1; done

read -p "Do you want to launch the Intel RealSense camera? (y/n): " choice
case "$choice" in 
  y|Y ) 
    echo "Starting RealSense..."
    source ~/d455_ros_ws/devel/setup.bash
    nohup roslaunch realsense2_camera rs_camera.launch > /tmp/14_realsense.log 2>&1 &
    echo "Waiting for RealSense to register..."
    while ! rostopic list 2>/dev/null | grep -q "/camera/color/image_raw"; do sleep 1; done
    ;;
  * ) echo "Skipping RealSense." ;;
esac

echo "Starting Bridge & API..."
source /opt/ros/melodic/setup.bash
nohup roslaunch rosbridge_server rosbridge_websocket.launch > /tmp/14_bridge.log 2>&1 &
echo "Jetson .14 Nodes Started Successfully."
