#!/bin/bash
echo "Starting Pi Nodes..."
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

echo "Waiting for default ROS Master..."
while ! rostopic list > /dev/null 2>&1; do sleep 1; done

echo "Starting Go1 Base..."
source ~/thesis_ws/devel/setup.sh 
nohup roslaunch go1_base base.launch working_mode:=high_level > /tmp/pi_base.log 2>&1 &

read -p "Do you want to launch the RoboSense LiDAR? (y/n): " choice
case "$choice" in 
  y|Y ) 
    echo "Setting up LiDAR Network..."
    echo "123" | sudo -S ifconfig eth0:0 192.168.1.102 up 2>/dev/null

    echo "Starting RoboSense LiDAR..."
    source ~/robosense_ws/devel/setup.bash
    nohup roslaunch rslidar_sdk start.launch > /tmp/pi_lidar.log 2>&1 &
    echo "Waiting for LiDAR to register..."
    while ! rostopic list 2>/dev/null | grep -q "/rslidar_points"; do sleep 1; done
    echo "LiDAR active."
    ;;
  * ) 
    echo "Skipping RoboSense LiDAR."
    ;;
esac

echo "Starting Bridge..."
source ~/rosbridge_ws/devel/setup.sh
nohup roslaunch rosbridge_server rosbridge_websocket.launch > /tmp/pi_bridge.log 2>&1 &
echo "Pi Nodes Started Successfully."
