#!/bin/bash
echo "Stopping ROS Data Collection Nodes..."

# Kill only our custom launch processes and bridge (Leaves roscore alive)
pkill -f roslaunch
pkill -f rosbridge

# Kill the specific camera nodes
killall -9 go1_camera_node 2>/dev/null
killall -9 rs_camera 2>/dev/null

# Reset the camera daemon on Jetsons (fails silently on the Pi)
echo "123" | sudo -S systemctl restart nvargus-daemon 2>/dev/null

echo "All custom nodes stopped and camera drivers reset. Default ROS Master is still running."
