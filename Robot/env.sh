#!/usr/bin/env bash
# Source this file; it changes only the current process environment.
MA_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export MA_TURTLEBOT_ROOT="$MA_ROOT"
source /opt/ros/humble/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
if [[ -d "$MA_ROOT/Install/ma_turtlebot_bringup/share/ament_index" ]]; then
 export AMENT_PREFIX_PATH="$MA_ROOT/Install/ma_turtlebot_bringup:$AMENT_PREFIX_PATH"
fi
export ROS_DOMAIN_ID=30 TURTLEBOT3_MODEL=waffle_pi LDS_MODEL=LDS-02
# Acquisition/navigation run on the robot; SSH is the laptop transport.
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$MA_ROOT/Logs/ros"
mkdir -p "$ROS_LOG_DIR"
if [[ -d "$MA_ROOT/Dependencies/root/opt/ros/humble" ]]; then
 export AMENT_PREFIX_PATH="$MA_ROOT/Dependencies/root/opt/ros/humble:$AMENT_PREFIX_PATH"
 export LD_LIBRARY_PATH="$MA_ROOT/Dependencies/root/opt/ros/humble/lib:$MA_ROOT/Dependencies/root/usr/lib/aarch64-linux-gnu:$MA_ROOT/Dependencies/root/usr/lib:$LD_LIBRARY_PATH"
fi
