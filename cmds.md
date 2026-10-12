# Unitree Go1 Multi-Terminal System Startup Guide

This document outlines the exact terminal commands required to bring up the Go1 robot's base services, native cameras, Intel RealSense D455, and the rosbridge websocket server.

---

## 1. Raspberry Pi (Go1 Robot Base & Low-Level Telemetry)

SSH into the Raspberry Pi:
```bash
ssh pi@192.168.123.161
```

Launch base telemetry and control nodes:
```bash
roslaunch go1_bringup go1_bringup.launch
```
* **Verification:** Run `rostopic list` to confirm internal sensor topics like `/lowstate` are publishing.

---

## 2. Jetson .15 (nx) - Companion Board

SSH into Jetson .15:
```bash
ssh unitree@192.168.123.15
```

Source workspace and export local ROS master:
```bash
cd ~/ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

---

## 3. Jetson .14 (unitree-desktop) - Native Cameras & RealSense

SSH into Jetson .14:
```bash
ssh unitree@192.168.123.14
```

### Terminal 3A: Launch Native Go1 Cameras (Front & Chin)
```bash
cd ~/ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch go1_camera go1_camera.launch
```

### Terminal 3B: Launch Intel RealSense D455 Camera
*(Ensure the camera is plugged into a blue USB 3.0 port)*
```bash
cd ~/d455_ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch realsense2_camera rs_camera.launch
```
* **Verification:** Check terminal logs for `RealSense Node Is Up!`.

### Terminal 3C: Launch Rosbridge Websocket Server
```bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rosbridge_server rosbridge_websocket.launch
```
* **Verification:** Confirm the websocket server starts listening on port 9090.

---

## 4. Laptop (Data Collection & Bagging)

Navigate to your workspace and activate your virtual environment:
```bash
cd ~/ROS/Go1_Project/DataCollection
source ../rosenv/bin/activate
```

Run the RealSense bagger script pointing to Jetson .14:
```bash
python3 realsense_cam.py --host 192.168.123.14
```

To extract and synchronize the resulting bag into viewable MP4 files:
```bash
python3 vid_extract.py ../Bags/go1_bags/<bag_folder_name>
```