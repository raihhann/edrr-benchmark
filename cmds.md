# Unitree Go1 Multi-Terminal System Startup Guide

This guide documents the terminal commands required to bring up the Go1 robot base services, native camera pipelines, the Intel RealSense D455, and the rosbridge WebSocket server.

---

## 0. Connect the robot and laptop to the same Wi‑Fi network

Ensure the robot and the laptop are on the same Wi‑Fi network, and update the laptop routing table so it can reach the robot subnet.

```bash
sudo ip route replace 192.168.123.0/24 via 192.168.0.47
```

---

## 1. Raspberry Pi: Go1 base and low-level telemetry

### 1.1 Connect to the Raspberry Pi

```bash
ssh pi@192.168.123.161
```

### 1.2 Start the rosbridge server

```bash
source rosbridge_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:11311
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 1.3 Launch the base telemetry and control nodes

```bash
source thesis_ws/devel/setup.sh 
export ROS_MASTER_URI=http://localhost:11311
roslaunch go1_base base.launch working_mode:=high_level
```

Verification:
- Open a second terminal and run:

```bash
rostopic list
```

This confirms that the internal sensor topics are being published.

After the base is running, a basic functional check can be performed by launching the navigation script:

```bash
python3 Navigation/Scripts/navigation.py --distance 1 --speed 0.2
```

---

## 2. Jetson .14

### 2.1 Connect to Jetson .14

```bash
ssh unitree@192.168.123.14
```

### 2.2 Launch the camera stack

```bash
killall -9 point_cloud_node
cd ~/ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch go1_camera cameras_14.launch
```

### 2.3 Launch the Intel RealSense camera stack (optional)

```bash
cd ~/d455_ros_ws
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch realsense2_camera rs_camera.launch
```

### 2.4 Start the rosbridge WebSocket server

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 2.5 Run the camera collection scripts from DataCollection

From the laptop or another terminal:

```bash
python3 camera.py --host 192.168.123.14
```

For the RealSense camera:

```bash
python3 realsense_cam.py --host 192.168.123.14
```

---

## 3. Jetson .15

### 3.1 Connect to Jetson .15

```bash
ssh unitree@192.168.123.15
```

### 3.2 Launch the camera feed

```bash
killall -9 point_cloud_node
source ros_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:11311
roslaunch go1_camera cameras_15.launch
```

### 3.3 Start the rosbridge WebSocket server

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

You can verify the camera feed using:

```bash
python3 camera.py --host 192.168.123.15
```

---

## 4. Jetson .13

### 4.1 Connect to Jetson .13

```bash
ssh unitree@192.168.123.13
```

### 4.2 Launch the camera feed

```bash
killall -9 point_cloud_node
source go1_camera_ws/devel/setup.sh
export ROS_MASTER_URI=http://localhost:11311
roslaunch go1_camera cameras_13.launch
```

### 4.3 Start the rosbridge WebSocket server

```bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

You can verify the camera feed using:

```bash
python3 camera@13.py
```

---

## 5. Laptop: Data collection and bagging

### 5.1 Navigate to the project workspace and activate the virtual environment

```bash
cd ~/ROS/Go1_Project/DataCollection
source ../rosenv/bin/activate
```

### 5.2 Run the RealSense bagging script

```bash
python3 realsense_cam.py --host 192.168.123.14
```

### 5.3 Extract bag files into MP4 format

```bash
python3 vid_extract.py ../Bags/go1_bags/<bag_folder_name>
```

---

## Troubleshooting

If you encounter an error similar to the following:

```bash
File "/home/fsti/.local/lib/python3.12/site-packages/roslibpy/core.py", line 463, in call
    raise ServiceException(call_results["exception"])
roslibpy.core.ServiceException: Service /rosapi/topics does not exist
```

Run the following command on the specific controller that is not exposing the expected topics:

```bash
rosrun rosapi rosapi_node
```

This starts the `rosapi` node required for topic discovery via the `/rosapi/topics` service.
