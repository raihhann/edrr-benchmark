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


## Latest v4 deployment (2026-10-23)

Use `python3 nav_and_collect_v4.py --navigate --distance 1 --speed 0.2` from NavigationScripts after starting the Pi with `--control`. V4 uses the single stamped odometry subscription and an independent recording writer. The real short movement test passed with the reduced Pi telemetry profile; rates and limitations are documented in Deployment/MA_Raihan/README.md and movement_verification.json.


## Navigation and automatic run-quality report — v5 (2026-11-12)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ROS/Go1_Project/NavigationScripts
python3 nav_and_collect_v5.py --navigate --distance 1 --speed 0.2
```

Navigation is explicit; stationary capture uses `--record-only --duration 20`. Each finalized bag receives `run_context.json`, `quality_report.json` and `quality_report.md`. The terminal prints VALID, VALID_WITH_WARNINGS, INVALID (fix and rerun), or UNVERIFIED. Reruns are never automatic. Default maximum missing ROS message sequences is 1% per live stream; use `--max-loss-percent 0` for strict zero loss. These are ROS message gaps, not network packet-loss measurements. See `NavigationScripts/RUN_QUALITY.md` for all rate, delay, alignment and completion thresholds. V4 and controller startup scripts remain unchanged.

Audit an old bag without contacting the robot:

```bash
python3 nav_and_collect_v5.py --audit-only /absolute/path/to/bag
```


## Pi startup with optional LiDAR — lidar_prompt_20261113_192302

On the Pi, `bash ~/start_pi.sh` asks whether the RoboSense LiDAR is connected
and powered. Answer `y` to configure its receiving address and start/check the
driver, or `n`/Enter to skip LiDAR startup. Default startup stays passive.

```bash
# Pi: interactive startup, then answer y or n
bash ~/start_pi.sh
# Pi: explicitly enable robot control; still asks about LiDAR
bash ~/start_pi.sh --control
# Pi: explicit options for automation (sudo access may be required)
bash ~/start_pi.sh --lidar
bash ~/start_pi.sh --no-lidar
bash ~/start_pi.sh --control --lidar
```

No password is embedded. `n` skips startup; it does not shut down an already
running LiDAR. Non-interactive calls default to no unless `--lidar` is given.
The helper reuses `~/robosense_ws` or a built MA_Raihan optional workspace.
It never rebuilds ROS Melodic. If no driver binary exists it reports the exact
optional-workspace build command instead of installing packages automatically.

First use creates `~/MA_Raihan/config/lidar_startup.yaml` from the driver config,
preserving the original. Current configuration: RSHELIOS, host 192.168.1.102/24
on eth0, UDP 6699/7788. The receiving address is added alongside the robot IP;
the robot address and default route are preserved. The LiDAR itself must send
to this destination. Edit the separate config if the model/network differs.
Initial timestamps use the Pi clock (`use_lidar_clock: false`); this is not
hardware synchronization. Raw-packet publication remains as configured (off
in the current base config).

No RViz is started. A running node is reused, and a nonempty point-cloud message
must arrive within 15 seconds before success is reported. On failure inspect
`~/MA_Raihan/logs/lidar.log`; the driver stays listening for the sensor.
Stop only LiDAR with `rosnode kill /rslidar_sdk_node` in its ROS environment.

Laptop, from `~/ROS/Go1_Project`, after the driver verifies actual points:

```bash
source /opt/ros/jazzy/setup.bash
python3 NavigationScripts/nav_and_collect_v4.py --record-only --duration 15 --include-lidar
# Deliberate navigation, only after reviewing the stationary capture:
python3 NavigationScripts/nav_and_collect_v4.py --navigate --distance 1 --speed 0.2 --include-lidar
```

Add `--include-realsense` separately when RealSense is connected and its driver
is started on .14. These recording flags do not launch sensor drivers.

Pi one-command rollback: `bash /home/pi/MA_Raihan/updates/lidar_prompt_20261113_192302/undo.sh`.
Laptop mirror and command-guide rollback: `bash /home/fsti/ROS/Go1_Project/HardwareScripts/backups/lidar_prompt_20261113_192302/undo_local.sh`.
New helper/config/log files are retained rather than deleted; the restored old
entry point does not invoke them. These rollback commands are independent.

## Native V5: local recording and automatic download

See [Native V5 instructions](Deployment/MA_Raihan/NativeV5/README.md) for startup, collection, quality thresholds and rollback. V5 automatically produces one combined.bag after downloading and verifying the native recordings; original per-controller bags are retained for recovery.

```bash
python3 NavigationScripts/nav_and_collect_v5.py --record-only --duration 15 --include-lidar --optional-camera chin
python3 NavigationScripts/nav_and_collect_v5.py --navigate --distance 1 --speed 0.2 --include-lidar --optional-camera chin
```

Pi navigation startup requires `bash ~/start_pi.sh --control --lidar`. Front currently measures about 13.7 FPS, below the default acceptance threshold; chin remains unavailable. Finish the pending controller update after the battery change as described in the linked guide.

### V5 ROS2 db3 output and cleanup

The final file is `Bags/go1_bags/RUN_ID/combined_ros2/combined_ros2.db3` (keep its `metadata.yaml`). Native local backups remain. Remote `capture.bag` files are deleted only after verified downloads and ROS2 conversion; use `--keep-remote` to retain them. Controller run folders are `~/MA_Raihan/captures/`; old runs are not automatically purged. Disk preflight now requires 1.5 GiB. See the NativeV5 README for details.

### Pi startup and RealSense

On Pi run `bash ~/start_pi.sh --control --lidar` (or `--no-lidar`). Pi startup only asks about LiDAR; it does not start RealSense. Connect the D455 to the center USB port on Jetson .15 and add `--include-realsense` to the laptop V5 command. V5 detects the USB host and starts/verifies the camera there before capture.

### RealSense center USB port (.15)

`--include-realsense` now detects its actual USB controller automatically; optional override: `--realsense-controller 15`. V5 starts/verifies the camera-only helper on that host and puts RGB, depth and camera IMU into the same db3. The center port is currently USB2 but delivers measured RGB near30 FPS with the reduced-depth profile. Hardware timestamp metadata/IMU factory-calibration warnings remain documented in the NativeV5 guide.

### Start all Jetsons from the Pi

After reboot, run on Pi: `bash ~/start_jetsons.sh`. This runs the existing `start_13.sh`, `start_14.sh`, and `start_15.sh` on their corresponding controllers over SSH, without enabling Pi control or movement. Logs are saved under `~/MA_Raihan/logs/jetson_startup/`. Use `bash ~/start_jetsons.sh --check` to verify access without starting nodes. Start Pi separately with `bash ~/start_pi.sh --control` and answer the LiDAR prompt (or pass `--lidar` / `--no-lidar`). RealSense remains managed by laptop V5 with `--include-realsense`.
