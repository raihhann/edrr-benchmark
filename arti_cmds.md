# Go1 startup and synchronized acquisition — MA_Raihan

Updated 23 October 2026. The previous guide is preserved as `cmds.before_MA_Raihan_20261023.md`. Full package/data descriptions are in `Deployment/MA_Raihan/README.md` and `~/MA_Raihan/README.md` on each controller.

## 1. Network

If directly connected by Ethernet and the robot subnet is reachable, no extra route is needed. With a Pi Wi-Fi gateway, read its current address on the Pi:

```bash
ip -4 addr show wlan0
```

Then on the laptop, replacing the placeholder:

```bash
sudo ip route replace 192.168.123.0/24 via <PI_WIFI_IP>
```

## 2. Start passive robot telemetry and rosbridge on the Pi

```bash
ssh pi@192.168.123.161
bash ~/MA_Raihan/start.sh
```

This does not enable motor control. It passively decodes the existing MCU feedback stream. Detailed motor state, 12 joint temperatures, measured joint angles/speeds/estimated torques, IMU, battery, foot-force fields, BMS temperatures, hardware tick and raw MCU packets are available under `/ma_raihan/robot/`. Board temperatures are under `/ma_raihan/controllers/`.

## 3. Start camera acquisition and rosbridge on each Jetson

Run in separate terminals:

```bash
ssh unitree@192.168.123.13
bash ~/MA_Raihan/start.sh --takeover
```

```bash
ssh unitree@192.168.123.14
bash ~/MA_Raihan/start.sh --takeover
```

```bash
ssh unitree@192.168.123.15
bash ~/MA_Raihan/start.sh --takeover
```

These scripts start local ROS masters if necessary and rosbridge on port 9090. Old camera processes are stopped to release the devices; their commands are saved for restoration. No original files are removed or edited. Camera identity and calibration come from the hardware, not assumed video device numbers.

Default normal-view topics:

```text
/ma_raihan/cameras/front/image_rect/compressed
/ma_raihan/cameras/left/image_rect/compressed
/ma_raihan/cameras/right/image_rect/compressed
/ma_raihan/cameras/rear_down/image_rect/compressed
```

Each camera also publishes `camera_info`, `factory_calibration`, `metadata`, `depth/image_native/compressed`, `points_native` and diagnostics. Video is a 464x400 perspective view, not a two-lens fisheye image. Native depth is not pixel-registered to perspective RGB; keep its metadata/calibration. A chin device was not detected on this robot today.

Check a controller's topics:

```bash
source ~/MA_Raihan/ros_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rostopic list
```

On a Jetson, replace `left` with its camera position:

```bash
rostopic hz /ma_raihan/cameras/left/image_rect/compressed
```

## 4. Laptop: stationary validation first

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ROS/Go1_Project/NavigationScripts
python3 nav_and_collect_v4.py --record-only --duration 20
```

No RealSense/LiDAR flags are needed while those devices are absent. v4 selects the new rectified camera streams and stamped telemetry, excluding legacy fisheye topics and stale legacy point-cloud registrations. It is stationary by default; motion requires `--navigate`. Custom ROS 1 messages are stored as JSON in ROS 2 String messages with source headers preserved. The bag timestamp is the source header time.

Read `capture_report.json`, `clock_reference.json` and `timing.jsonl` in the resulting bag. A complete report confirms all selected streams delivered data, not that unsupported/absent hardware was present. Different sensor rates are expected; their summed Hz is not a synchronization metric.

## 5. Optional intentional navigation

Only when the robot is prepared for control, enable the copied base driver on the Pi:

```bash
bash ~/MA_Raihan/start.sh --control
```

This driver uses local UDP port 8091 to avoid the vendor programming service on 8090. It initializes walking mode and may step. Do not launch it alongside another base driver. It was built but was not activated during this sensor setup.

On the laptop:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ROS/Go1_Project/NavigationScripts
python3 nav_and_collect_v4.py --navigate --distance 2 --speed 0.2
```

Speed is metres per second; distance is metres. v2 and v3 remain unchanged. Once high-level feedback is active, the passive recorder additionally exposes full stamped HighState and odometry.

## 6. Stop / undo the new acquisition setup

On each controller:

```bash
bash ~/MA_Raihan/stop.sh --restore
```

This stops only the new nodes and restores previously running camera commands. It deletes no files and leaves ROS masters running. To stop without restoring old cameras, omit `--restore`.

The older driver-source update has its own independent rollback on the Pi:

```bash
bash ~/driverupdatesbyraihan/undo.sh
```

## 7. Battery replacement

Stop the laptop recording with Ctrl+C and let the bag finalize. Stop MA_Raihan on each controller before powering down if possible. Files and builds persist. After reconnecting with the replacement battery, repeat sections 2–4; rebuilding is unnecessary on the same controllers. Start `--control` only when intentionally ready for control.

## 8. Move to another Go1

Role-specific source/SDK archives are under laptop `Deployment/MA_Raihan/controllers/`. Transfer the matching archive to the matching controller; each archive contains the top-level `MA_Raihan` directory. Rebuild with:

```bash
bash ~/MA_Raihan/scripts/build.sh
```

The replacement requires compatible ROS 1 Melodic/system libraries and firmware; package-version inventories are included. Build caches are not portable. Check IP roles, per-camera factory calibration, clock synchronization and a stationary recording before navigation. Camera flash is read, never rewritten.

## 9. External sensors for later

RealSense sources are in `.14:~/MA_Raihan/extras/`; RoboSense sources are in `Pi:~/MA_Raihan/extras/`. They were not connected and were not tested during this setup.

Build on the relevant controller after compatible system dependencies are installed:

```bash
bash ~/MA_Raihan/scripts/prepare_optional.sh
```

Launch explicitly on .14:

```bash
bash ~/MA_Raihan/scripts/start_optional.sh realsense
```

Or on the Pi, after checking the IP/subnet against `extras/rslidar_sdk/config/config.yaml`:

```bash
bash ~/MA_Raihan/scripts/start_optional.sh lidar
```

Add `--include-realsense` and/or `--include-lidar` to the laptop v4 command. Their additional bandwidth and timing must be measured when connected; the built-in-only results do not validate them.

## Timing and data limits

Camera headers preserve SDK acquisition time. MCU/motor headers use Pi packet-receipt time and retain the MCU hardware tick; they are not proven simultaneous with camera exposures. The existing Jetson clock service references the Pi; the Pi local reference is not independently verified UTC. Laptop receipt times are corrected by the measured Pi offset. v4 rejects camera delays over 1 second, required robot packet delays over 500 ms, conversion errors and local queue drops. Zero latency is not guaranteed.

Measured built-in profile: roughly 20 FPS per normal-view camera, 5 Hz depth, 1 Hz native point clouds, 50 Hz measured joints/IMU/battery/raw MCU, 25 Hz detailed motor messages, 10 Hz full MCU state, 1 Hz individual motor/controller temperatures. Raw data preserves additional SDK fields; unavailable sensors or firmware registers are not fabricated.

## Legacy entry points and local HardwareScripts copies

Home `start_pi.sh`, `start_13.sh`, `start_14.sh`, `start_15.sh` and `stop_nodes.sh` now delegate to MA_Raihan. The matching files under laptop `HardwareScripts/` are identical. Originals are preserved under each controller's `MA_Raihan/inventory/legacy_scripts_before_20261023/` and laptop `HardwareScripts/backups_MA_20261023/`.

`bash ~/start_pi.sh` starts passive telemetry/bridge; `bash ~/start_pi.sh --control` explicitly enables the copied high-level controller. Jetson start entry points include camera takeover. `bash ~/stop_nodes.sh` stops only MA_Raihan nodes; add `--restore` to restore the saved factory camera processes.

One-command restoration of previous home startup files on each controller:

```bash
bash ~/MA_Raihan/undo_startup_scripts.sh
```

### Depth provenance

On this firmware, SDK getDepthFrame produced quantized 0/1 images. The delivered depth channel instead projects the SDK floating-point XYZ measurements onto a defined perspective grid using a nearest-Z buffer and publishes a lossless uint16 PNG in millimetres. `depth/camera_info` supplies that grid's intrinsics. Holes are zero; no interpolation invents missing depth. This is a sparse derived product, not a dense depth sensor or a depth image registered to the RGB lens. The native XYZ cloud is retained without decimation within each published cloud.
