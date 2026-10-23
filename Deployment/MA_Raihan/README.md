# MA_Raihan — portable Go1 acquisition (23 October 2026)

Each controller has its own `~/MA_Raihan`. All previous packages and startup files are retained. This directory contains copied driver/SDK sources, message definitions, startup/stop scripts, factory calibration exports, dependency inventories, logs and build files. Laptop recorder: `NavigationScripts/nav_and_collect_v4.py`; v2/v3 were not changed.

## Start after boot / battery replacement

On the Pi (`ssh pi@192.168.123.161`):

```bash
bash ~/MA_Raihan/start.sh
```

On each Jetson (`ssh unitree@192.168.123.13`, `.14`, `.15`):

```bash
bash ~/MA_Raihan/start.sh --takeover
```

`--takeover` stops only old camera/point-cloud/JPEG processes that occupy the cameras. Exact previous commands are saved in `inventory/previous_cameras.json`. It does not stop the vendor sport service or change robot mode. Each camera is opened once, and the new process produces perspective video, native depth and point clouds together. The script also starts rosbridge on 9090, creating a local ROS master if needed. Check `logs` for errors; startup text alone does not confirm functioning streams.

Default Pi acquisition is **passive**: it observes existing MCU packets and decodes them with the bundled SDK via loopback-only UDP. It does not call SDK Send, emit a robot command or change robot mode. The decoder uses local-only ports 18008/18009. No low-level motor controller is started. sudo/CAP_NET_RAW is required for packet observation.

### Stop and return to previous camera setup (one command per controller)

```bash
bash ~/MA_Raihan/stop.sh --restore
```

Only MA_Raihan processes are stopped; saved camera commands are restored. No files are deleted. ROS masters are left running. Original start scripts/workspaces remain available. The separate earlier Pi-driver undo remains `bash ~/driverupdatesbyraihan/undo.sh` if that older update also needs reverting.

## Laptop recording

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ROS/Go1_Project/NavigationScripts
python3 nav_and_collect_v4.py --record-only --duration 20
```

v4 is stationary by default. It requires actual messages from rectified cameras and stamped MCU motor/IMU/battery/low-state streams before declaring readiness. It rejects conversion errors, queue drops, stale required streams and camera delivery delays over 1 second. Source timestamps are preserved in the bag; `capture_report.json`, `clock_reference.json` and `timing.jsonl` describe the run.

### Intentional navigation, only when the robot is prepared for control

The passive capture node does not provide a command service. On the Pi, build the copied control driver once (`bash ~/MA_Raihan/scripts/build.sh`) and then deliberately enable control:

```bash
bash ~/MA_Raihan/start.sh --control
```

This starts the existing copied high-level control driver on local port 8091. It initializes walking mode and may step; this command was NOT run during setup/testing. Do not run a second legacy base driver at the same time. From the laptop, navigation is explicitly requested with:

```bash
python3 nav_and_collect_v4.py --navigate --distance 2 --speed 0.2
```

The passive node additionally records complete stamped HighState and odometry once high-level feedback becomes available. MCU measurements are still obtained independently of the control client. The navigation algorithm is inherited from v3; this deployment was tested stationary only.

## Recorded measurements and names

| Data | Topic / location | Profile |
|---|---|---|
| Normal perspective video | `/ma_raihan/cameras/{front,left,right,rear_down}/image_rect/compressed` | ~20 FPS in verified run; 464x400 JPEG, quality 80, 90-degree horizontal FOV |
| Perspective intrinsics | same camera `/camera_info` | Same header/time as video; zero distortion after remapping |
| Factory calibration + metadata | same camera `/factory_calibration`, `/metadata` | Latched and saved per camera; both lens calibrations retained |
| Sparse stereo Z-depth | same camera `/depth/image_native/compressed` | 5 Hz PNG; decoded uint16 millimetres; zero/invalid values retained |
| Native XYZ point cloud | same camera `/points_native` | 1 Hz, full native grid, SDK metre coordinates |
| Measured joint angles/velocities/estimated torques | `/ma_raihan/robot/joint_states_measured` | ~50 Hz; 12 motors; rad, rad/s, Nm |
| Detailed motor state | `/ma_raihan/robot/motor_states` | ~25 Hz; named 12 motors, mode, q/dq/ddq, estimated torque, raw fields, temperature, reserve |
| Complete MCU state | `/ma_raihan/robot/low_state` | ~10 Hz; all 20 SDK slots, IMU including temperature, BMS, foot-force values, hardware tick, remote bytes, IDs/version/CRC; eight extra slots are not extra actuated joints |
| Exact sampled MCU wire packet | `/ma_raihan/robot/raw_low_state` | ~50 Hz; preserves wire data for offline decoding |
| IMU | `/ma_raihan/robot/imu` | ~50 Hz |
| Battery | `/ma_raihan/robot/battery` | ~50 Hz; fraction, V, A; unavailable Ah/capacity values are NaN |
| Individual joint temperatures | `/ma_raihan/robot/temperature/{FR,FL,RR,RL}_{hip,thigh,calf}_joint` | 1 Hz, degrees Celsius |
| CPU/GPU/board thermal zones | `/ma_raihan/controllers/{pi,13,14,15}/temperature/*` | 1 Hz, only zones the current OS exposes |
| Optional high-level estimates | `/ma_raihan/robot/high_state`, `/odom`, `/raw_high_state` | Only advertised when high-level packets are actually observed |

Full-state snapshots include BMS temperatures/cell data and raw foot-force fields. Foot-force units follow the SDK and are not asserted to be calibrated Newtons. Motor current, true shaft torque, and other measurements not exposed by this firmware are not invented. Estimated torque is not an independent torque sensor.

Default video is one corrected left-lens view per physical stereo module. Both lenses are used internally for depth. An optional `_publish_stereo:=true` camera parameter publishes the corrected right lens too, at additional CPU/network cost; in testing, full stereo reduced dual-controller video FPS. Raw fisheye images are not selected by v4.

Depth uses a native SDK projection and is **not registered to the perspective RGB image**. Its frame ID differs intentionally. Native depth/cloud grids are 116x100 on dual-camera .14 and 232x200 on .13/.15. Save calibration and reproject if pixel-aligned depth is required. Invalid depth/cloud pixels remain invalid.

### Synchronization limits

Video/depth/cloud headers use the SDK's acquisition epoch time. Motor/MCU headers use Pi packet-receipt time; MCU tick is also preserved. This improves association over laptop delivery time, but is not a shared hardware trigger or proof of exact encoder/exposure simultaneity. Controllers must share a clock reference; the existing Jetson NTP configuration points to the Pi. Pi local/orphan reference is not independently verified UTC. Laptop receipt times are corrected by measured Pi-minus-laptop offset. Mixed sensor rates are expected; adding their Hz does not measure synchronization. Zero latency cannot be guaranteed.

## Moving to another Go1

Copy the matching controller's complete folder to the same role on the other robot, for example from the laptop:

```bash
scp -r pi@192.168.123.161:MA_Raihan ./MA_Raihan_pi
# After connecting to the replacement robot:
scp -r ./MA_Raihan_pi pi@192.168.123.161:MA_Raihan
```

Use separate archives for Pi, .13, .14 and .15. `role` identifies the controller. Do not mix the Pi's SDK/libraries with a Jetson's. ROS 1 Melodic, architecture-compatible system libraries and Python dependencies are prerequisites; their installed versions are listed in `inventory/packages.txt`. Source copies cannot replace an OS installation or guarantee compatibility with a different Go1 variant/firmware.

Run `bash ~/MA_Raihan/scripts/build.sh` after transfer; never rely on copied catkin build caches, which contain the old machine's paths. Native camera SDK libraries are architecture-specific. Each camera reads its **own factory calibration from flash on startup** and overwrites its MA_Raihan calibration export, so another robot's saved intrinsics are not silently reused. No camera firmware/calibration flash is written. Verify camera position IDs, number of detected cameras, clock alignment and a stationary bag on the replacement robot before navigation.

rosbridge source packages are on the Pi in `ros_ws/src`; Jetsons retain their working system bridge and include source copies under `extras/rosbridge_source` for rebuilding on another installation. If the replacement Jetson lacks rosbridge, copy these into the new workspace's `src` and run the build script.

## Optional external sensors (not physically connected during this deployment)

RealSense source packages are in .14 `extras`; RoboSense SDK sources are in Pi `extras`. Use `scripts/prepare_optional.sh` to build separate optional workspaces, and `scripts/start_optional.sh realsense|lidar` to launch explicitly. RealSense also needs compatible librealsense system libraries; LiDAR needs a matching IP/interface configuration. External sensors are not silently marked as captured while absent.

The current robot exposes four V4L2 stereo modules: front, left, right, rear down. A separate chin device was not detected today. The camera node labels devices from factory position IDs rather than assuming `/dev/videoN` is always the same camera.

## Legacy entry points and local HardwareScripts copies

Home `start_pi.sh`, `start_13.sh`, `start_14.sh`, `start_15.sh` and `stop_nodes.sh` now delegate to MA_Raihan. The matching files under laptop `HardwareScripts/` are identical. Originals are preserved under each controller's `MA_Raihan/inventory/legacy_scripts_before_20261023/` and laptop `HardwareScripts/backups_MA_20261023/`.

`bash ~/start_pi.sh` starts passive telemetry/bridge; `bash ~/start_pi.sh --control` explicitly enables the copied high-level controller. Jetson start entry points include camera takeover. `bash ~/stop_nodes.sh` stops only MA_Raihan nodes; add `--restore` to restore the saved factory camera processes.

One-command restoration of previous home startup files on each controller:

```bash
bash ~/MA_Raihan/undo_startup_scripts.sh
```

### Depth provenance

On this firmware, SDK getDepthFrame produced quantized 0/1 images. The delivered depth channel instead projects the SDK floating-point XYZ measurements onto a defined perspective grid using a nearest-Z buffer and publishes a lossless uint16 PNG in millimetres. `depth/camera_info` supplies that grid's intrinsics. Holes are zero; no interpolation invents missing depth. This is a sparse derived product, not a dense depth sensor or a depth image registered to the RGB lens. The native XYZ cloud is retained without decimation within each published cloud.


### Navigation recording correction — 2026-10-23

v4 now records on an independent writer thread, so walking-mode service calls and navigation sleeps do not pause bag processing. Worker errors still abort navigation, and shutdown joins the writer before finalization. Startup waits for fresh samples; the runtime 500 ms telemetry limit remains active. The Pi publishes complete HighState and redundant raw-packet snapshots at 10 Hz; measured motor/IMU publication rates are unchanged. Pi rosbridge subscriptions request TCP_NODELAY. Rollback on the Pi: `bash ~/MA_Raihan/updates/highstate_20261023/undo.sh`.

Validation used all 73 real topics with a simulated navigation loop and a three-second service delay; all robot actions were replaced by no-ops. The run wrote 6,720 messages with no conversion failures or local queue drops. Actual movement remains user-tested. See `navigation_writer_verification.json`.


### Final movement verification — 2026-10-23, 20:27

The approved 0.2 m real movement test reached 0.18 m odometry (the inherited 2 cm stopping tolerance), then issued stop commands. All 73 topics delivered messages: 4,186 messages, no conversion failures, local queue drops or unprocessed messages. Status complete; see movement_verification.json. The test had a three-second host limit and a temporary Pi stop watchdog. This validates a short movement test, not arbitrary distances or speeds.

Final v4 uses the recorder's single /ma_raihan/robot/odom subscription for navigation, requires its freshness, and rechecks readiness after the walking-mode service. No second /go1_controller/odom subscription is added. Recording runs on its independent thread.

The final Pi profile preserves all fields but reduces sampling rates to avoid rosbridge backlog: IMU/measured joints/battery nominal 25 Hz, motor detail nominal 10 Hz (measured 8.3), full High/Low snapshots and raw packets nominal 5 Hz, odometry measured 12.4 Hz. Motor/IMU/odometry peak receipt ages were below 79 ms in this test. Camera settings were unchanged; measured front 9.6 FPS, left 19.4, right 24.9, rear-down 20.2. Front camera FPS remains below the earlier 20 FPS stationary result and needs follow-up. This final profile supersedes earlier rate descriptions above. The existing Pi rollback restores all telemetry source, binary and rosbridge changes.
