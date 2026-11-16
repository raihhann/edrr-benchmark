# Native V5 recording and navigation

V5 records original ROS1 messages locally on each controller using native rosbag. After recording, the laptop downloads the four source recordings, verifies SHA256 hashes, merges every message into one combined.bag, verifies the merged payloads/timestamps, and writes quality_report.md and quality_report.json. V4 is preserved. No laptop ROS setup is required for V5.

## Startup after powering on

On Pi (192.168.123.161), stationary collection:

```bash
bash ~/start_pi.sh
```

It asks whether LiDAR is connected. Explicit alternatives are `--lidar` and `--no-lidar`. For navigation, enable the control driver explicitly:

```bash
bash ~/start_pi.sh --control --lidar
```

On the corresponding Jetsons:

```bash
# .13
bash ~/start_13.sh
# .14
bash ~/start_14.sh
# .15
bash ~/start_15.sh
```

The wrappers start the existing sensor/rosbridge stack and camera recovery. Native recorders run only during a V5 session. RealSense needs its existing driver started separately; --include-realsense verifies it is producing data, rather than installing or launching it.

## Laptop commands

From /home/fsti/ROS/Go1_Project:

```bash
# Stationary, LiDAR required, chin explicitly optional while its SDK issue remains
python3 NavigationScripts/nav_and_collect_v5.py --record-only --duration 15 --include-lidar --optional-camera chin

# Navigation: Pi control driver must already be enabled; clear path and remote ready
python3 NavigationScripts/nav_and_collect_v5.py --navigate --distance 1 --speed 0.2 --include-lidar --optional-camera chin
```

Omitting --navigate defaults to stationary collection. Speed is m/s, distance is metres. Software speed limit is 0.5 m/s. --bag-name NAME gives a custom name; otherwise a timestamp/random name is generated. --start-nodes optionally starts the controller wrappers too; with --navigate it explicitly requests Pi control mode.

All published topics are selected at their original published rates by default, without sampling or throttling. --include-lidar / --include-realsense additionally require those devices to produce data. --skip-lidar / --skip-realsense explicitly exclude their topics. --optional-camera chin permits its absence but still records it if it publishes. None of these flags expose hardware data that the ROS drivers never publish.

## Output and recovery

Default laptop output: Bags/go1_bags/RUN_ID/. Each role folder pi/, 13/, 14/, 15/ contains capture.bag (native ROS1), audit and run metadata. experiment.json and quality_report.md/json describe the combined run. Use combined_ros2/combined_ros2.db3 and its metadata.yaml as the final all-controller ROS2 dataset; combined.bag remains the native ROS1 backup. The four source bags remain for recovery and quality checks. combined.bag is native ROS1, not ROS2 SQLite; ROS2-only extraction tools need conversion or ROS1 support. Controller capture.bag files are removed after verified download and ROS2 conversion unless --keep-remote is used; controller metadata and local backups remain.

```bash
# Retry download/audit after an interrupted transfer
python3 NavigationScripts/nav_and_collect_v5.py --fetch-only RUN_ID
# Recheck already downloaded files
python3 NavigationScripts/nav_and_collect_v5.py --audit-only /absolute/path/to/RUN_ID
```

Quality checks include file integrity, payload decoding, all 12 measured motors, source sequence gaps, actual rates, timestamp age, camera/IMU nearest timestamps and relative controller clock estimates. Default limits: cameras >=15 FPS, sequence loss <=1%, required stream gaps <=600 ms, source age p95 <=250 ms and max <=1000 ms, nearest camera/IMU timestamp <=50 ms. --min-camera-fps and --max-loss-percent explicitly choose other acceptance criteria; changing a criterion does not repair a stream. INVALID recommends a rerun after fixing the reported cause. UNVERIFIED means evidence is insufficient. ROS sequence gaps do not measure Ethernet packet loss, and aligned timestamps do not establish hardware-triggered synchronization.

Native recorders require 1.5 GiB free at startup and stop below 1.25 GiB, above rosbag's own 1 GiB cutoff. Jetson .13 had only about 1.1 GB free: transfer and archive data before extended recording; no original data was deleted. Navigation has independent heartbeat, stale-odometry, path and final-zero guards. Navigation is unit-tested but was not physically exercised during this V5 deployment.

## Verified stationary test and remaining issues

20261113_201106_go1_native_2b7e8f: four bags downloaded and SHA256-verified, 4234 messages, zero observed sensor sequence gaps and no payload decoding failures. LiDAR ~10 Hz; IMU ~24.7 Hz. Cameras: front 13.7, left 28.1, right 20.0, rear_down 25.2 FPS. Chin produced no frames and was explicitly optional. The run is INVALID against the default 15 FPS minimum because front is below it. The transfer/recording workflow passed; a complete all-camera dataset remains unresolved.

The robot battery died during the final incremental source sync. The tested deployment was installed earlier; final runtime revisions are not confirmed on every controller. After powering on, finish and verify that update before collecting:

```bash
python3 /home/fsti/ROS/Go1_Project/Deployment/MA_Raihan/NativeV5/finish_controller_update.py
```

This updates only the native helper files, verifies their hashes, and does not start nodes or move the robot. Enter SSH passwords when asked. The NativeV5 directory contains current sources; older role deployment tar archives have not been repacked for this update.

## Undo

On each controller, one command restores the backed-up scripts and camera source/binary:

```bash
bash ~/MA_Raihan/updates/native_v5_20261113_200031/undo.sh
```

Restart nodes normally afterward to load restored executables. New captures and helper files are retained. Laptop rollback:

```bash
bash /home/fsti/ROS/Go1_Project/HardwareScripts/backups/native_v5_20261113_200031/undo_local.sh
```

## Single combined bag (default)

Every V5 download now automatically produces `RUN_ID/combined.bag` and `merge_report.json`. The merge retains all original serialized messages, message definitions, headers and native recorder timestamps, ordered by recorded time. It does not resample, deduplicate or correct clocks. Verification rereads the result and compares per-connection payload/timestamp hashes and topic counts. Source bags are kept for recovery.

Merge an existing downloaded run without starting nodes, recording or navigation:

```bash
python3 NavigationScripts/nav_and_collect_v5.py --merge-only /absolute/path/to/RUN_ID
```

The isolated laptop dependency is in `.venv-native-bags`; when copying this project to another laptop, create that venv and install `rosbags==0.11.9`. Writer API documentation: https://ternaris.gitlab.io/rosbags/api/rosbags.rosbag1.html .

## ROS2 SQLite output and automatic remote cleanup (2026-11-16)

V5 now exports `RUN_ID/combined_ros2/combined_ros2.db3` plus `metadata.yaml` automatically after the verified native merge. `db3_report.json` records checksums and verification of every converted message, timestamp, topic count and SQLite integrity. The dataset includes all selected topics and their custom message definitions. ROS2 Header has no ROS1 `seq` field: original sequence values remain in the laptop native bags and sequence audit. Custom ROS2 interfaces may still be needed by playback subscribers.

Only after complete downloads and successful conversion, each controller's exact `~/MA_Raihan/captures/RUN_ID/capture.bag` is checksum-verified again and deleted. It is never deleted while an active recorder/file is detected, on checksum mismatch or if conversion fails. Local source bags, merged ROS1 backup, metadata and diagnostics remain. `--keep-remote` disables this automatic deletion; `--no-fetch` also leaves recordings on the robot. Closed partial experiments are exported and cleaned after integrity verification but remain INVALID in the quality report. Cleanup does not upgrade experimental validity. `remote_cleanup.json` records what was removed or retained.

Local run folders: `/home/fsti/ROS/Go1_Project/Bags/go1_bags/RUN_ID/`. Controller run folders: `/home/pi/MA_Raihan/captures/RUN_ID/` on Pi; `/home/unitree/MA_Raihan/captures/RUN_ID/` on Jetsons. Older runs are not purged automatically. The experiment's original `run.json` and events remain with these folders.

Native recorder startup now requires >=1.5 GiB free and stops below 1.25 GiB, above rosbag's own 1 GiB cutoff. Reclaim space from older verified recordings before retrying a controller that cannot pass this check.

Existing downloaded runs can be converted without contacting the robot using `--merge-only /absolute/path/to/RUN_ID`. To finish downloads/export and clean a specific already-collected run on controllers, use `--fetch-only RUN_ID`; existing local copies are verified and reused, so remote audit is not needed after their capture.bag has been deleted.

## RealSense D455 on Pi (2026-11-16)

Physical attachment was moved from Jetson .14 to Pi .161. The Pi reuses its existing `~/d455_ros_ws` driver (RealSense ROS 2.3.2 / librealsense 2.50.0); no ROS rebuild or firmware update was performed. A managed camera-only launch is stored at `~/MA_Raihan/config/realsense_pi.launch`.

`--include-realsense` in laptop V5 now starts/verifies the camera on the Pi automatically, without starting the robot base or changing walking mode. It requires actual color, depth, calibration and IMU streams before capture/navigation readiness. Pi `start_pi.sh` no longer starts or prompts for RealSense. Use `bash ~/start_pi.sh --control --lidar` for Pi startup. The camera is now attached to Jetson .15; laptop V5 detects its USB host with `--include-realsense` (see the updated attachment section below). To start just the camera manually: `bash ~/MA_Raihan/scripts/start_realsense.sh`.

Requested profiles: RGB 640x480 at 30 FPS; depth 640x480 at 15 FPS; gyro 200 Hz and accel 100 Hz. These are requests, not measured guarantees. Infrared duplicates, derived point clouds and depth-to-color alignment are disabled in this profile. Original raw RGB/depth, camera calibration, extrinsics, diagnostics and camera IMU topics are recorded at their published rates. Duplicate compressed/Theora image transports are disabled before camera startup; Pi native bag chunks use lossless LZ4 when RealSense is required. The final ROS2 db3 still contains raw sensor_msgs/Image streams, not a pre-encoded MP4. The quality report checks RGB rate/timing, depth/IMU readiness and RGB-to-robot-IMU timestamp alignment.

Example after normal robot startup (navigation only with clear path and remote ready):

```bash
python3 NavigationScripts/nav_and_collect_v5.py --navigate --distance 1 --speed 0.2 --include-lidar --include-realsense --optional-camera chin
```

Pi USB currently enumerates through a 480 Mb/s hub. Depth was observed at 15 FPS, but initial USB2 test produced no RGB or camera IMU messages. Do not treat the camera as ready until the helper verifies these streams. A direct USB3 cable/port check has been requested; full capture validation is pending.

Pi rollback: `bash ~/MA_Raihan/updates/realsense_pi_20261116/undo.sh`. This stops only the managed RealSense launch and restores the backed-up startup/runtime files. Local previous V5 files are in `HardwareScripts/backups/realsense_pi_20261116/`.

## Current RealSense location and automatic detection

The Go1 center USB port maps to controller .15. On 2026-11-16, the D455 connected there at 480 Mb/s (USB2), despite the internal host supporting faster USB. A six-second subscriber check measured RGB 640x480 at 29.99 FPS, depth 480x270 at 15.00 FPS, gyro 199.47 Hz and accelerometer 100.65 Hz, without observed sequence gaps. Timestamp ages were about 12 ms p95 for RGB and 2.2 ms for the other streams. The driver reports missing hardware timestamp metadata and unavailable factory IMU calibration, so host-time alignment is not proof of synchronized hardware exposures or calibrated camera IMU accuracy.

V5 `--include-realsense` now automatically detects the USB host across pi/.13/.14/.15, starts its managed camera-only helper and requires actual images/IMU. Override with `--realsense-controller 15` (or another role) when needed. USB2 startup selects `config/realsense_usb2.launch`; USB3 selects `config/realsense.launch`. All hosts have startup helpers, but an existing built RealSense workspace is still required. The normal navigation command can simply add --include-realsense.

Atomic helper repair/deployment command after an interrupted battery update: `python3 Deployment/MA_Raihan/NativeV5/deploy_realsense_support.py`. It verifies SHA256 and fsyncs every file; it does not start nodes or motion. Backups reside under `~/MA_Raihan/updates/realsense_repair_20261116/`.

### RealSense end-to-end verification completed

`Bags/go1_bags/realsense_center_port_verify_20261116/` contains the stationary validation dataset. All four controllers were recorded, downloaded and checksum-verified; the combined ROS2 SQLite bag contains 18,591 messages across 100 topics (639.4 MiB). Corrected native audit retains raw RGB timestamps and permits quiet dynamic `/tf` when latched `/tf_static` was captured. The same recorded source was re-audited without additional motion or capture. Final verdict: VALID_WITH_WARNINGS (chin optional; hardware exposure synchronization and camera IMU calibration not certified). RealSense recorded RGB 29.98 FPS, depth 15.00 FPS, gyro199.44 Hz and accel100.65 Hz with no observed sequence gaps; RGB-to-nearest robot IMU maximum23.29 ms. Remote recording copies were removed after final verification.

Use the usual V5 command with `--include-realsense`; physical host detection is automatic. For passive testing use `--record-only --duration 10 --include-lidar --include-realsense --optional-camera chin`. Navigation still requires an explicitly enabled Pi control driver and a clear path/remote stop ready; no movement was commanded during this validation.

### Recovery from an advertised but stalled RealSense

The driver can remain registered while logging repeated `Frames didn't arrived within 5 seconds`. Startup now verifies at least three seconds of fresh, continuous RGB/depth/gyro/accel samples instead of trusting node presence or a single frame. If that fails, it checks that no native capture is active, then attempts exactly one driver `/camera/realsense2_camera/reset` call and rechecks delivery. The reset targets only the D455; it preserves the ROS master and robot control driver. A failed recovery still prevents recording/navigation. USB2 reliability and the underlying driver/device stall are not certified as permanently fixed by this recovery.

Recovery was tested without motion: RGB30.03 FPS, depth15.01 FPS, gyro199.80 Hz, accel100.64 Hz. Pi `/set_mode` registration and stamped odometry presence were checked without calling the mode service. V5 also checks those prerequisites before starting recordings for a navigation run, so missing `start_pi.sh --control` is reported early.

## Pi command to start all Jetsons

`bash ~/start_jetsons.sh` on Pi runs the existing startup scripts on .13, .14 and .15 over SSH. `--check` verifies access and script syntax without starting nodes. The laptop deployment copy is `start_jetsons.sh`; install it as `~/start_jetsons.sh` and `~/MA_Raihan/scripts/start_jetsons.sh` on Pi. Pi control/LiDAR startup remains separate: `bash ~/start_pi.sh --control` (answer the LiDAR prompt or pass `--lidar` / `--no-lidar`). RealSense is started by laptop V5 using `--include-realsense` on its detected USB host.
