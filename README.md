# TurtleBot3 Waffle Pi — navigation and sensor recording

Laptop tools and robot-side ROS 2 scripts for straight-line navigation, keyboard driving, native bag recording, video extraction and offline bag analysis.

## Current setup

| Component | Configuration |
|---|---|
| Robot | TurtleBot3 Waffle Pi, Raspberry Pi, OpenCR |
| Robot operating system / ROS | Ubuntu 22.04 / ROS 2 Humble |
| Laptop ROS | ROS 2 Jazzy |
| Robot SSH | `ubuntu@10.130.58.6` |
| Robot project directory | `~/MA_Raihan` |
| Laptop project directory | `/home/fsti/ROS/TurtleBot` |
| Attached sensors | LDS-02 LiDAR, IMX219 Pi camera, base IMU/magnetometer, wheel encoders and battery telemetry |

Edit [Config/connection.json](Config/connection.json) when the robot IP or SSH username changes. Passwords are requested by SSH and are not stored in this project.

The laptop starts commands through SSH. Navigation and `ros2 bag record` execute on the Pi; finalized bags are copied to the laptop and checked with SHA-256. **Rosbridge is not used.** ROS networking stays local to the robot, so laptop Jazzy and robot Humble do not need to communicate through DDS.

## Project layout

```text
Config/                 SSH connection configuration
HardwareScripts/        Laptop start/stop wrappers
NavigationScripts/      Distance navigation and keyboard driving
DataCollection/         Recording, download, audits, video and insights tools
Robot/                  Source files deployed under ~/MA_Raihan on the Pi
Interfaces/             Matching TurtleBot custom message definitions
Tests/                  Navigation math and isolated command-guard tests
Documentation/          Experiment metadata reference
Deployment/             Setup evidence and historical setup helpers
Bags/                   Downloaded datasets; excluded from Git
.runtime/               Temporary SSH sockets; excluded from Git
```

## Requirements and deployment

For laptop launch, recording and navigation wrappers: Python 3, OpenSSH (`ssh`/`scp`), network access to the Pi and a configured robot installation. Sourcing laptop ROS is unnecessary for these commands.

For video extraction: source Jazzy, with `rclpy`, `sensor_msgs`, NumPy, OpenCV Python and FFmpeg available. The terminal and HTML insight scripts use only the Python standard library.

The configured robot requires its existing Humble TurtleBot workspace at `~/turtlebot3_ws`, working OpenCR firmware, the LDS-02 driver, and the private camera runtime under `~/MA_Raihan/Dependencies/root`. The default bringup explicitly uses the original TurtleBot base driver in that existing workspace.

To copy this repository's robot source into a prepared robot installation, run on the laptop:

```bash
cd /home/fsti/ROS/TurtleBot
ssh ubuntu@10.130.58.6 'mkdir -p ~/MA_Raihan'
rsync -av --exclude='__pycache__/' Robot/ ubuntu@10.130.58.6:MA_Raihan/
ssh ubuntu@10.130.58.6 'bash ~/MA_Raihan/build.sh'
```

This copy does not delete existing robot files. The build compiles only `ma_turtlebot_bringup`; it excludes the experimental base-driver extension. **The repository is a source project, not a complete robot image:** private camera binaries and existing TurtleBot system/workspace dependencies are not bundled in Git. A replacement robot needs compatible dependencies and hardware configuration before this deployment can run.

## 1. Start the robot nodes

Run on the laptop:

```bash
cd /home/fsti/ROS/TurtleBot
python3 HardwareScripts/start_robot.py --lidar
```

This starts the base, robot state publisher, command watchdog, LDS-02 LiDAR and prepared Pi camera driver. It does not start navigation or recording. The wrapper runs in the background; robot logs are under `~/MA_Raihan/Logs/`.

Alternatively, on the Pi:

```bash
bash ~/MA_Raihan/start.sh --lidar
```

For foreground native ROS launch on the Pi:

```bash
source ~/MA_Raihan/env.sh
ros2 launch ma_turtlebot_bringup robot.launch.py lidar:=true camera:=true
```

Use the foreground launch or background wrapper, not both. The Pi wrapper also accepts `--no-camera` and `--no-lidar`; stop existing nodes before changing startup options.

## 2. Navigate and record

Prepare clear floor space and remain ready to stop the robot. Run on the laptop:

```bash
cd /home/fsti/ROS/TurtleBot
python3 NavigationScripts/nav_and_collect_v2.py \
  --navigate --distance 1 --speed 0.1 \
  --bag-name walk_01 --audit
```

- `--distance`: forward distance in metres, greater than 0 and at most 20.
- `--speed`: requested speed in metres/second, from 0.02 to **0.26**.
- `--bag-name`: optional unique name; omit for a timestamp and random suffix.
- `--audit`: validate the finalized bag on the robot before downloading it.
- `--no-fetch`: retain the bag on the Pi and download it later.
- `--require-topic /exact/topic`: require an additional stream at startup; repeat as needed.

Without `--navigate`, this script performs stationary recording. Automatic distance navigation currently always records a bag.

Navigation uses wheel odometry, acceleration/braking, heading correction, fresh-data checks and a forward LiDAR clearance check. It is a straight-line acquisition controller, **not a Nav2 route planner**. Odometry distance is an estimate; wheel slip and stopping error remain possible.

The configured translational maximum is **0.26 m/s**, matching the [ROBOTIS Waffle Pi specification](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/). The command guard retains a **0.8 rad/s** turn limit. Defaults remain lower; navigation does not instantly jump to maximum speed.

## 3. Keyboard driving without recording

```bash
cd /home/fsti/ROS/TurtleBot
python3 NavigationScripts/teleop_v2.py --speed 0.15 --turn-speed 0.5
```

| Key | Action |
|---|---|
| W / S | Forward / backward |
| A / D | Turn left / right |
| Space | Stop |
| X | Exit |

`--speed` defaults to 0.08 m/s and permits up to 0.26 m/s. `--turn-speed` defaults to 0.35 rad/s and permits up to 0.8 rad/s. Key input expires after 0.2 seconds; keyboard repeat affects responsiveness. This tool does not create a bag. Do not run it alongside automatic navigation.

The separate robot watchdog accepts fresh stamped commands on `/ma_raihan/nav_cmd_vel` and publishes to `/ma_raihan/driver_cmd_vel`. It sends zero when commands expire after approximately 0.3 seconds. Our base driver is remapped to that guarded output.

## 4. Record without moving

```bash
cd /home/fsti/ROS/TurtleBot
python3 DataCollection/record_bag.py \
  --duration 20 --bag-name stationary_01 --audit
```

The recorder discovers active topics, including camera, IMU, magnetometer, odometry, wheel joints, battery, sensor state, laser scans, TF, diagnostics and command streams. Hidden topics are included. Preflight requires actual messages on required streams before navigation starts.

A completed run is stored at both:

```text
Robot:  ~/MA_Raihan/Bags/<bag-name>/
Laptop: /home/fsti/ROS/TurtleBot/Bags/<bag-name>/
```

Wait for `Verified laptop bag: ...` before assuming download is complete. Large image bags can take time to copy. The download does not delete the robot copy.

To fetch a bag saved using `--no-fetch`, or retry an incomplete transfer:

```bash
python3 DataCollection/fetch_bag.py stationary_01
```

Existing finalized local bags are not overwritten. A bag contains the `.db3` database, `metadata.yaml`, `experiment.json`, and, when requested before download, `audit.json`. See [Documentation/EXPERIMENT_FIELDS.md](Documentation/EXPERIMENT_FIELDS.md) for experiment fields.

## 5. Inspect a downloaded bag in the terminal

No ROS environment or robot connection is needed:

```bash
python3 /home/fsti/ROS/TurtleBot/DataCollection/bag_insights.py \
  /home/fsti/ROS/TurtleBot/Bags/walk_01
```

This prints message counts, sensor rates/FPS, source and receipt timing, large gaps, camera-to-IMU/odometry/LiDAR timestamp proximity, saved navigation results and warnings. It creates no report files.

For the optional HTML report instead:

```bash
python3 /home/fsti/ROS/TurtleBot/DataCollection/bag_insights_html.py \
  /home/fsti/ROS/TurtleBot/Bags/walk_01
```

That version writes `Insights/insights.html`, `insights.md` and `insights.json` inside the bag folder.

The native payload audit is a separate robot-side operation:

```bash
python3 DataCollection/audit_bag.py walk_01
```

It audits the robot's copy; it does not update an already downloaded laptop audit automatically. Use `--audit` during collection to include the audit in the initial download.

## 6. Extract one MP4 camera video

```bash
source /opt/ros/jazzy/setup.bash
python3 /home/fsti/ROS/TurtleBot/DataCollection/extract_video.py \
  /home/fsti/ROS/TurtleBot/Bags/walk_01
```

Output is under the bag's `ExtractedVideo/` folder:

- `camera_pi_image_raw_compressed.mp4`: one video, using the MPEG-4 codec to avoid the observed H.264 green-screen playback issue.
- `camera_pi_image_raw_compressed_timestamps.csv`: exact source and bag receipt timestamps for each frame.
- `camera_pi_image_raw_compressed_export.json`: export metadata.

The default input is `/camera/pi/image_raw/compressed`; use `--topic /other/compressed/topic` to select another `sensor_msgs/msg/CompressedImage` stream. Existing exports are not overwritten. Video playback uses the median source frame interval; use the CSV for exact alignment or gaps.

## Timing and sensor limitations

- Source headers are preserved. Native bag timestamps describe recorder receipt time, not simultaneous acquisition across sensors.
- Camera, IMU and LiDAR publish at different rates. Nearest timestamps measure sampling proximity; they do not establish hardware synchronization.
- Receipt-minus-source includes pipeline delay and possible clock offset. No zero-latency or loss-free claim is made.
- Pi camera intrinsic calibration is currently absent. CameraInfo calibration matrices may be zero; images are not calibrated/rectified.
- The active base driver provides wheel positions/velocities and sensor state. JointState effort is empty; wheel motor temperature is not exposed by this profile.
- The experimental raw-register/current driver under `Robot/Packages/turtlebot3_node` is retained for diagnosis but is disabled and excluded from the default build. Do not treat it as a validated data source.
- Disabled/uninstalled bumper, IR, sonar and light channels must not be treated as measured zeros just because SensorState has fields for them.
- RealSense is not connected or prepared. Its helper checks for a compatible installed driver. A future RealSense or different LiDAR requires driver, power, bandwidth and frame/extrinsic checks; active topics can then be recorded with the same recorder.

## Verification

The approved initial movement test completed at approximately 0.291 m by odometry for a 0.3 m target. The subsequent `walk_01` user run reported 0.990 m forward progress for a 1 m target; its native audit passed with 4,145 messages across 19 topics and a final zero driver command. This is evidence for those datasets, not a guarantee for future runs.

Eleven offline navigation tests and an isolated watchdog test passed after the 0.26 m/s update. Isolated tests use separate ROS domain/topic names; no physical maximum-speed movement test was performed.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s Tests -v
```

## Stop and rollback

```bash
python3 HardwareScripts/stop_robot.py
```

Or on the Pi:

```bash
bash ~/MA_Raihan/stop.sh
```

These stop our scoped processes. Existing TurtleBot workspaces and firmware are preserved. For the deployed speed-limit change, a robot-side backup can be restored with:

```bash
bash ~/MA_Raihan/Updates/undo_speed_026.sh
```

That rollback helper belongs to the deployed robot and restores its earlier files; laptop arguments would also need to respect the restored 0.15 m/s limit.

## Publish the source on the `turtlebot` Git branch

The laptop folder currently has no Git repository. These commands initialize it as a standalone `turtlebot` branch and push that branch to the existing repository. They do not force-push or modify other branches:

```bash
cd /home/fsti/ROS/TurtleBot

git init -b turtlebot
git remote add origin https://github.com/raihhann/edrr-benchmark.git

git add .
git status --short
git diff --cached --stat

git commit -m "Add TurtleBot Waffle Pi navigation and sensor collection"
git push -u origin turtlebot
```

This creates a branch with its own root history, rather than inheriting the Go1 branch. If GitHub already has a `turtlebot` branch, stop and reconcile it before pushing; do not force-push over it. GitHub authentication may require a configured credential helper or personal access token.

The `.gitignore` excludes recordings, extracted videos, SSH sockets, downloaded binary dependencies, build outputs and machine-specific setup evidence. Review `git status` before committing. Dataset files stay local.
