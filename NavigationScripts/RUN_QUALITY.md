# Navigate, collect, and decide whether to repeat a run

`nav_and_collect_v5.py` reuses v4's single stamped odometry subscription and independent bag writer. V2, v3, v4 and controller startup scripts are not changed. Navigation remains opt-in. By default v5 subscribes to all topics discovered on the four controller ROS masters, including ROS logs, rosbridge status and legacy sensor topics. RealSense and LiDAR stay excluded unless explicitly included. Use `--ma-only` for the previous curated selection. Reports never trigger another movement or repeat a collection automatically.

## Commands

Start the existing camera/rosbridge scripts on the three Jetsons. For navigation, prepare the robot with the remote and deliberately enable the existing Pi control driver with `bash ~/start_pi.sh --control`. That control startup can initialize walking mode. Plain `bash ~/start_pi.sh` provides passive capture only.

On the laptop:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ROS/Go1_Project/NavigationScripts

# Navigate one metre and collect data; prints quality verdict after finalizing.
python3 nav_and_collect_v5.py --navigate --distance 1 --speed 0.2

# No motion publisher or walking-mode request is created in this mode.
python3 nav_and_collect_v5.py --record-only --duration 20

# Recheck an existing bag without contacting the robot.
python3 nav_and_collect_v5.py --audit-only /absolute/path/to/bag

# Require zero ROS message sequence gaps instead of allowing up to 1 percent.
python3 nav_and_collect_v5.py --navigate --distance 1 --speed 0.2 --max-loss-percent 0

# Require at least 20 FPS for every built-in camera.
python3 nav_and_collect_v5.py --record-only --duration 20 --min-camera-fps 20
```

`--out DIR` changes the bag parent directory. `--report-dir DIR` changes only the report destination. RealSense/LiDAR are excluded by default; existing `--include-realsense` and `--include-lidar` options opt in to connected, launched sensors. The default quality rate profile is for the built-in MA_Raihan streams, not a specification for external devices.

Use the ROS Jazzy environment for both capture and full audit. If ROS message decoding or OpenCV is unavailable, the result is **UNVERIFIED**. `--skip-payload-check` likewise cannot produce a fully verified verdict.

## Verdict and recommendation

Every finalized v5 bag contains:

- `topic_inventory.json`: every selected host/topic/type, its collision-safe bag name and final recorded count.
- `run_context.json`: requested mode, distance, speed, readiness time, reached distance and target completion.
- `quality_report.json`: machine-readable metrics, thresholds, issues, verdict and `rerun_recommended` boolean.
- `quality_report.md`: human-readable table and findings.
- Existing `capture_report.json`, `timing.jsonl`, clock reference, metadata and SQLite bag files.

| Verdict | What to do | Exit code |
|---|---|---:|
| VALID | Keep this run. | 0 |
| VALID_WITH_WARNINGS | Meets current limits; keep it if warnings fit the experiment. | 0 |
| INVALID | Fix the listed cause, then repeat collection. | 2 |
| UNVERIFIED | Complete verification; this is not a valid pass. A repeat is not automatically necessary. | 3 |

If connection/setup fails before a bag exists, the command prints INVALID and exits 2. No reports can be placed inside a bag that was never created. An aborted/interrupted run stays INVALID even if some useful data was saved. The script does not delete partial bags.

## Default acceptance thresholds

These are initial experiment acceptance limits based on the reduced Pi profile verified in October 2026. They are not hardware maximum rates or evidence that every experiment needs the same limits.

| Check | Default failure condition |
|---|---|
| Recorder integrity | Any conversion failure, local queue drop or unprocessed queue entry |
| Completeness | Any required built-in camera/core robot stream absent; any selected topic empty, except intentional command silence during record-only capture |
| ROS message sequence gaps | More than 1% missing sequence numbers **on any individual live stream**; smaller nonzero gaps produce a warning |
| Source restart/order | Sequence reset/duplicate or repeated/decreasing source timestamps |
| Timestamp preservation | Bag timestamps do not match available source headers |
| Robot/thermal receipt delay | p95 above 200 ms or maximum above 500 ms |
| Camera/depth/cloud receipt delay | p95 above 250 ms or maximum above 1,000 ms |
| Clock inconsistency | Any live source timestamp more than 50 ms ahead of corrected laptop receipt time |
| Camera/IMU timestamp proximity | Nearest IMU sample more than 50 ms from a camera frame within their common timestamp interval |
| Capture duration | Less than 2 seconds in the assessed stable window; 10+ seconds is recommended for rate estimates |
| Navigation | Target not reached, distance error above max(5 cm, 5% of requested distance), or final recorded velocity command nonzero |
| Payloads | SQLite integrity error/count mismatch, deserialization failure, undecodable compressed image, missing motor fields or motor entries other than 12 |

The stable-window checks exclude subscription setup. For v5 navigation they start after the post-walking-mode readiness check. For legacy bags the analyzer estimates a shared start time one second after all available core streams/cameras first arrive. The window ends at the latest live core/camera receipt. Start/end gaps count, so a stream dying near the end does not pass because its earlier rate looked good. Database integrity, payload checks and recorder errors cover the entire saved bag.

| Stream | Minimum source rate | Maximum receipt gap |
|---|---:|---:|
| Front camera | 8 FPS | 0.5 s |
| Left, right, rear-down cameras | 15 FPS | 0.5 s |
| IMU, measured joints, battery | 20 Hz | 0.5 s |
| Detailed motor state | 6 Hz | 0.5 s |
| Odometry | 10 Hz | 0.5 s |
| Full High/Low state and raw state snapshots | 3 Hz | 0.5 s |
| Depth | 3 Hz | 1 s |
| Point clouds | 0.5 Hz | 3 s |
| Joint/controller temperatures | 0.5 Hz | 2.5 s |

A camera below 15 FPS produces a warning even if it meets the front camera's 8 FPS minimum. This explicitly accommodates the previously measured front-camera limitation; use `--min-camera-fps 20` if your experiment requires 20 FPS from all cameras. Calibration, metadata and CameraInfo may be latched and are not forced to match video rates. All selected streams still need at least one message.

Navigation timeout defaults to `max(10, 3*distance/speed + 10)` seconds, counted from the first navigation pose. Override with `--navigation-timeout SECONDS`. A stalled run raises an error and follows the existing velocity-stop/finalization path. This is a laptop timeout, not a replacement for a robot-side watchdog or the remote stop control. The inherited navigation still slows near its destination and uses its existing 2 cm target tolerance.

## What loss and synchronization mean here

`missing_sequences` and `loss_percent` describe gaps in ROS 1 `Header.seq` visible at recording time. They do not identify whether loss happened in a driver, a ROS queue, rosbridge, or the laptop. TCP can retransmit lost Ethernet/Wi-Fi packets without losing ROS messages, so zero ROS gaps does not prove zero network packet loss. Exact NIC/TCP packet statistics need separate controller/network instrumentation.

When sequence counters are unavailable, the report says that exact message-loss counting is unavailable and uses rate/gap/timestamp checks instead. It never calls interval-based estimates a measured network packet-loss count. Intentional sampling rates are the publisher's current rates; the script does not infer unexposed hardware samples. Sequence wrap is handled, while resets are reported.

Delay is corrected laptop receipt time minus the published source stamp. Pi passive feedback is stamped when observed, not at the motor encoder's hardware acquisition instant. Camera/IMU pairing and preserved stamps help assess alignment but do not prove hardware synchronization or UTC accuracy. Latched metadata/CameraInfo and intentionally sparse depth are not interchangeable with video frames.

The quality report uses full message/image decoding, but does not judge exposure, lens occlusion, scene usefulness, calibration accuracy, dense-depth coverage or odometry's physical ground-truth accuracy. Sparse zero-depth regions can be legitimate. Review camera previews for the experiment itself.

Legacy bags without `run_context.json` can be checked for data quality, but requested distance and target completion remain unknown and are flagged. A report can recommend keeping their data without proving that a particular navigation task succeeded.

## Validation of this version

Tests cover healthy data, low and excessive sequence loss, sequence resets/wrap, delay spikes, clocks, streams dying early, missing sensors, recorder loss, aborted runs, target failure/overshoot, skipped payload checks, wrapper error handling and navigation timeout. The previous successful 3 m bag passes with a front-FPS warning; the previous telemetry-delay failure is rejected. Current hardware movement is not part of development testing.


## All-topic inventory (2026-11-12)

All-topic capture is now the default. Identical names from different ROS masters (for example `/rosout`) are preserved under `/source_<IP>_<port>/...`; they are not merged or overwritten. All image transports actually advertised are included in this mode, so it can require more bandwidth than `--ma-only`. Raw/fisheye topics are not silently filtered in all-topic mode.

Log/client-status topics can legitimately stay silent; their zero counts are listed as `silent_event_topics`, not declared sensor loss. Empty sensor topics still invalidate the run. The recorder compares the final topic list with the initial selection and marks the run INVALID if new selected topics appeared after discovery. Final-inventory query failures produce UNVERIFIED. Start all required sensor drivers before collecting; v5 does not claim it recorded topics created mid-run.

Four unreachable old factory camera node registrations (`/camera1`, `/camera3`, `/camera4`, `/camera5`) were checked before ROS-master cleanup on the Pi. They accounted for seven phantom point-cloud/range topics. No source files or active camera processes were removed. Restoring the original camera drivers recreates their registrations. Current corrected camera/depth/native-point streams remain available in MA_Raihan.
