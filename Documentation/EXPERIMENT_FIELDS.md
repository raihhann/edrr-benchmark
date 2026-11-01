# experiment.json

- `name`: unique bag directory name; never overwrites existing recordings.
- `mode`: `record` (stationary) or `navigate` (actual movement requested).
- `arguments`: requested duration, distance in metres, speed in m/s, startup timeout, and explicitly required topics.
- `status`: `preparing`, `complete`, `aborted`, `interrupted`, or `partial`. Complete requires a finalized bag, readable SQLite and actual required messages. It does not prove hardware-trigger synchronization or zero packet loss.
- `error`: reason an interrupted/aborted/partial run did not complete.
- `started_utc`: timezone-aware start time.
- `events`: time_ns uses epoch nanoseconds; records readiness, motion progress, target reached and errors. Forward/lateral values are metres in the initial robot heading frame; heading is degrees and commanded speed m/s.
- `recording_location`: `robot`; Wi-Fi is used for SSH control and later transfer.
- `timestamp_semantics`: native rosbag2 receipt timestamps, with source headers preserved inside messages.
- `hardware_trigger_sync`: false; no common camera/IMU hardware trigger is configured.
- `required_topics`: streams that must deliver actual fresh data before movement and during the run.
- `topic_types_at_start`: discovered ROS interface names.
- `optional_hardware`: whether camera and laser publishers were observed, not a guarantee that every optional sensor is attached.
- `monitor_topics`: lightweight independent subscriber observations: count, bytes, epoch first/last arrival, header-to-monitor receipt age in ms. These are monitoring counts, not authoritative bag counts. The native recorder runs as its own process.
- `bag_topics`: authoritative SQLite message counts and types after finalization.
- `missing_required_in_bag`: required topics with fewer than two recorded messages.
- `sqlite_integrity`: SQLite quick_check result for each split database.

`audit.json` decodes all bag messages on the robot, reports actual source rates, timestamp gaps/backwards values, frame IDs, image shape, CameraInfo calibration state, scan validity, OpenCR uptime and register fields, final commands and odometry displacement. `download_verified.json` on the laptop contains SHA-256 hashes verified against the finalized robot files. No passwords are stored.
