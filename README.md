# ROS 2 Multi-Robot Bag Visualization Studio

A synchronized web-based visualization cockpit for ROS 2 SQLite3 (`.db3`) bag recordings across different robots (Unitree Go1, TurtleBot3, and generic ROS 2 systems).

## Key Features

- **Upload or Open Any ROS 2 Bag (`.db3` or Bag Folder)**:
  - Upload `.db3` file(s) or an entire bag folder directly from your browser via drag-and-drop or file picker.
  - One-click loading for discovered bags on disk (`/home/fsti/ROS/Go1_Project/Bags/go1_bags/...`, `/home/fsti/ROS/TurtleBot/Bags/...`, and `sample_bags/`), or load any local path.
- **Synchronized Multi-Camera Video Streaming**:
  - Toggle between **Single Stream** focus mode and **Multi-Stream Grid** (`Auto`, `1`, `2`, `3`, or `4` columns).
  - Select individual camera streams or quick-filter by **All RGB**, **All Depth**, or **All Streams**.
  - Automatic 16-bit depth (`16UC1`) colormapping (`Turbo`, `Inferno`, `Viridis`, `Bone`, `Grayscale`).
  - Maximize/Solo any camera stream with a single click.
- **Synchronized 2D / 3D Spatial Sensor View**:
  - **2D Odometry Trajectory Map**: Full path `(x, y)`, start marker, and live robot position + heading vector.
  - **3D PointCloud2 Viewer**: Interactive 3D orbit/pan/zoom canvas for multi-camera point clouds (`/ma_raihan/cameras/*/points_native`).
  - **2D LiDAR LaserScan Radar**: Polar range radar for `/scan` topics.
- **Synchronized Telemetry Time-Series Plots**:
  - Interactive 4-panel charts (Velocity & Commands, IMU Roll/Pitch/Yaw & Accel/Gyro, Joint Positions/Velocities/Efforts/Foot Forces/Wheel Encoders, Battery & Temperatures).
  - Switch between **Full Trace + Cursor** and **Progressive Live Stream** modes; click/drag any chart to scrub time.
- **All-Topics Live Sync Inspector**:
  - Real-time decoded state and message age (`Δt`) for every topic in the bag at the current playback time, with search, category filter, and expandable JSON view.

## Quick Start

```bash
cd /home/fsti/ROS/Visualization
./run.sh
```

Then open **http://127.0.0.1:8765** in your browser.
