# Go1 Project

This repository contains the tools used to collect, validate, calibrate, and replay data from a Unitree Go1 robot. The project is organized around ROS bag capture, camera calibration, quick verification, and robot motion commands.

## Project structure

- `DataCollection/`: scripts that connect to the robot over rosbridge and record sensor streams into ROS 2 bags.
  - `pi_collector.py`: collects Raspberry Pi sensor and topic data.
  - `camera.py` and `camera@13.py`: capture Jetson vision streams and camera topics.
- `Calibration/`: scripts that read bag files, apply factory camera calibration, and output rectified MP4 videos for each lens.
- `Verification/`: scripts that validate bag contents and export frames from recorded videos.
  - `verify_pi.py`: checks whether expected robot topics are present and populated.
  - `vid_extract.py`: converts image topics in a bag to MP4 files.
- `Navigation Scripts/`: movement control for the robot using odometry and velocity commands.
- `Bags/`: generated ROS bag data folders used during collection and testing.
- `rosenv/`: local Python virtual environment for project dependencies.

## Typical workflow

1. Activate the project environment:
   ```bash
   source rosenv/bin/activate
   ```
2. Collect raw robot data from the Pi:
   ```bash
   cd DataCollection
   python3 pi_collector.py
   ```
3. Collect Jetson camera data:
   ```bash
   cd DataCollection
   python3 camera.py --host 192.168.123.14
   ```
4. Verify the bag contents:
   ```bash
   cd Verification
   python3 verify_pi.py ../Bags/go1_bags/<bag_name>
   ```
5. Export image topics to videos:
   ```bash
   cd Verification
   python3 vid_extract.py ../Bags/go1_bags/<bag_name>
   ```
6. Apply calibration to a camera bag:
   ```bash
   cd Calibration
   python3 vid_calib_front.py ../Bags/go1_bags/<bag_name>
   ```
7. Move the Go1 robot with the navigation script:
   ```bash
   cd "Navigation Scripts"
   python3 navigation.py --distance 2.0 --speed 0.2
   ```

## Notes

- This project depends on ROS 2 tooling, `rosbag2`, `rosbags`, `roslibpy`, and OpenCV.
- Raw bag data and the local virtual environment are intentionally kept out of source control.
- The scripts expect the robot to be reachable over the configured IP and rosbridge ports.
