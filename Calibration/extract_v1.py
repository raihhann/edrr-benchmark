#!/usr/bin/env python3
"""
Extracts video topics from a ROS 2 bag into rigidly synchronized MP4 files.
Uses a global timeline and Zero-Order Hold to ensure all videos are the exact
same length and perfectly aligned with sensor data.

Usage: python3 vid_extract.py ../Bags/go1_bags/<bag_folder>
"""
import sys
import csv
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TARGET_FPS = 15.0  # Unified playback framerate for all videos

if len(sys.argv) < 2:
    print("Usage: python3 vid_extract.py <path_to_bag_folder>")
    sys.exit(1)

bag_dir = Path(sys.argv[1])
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('cdr', 'cdr')
)

type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
frames_by_topic = {}
timestamps_by_topic = {}

print("[INFO] Reading messages into memory...")
while reader.has_next():
    topic_name, data, timestamp = reader.read_next()
    msg_type_str = type_map[topic_name]
    
    if 'Image' not in msg_type_str:
        continue

    msg_cls = get_message(msg_type_str)
    msg = deserialize_message(data, msg_cls)

    if hasattr(msg, 'format'):  # CompressedImage
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:  # Raw sensor_msgs/Image
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if topic_name not in frames_by_topic:
        frames_by_topic[topic_name] = []
        timestamps_by_topic[topic_name] = []

    frames_by_topic[topic_name].append(frame)
    timestamps_by_topic[topic_name].append(timestamp)

if not timestamps_by_topic:
    print("[ERROR] No images found in bag.")
    sys.exit(0)

# 1. Establish the Global Timeline across ALL cameras
all_timestamps = [ts for timestamps in timestamps_by_topic.values() for ts in timestamps]
global_start_ns = min(all_timestamps)
global_end_ns = max(all_timestamps)
duration_sec = (global_end_ns - global_start_ns) / 1e9
total_target_frames = int(duration_sec * TARGET_FPS)

print(f"\n[INFO] Global Bag Duration: {duration_sec:.2f} seconds")
print(f"[INFO] Generating {total_target_frames} frames per video at {TARGET_FPS} FPS to maintain sync...\n")

# 2. Extract and Align each topic
for topic_name, frames in frames_by_topic.items():
    timestamps = timestamps_by_topic[topic_name]
    h, w = frames[0].shape[:2]
    blank_frame = np.zeros((h, w, 3), dtype=np.uint8)
    
    clean_name = topic_name.strip('/').replace('/', '_')
    out_vid = str(bag_dir / f"{clean_name}.mp4")
    out_csv = str(bag_dir / f"{clean_name}_timestamps.csv")
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_vid, fourcc, TARGET_FPS, (w, h))
    
    csv_file = open(out_csv, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["mp4_frame_index", "ros_timestamp_ns", "interpolated"])

    current_source_idx = 0
    
    for tick in range(total_target_frames):
        # Calculate the exact nanosecond time of this frame tick
        tick_time_ns = global_start_ns + int((tick / TARGET_FPS) * 1e9)
        
        # Advance the source frame index until it matches the current timeline tick
        while current_source_idx < len(timestamps) - 1 and timestamps[current_source_idx + 1] <= tick_time_ns:
            current_source_idx += 1
            
        # If the camera hasn't started streaming yet, write a black frame
        if tick_time_ns < timestamps[0]:
            writer.write(blank_frame)
            csv_writer.writerow([tick, tick_time_ns, "True (Pre-boot Blank)"])
        else:
            # Write the most recent frame (Zero-Order Hold)
            writer.write(frames[current_source_idx])
            is_held = "True (Held)" if timestamps[current_source_idx] < tick_time_ns - (1e9/TARGET_FPS) else "False"
            csv_writer.writerow([tick, timestamps[current_source_idx], is_held])

    writer.release()
    csv_file.close()

    print(f"[SUCCESS] {topic_name}")
    print(f"   -> Video: {out_vid}")
    print(f"   -> Timestamps: {out_csv}")

print("\n[INFO] Synchronized Extraction complete.")