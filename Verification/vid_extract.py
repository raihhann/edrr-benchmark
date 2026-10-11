#!/usr/bin/env python3
"""
Extracts video topics from a ROS 2 bag into MP4 files with accurate playback duration.
Usage: python3 vid_extract.py ../Bags/go1_bags/<bag_folder>
"""
import sys
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

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

print("Reading frames from bag...")
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

for topic_name, frames in frames_by_topic.items():
    if not frames:
        continue

    timestamps = timestamps_by_topic[topic_name]
    total_frames = len(frames)
    
    # Calculate real elapsed time in seconds from nanosecond timestamps
    duration_sec = (timestamps[-1] - timestamps[0]) / 1e9
    if duration_sec > 0:
        actual_fps = max(1.0, total_frames / duration_sec)
    else:
        actual_fps = 15.0

    h, w = frames[0].shape[:2]
    clean_name = topic_name.strip('/').replace('/', '_')
    out_file = str(bag_dir / f"{clean_name}.mp4")
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_file, fourcc, actual_fps, (w, h))

    for f in frames:
        writer.write(f)
    writer.release()

    print(f"[{topic_name}] Exported {total_frames} frames over {duration_sec:.2f}s (Playback FPS: {actual_fps:.2f}) -> {out_file}")

print("Extraction complete.")