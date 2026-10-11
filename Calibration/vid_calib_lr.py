#!/usr/bin/env python3
"""
Extracts and undistorts only the flat (right) lens from Go1 stereo camera bags.
Outputs exactly two MP4 videos.
Usage: python3 vid_extract.py ../Bags/go1_bags/<bag_folder>
"""
import sys
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Factory Calibration Data for the Right (Flat) Lenses only
CALIB = {
    '/camera/left/image': { # cam3 right lens
        'K': np.array([[367.64, 0.74, 481.73], [0.0, 378.37, 364.62], [0.0, 0.0, 1.0]]),
        'D': np.array([-0.2122, 0.0290, -0.000727, 0.000667])
    },
    '/camera/right/image': { # cam4 right lens
        'K': np.array([[371.36, 0.89, 424.74], [0.0, 382.38, 355.96], [0.0, 0.0, 1.0]]),
        'D': np.array([-0.2088, 0.0275, -0.00115, 0.00101])
    }
}

if len(sys.argv) < 2:
    print("Usage: python3 vid_extract.py <path_to_bag_folder>")
    sys.exit(1)

bag_dir = Path(sys.argv[1])
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('cdr', 'cdr')
)

type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
frames = {}
timestamps = {}

print("Reading and decoding frames from bag...")
while reader.has_next():
    topic_name, data, timestamp = reader.read_next()
    if 'Image' not in type_map.get(topic_name, ''):
        continue

    msg_cls = get_message(type_map[topic_name])
    msg = deserialize_message(data, msg_cls)

    if hasattr(msg, 'format'):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    frames.setdefault(topic_name, []).append(frame)
    timestamps.setdefault(topic_name, []).append(timestamp)

for topic, imgs in frames.items():
    if not imgs or topic not in CALIB:
        continue

    ts = timestamps[topic]
    duration_sec = (ts[-1] - ts[0]) / 1e9
    actual_fps = max(1.0, len(imgs) / duration_sec) if duration_sec > 0 else 30.0

    full_h, full_w = imgs[0].shape[:2]
    lens_w = full_w // 2
    
    scale_x = lens_w / 928.0
    scale_y = full_h / 800.0

    K_scaled = CALIB[topic]['K'].copy()
    K_scaled[0, 0] *= scale_x
    K_scaled[1, 1] *= scale_y
    K_scaled[0, 2] *= scale_x
    K_scaled[1, 2] *= scale_y
    D = CALIB[topic]['D']

    clean_name = topic.strip('/').replace('/', '_')
    out_file = str(bag_dir / f"{clean_name}_flat.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_file, fourcc, actual_fps, (lens_w, full_h))

    print(f"[{topic}] Extracting and un-warping {len(imgs)} frames at {actual_fps:.2f} FPS...")
    
    for img in imgs:
        # Only slice the right half of the side-by-side image
        right_raw = img[:, lens_w:]
        flat = cv2.undistort(right_raw, K_scaled, D)
        writer.write(flat)

    writer.release()

print("Extraction complete. Generated 2 flat videos.")