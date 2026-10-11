#!/usr/bin/env python3
"""
Extracts and undistorts the left fisheye lens from Go1 Jetson .13 Chin Camera bags.
Outputs an MP4 video directly inside the bag folder.
Usage: python3 vid_calib_chin.py ../Bags/go1_bags/<bag_folder>
"""
import sys
from pathlib import Path
import cv2
import numpy as np
from rosbags.highlevel import AnyReader

# Factory Calibration Data for Jetson .13 (Chin Camera - Left Eye)
CALIB_DATA = {
    'K': np.array([
        [3.8236638117377043e+02, 1.8639003728345474e-01, 4.3705970837958563e+02],
        [0., 3.9350511522633599e+02, 3.7500953475197059e+02],
        [0., 0., 1.]
    ]),
    'D': np.array([-2.0982121740198759e-01, 2.6892071439523178e-02, 1.3335186847985123e-03, 1.2056077939820386e-03]),
    'xi': np.array([5.5387052837371287e-01], dtype=np.float64)
}

# Map both raw and compressed topics to the same calibration data
CALIB = {
    '/camera/chin/image': CALIB_DATA,
    '/camera/chin/image/compressed': CALIB_DATA
}

if len(sys.argv) < 2:
    print("Usage: python3 vid_calib_chin.py <path_to_bag_folder>")
    sys.exit(1)

bag_path = Path(sys.argv[1])
frames = {}
timestamps = {}

print(f"Reading and decoding frames from bag: {bag_path.name}...")
with AnyReader([bag_path]) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic not in CALIB:
            continue
            
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        # Handle both CompressedImage and raw Image formats
        if hasattr(msg, 'format'): 
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else: 
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if hasattr(msg, 'encoding') and msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
        frames.setdefault(connection.topic, []).append(frame)
        timestamps.setdefault(connection.topic, []).append(timestamp)

for topic, imgs in frames.items():
    if not imgs:
        continue

    ts = timestamps[topic]
    duration_sec = (ts[-1] - ts[0]) / 1e9
    actual_fps = max(1.0, len(imgs) / duration_sec) if duration_sec > 0 else 30.0

    full_h, full_w = imgs[0].shape[:2]
    lens_w = full_w // 2

    # Scale the factory calibration (864x800) to the active resolution
    scale_x = lens_w / 864.0
    scale_y = full_h / 800.0

    K_scaled = CALIB[topic]['K'].copy()
    K_scaled[0, 0] *= scale_x
    K_scaled[1, 1] *= scale_y
    K_scaled[0, 2] *= scale_x
    K_scaled[1, 2] *= scale_y
    
    D = CALIB[topic]['D']
    xi = CALIB[topic]['xi']

    K_new = K_scaled.copy()
    K_new[0, 0] *= 0.8 
    K_new[1, 1] *= 0.8

    map1, map2 = cv2.omnidir.initUndistortRectifyMap(
        K_scaled, D, xi, np.eye(3), K_new, (lens_w, full_h), cv2.CV_16SC2, cv2.omnidir.RECTIFY_PERSPECTIVE
    )

    clean_name = topic.strip('/').replace('/', '_')
    out_file = str(bag_path / f"{clean_name}_flat.mp4")
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_file, fourcc, actual_fps, (lens_w, full_h))

    print(f"[{topic}] Extracting and un-warping {len(imgs)} frames at {actual_fps:.2f} FPS...")
    print(f"Saving to: {out_file}")

    for img in imgs:
        left_raw = img[:, :lens_w]
        flat = cv2.remap(left_raw, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        writer.write(flat)

    writer.release()

print("Extraction complete.")