#!/usr/bin/env python3
"""Export all image topics from a ROS 2 bag, rectifying known Go1 cameras.

Fisheye cameras use the project factory omnidirectional calibration and export
the left lens. The calibrated stereo cameras export their right lens with
pinhole undistortion. Other image topics are exported unchanged.

Usage:
    python3 Calibration/all_vid_xtract.py Bags/go1_bags/<bag_folder>
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


FISHEYE_CALIBRATIONS = {
    "/camera/chin/image": {
        "K": np.array(
            [
                [382.36638117377043, 0.18639003728345474, 437.05970837958563],
                [0.0, 393.50511522633599, 375.00953475197059],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "D": np.array(
            [-0.20982121740198759, 0.026892071439523178, 0.0013335186847985123, 0.0012056077939820386]
        ),
        "xi": np.array([0.55387052837371287], dtype=np.float64),
        "calibration_width": 864.0,
        "mode": "omni",
    },
    "/camera/front/image": {
        "K": np.array(
            [
                [377.13841059981576, 0.89369206792650369, 407.0471737933687],
                [0.0, 387.99137862130459, 418.52546766041572],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "D": np.array(
            [-0.21219337183569353, 0.02751333301953399, 0.00028675441501255222, -0.0024604860463458425]
        ),
        "xi": np.array([0.53688234227411147], dtype=np.float64),
        "calibration_width": 864.0,
        "mode": "omni",
    },
    "/camera/rear_down/image": {
        "K": np.array(
            [
                [372.89966334699091, 0.79762475220521589, 451.5989150378611],
                [0.0, 383.66047639225144, 361.98445649086148],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "D": np.array(
            [-0.2112017955338347, 0.02803476940525813, 0.0015826966805598311, 0.0006995638838497443]
        ),
        "xi": np.array([0.52321023682028145], dtype=np.float64),
        "calibration_width": 928.0,
        "mode": "omni",
    },
}

STEREO_CALIBRATIONS = {
    "/camera/left/image": {
        "K": np.array(
            [[367.64, 0.74, 481.73], [0.0, 378.37, 364.62], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        "D": np.array([-0.2122, 0.0290, -0.000727, 0.000667]),
        "calibration_width": 928.0,
    },
    "/camera/right/image": {
        "K": np.array(
            [[371.36, 0.89, 424.74], [0.0, 382.38, 355.96], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        "D": np.array([-0.2088, 0.0275, -0.00115, 0.00101]),
        "calibration_width": 928.0,
    },
}


def calibration_for(topic, calibration_set):
    normalized_topic = topic.removesuffix("/compressed")
    for base_topic, calibration in calibration_set.items():
        if normalized_topic == base_topic or normalized_topic.endswith(base_topic):
            return calibration
    return None


def decode_frame(message):
    if hasattr(message, "format"):
        frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("OpenCV could not decode the CompressedImage payload")
        return frame

    encoding = message.encoding.lower()
    height, width, step = message.height, message.width, message.step
    if encoding in {"bgr8", "rgb8", "8uc3"}:
        channels, dtype = 3, np.uint8
    elif encoding in {"bgra8", "rgba8", "8uc4"}:
        channels, dtype = 4, np.uint8
    elif encoding in {"mono8", "8uc1", "8sc1"}:
        channels, dtype = 1, np.uint8
    elif encoding in {"mono16", "16uc1", "16sc1"}:
        channels, dtype = 1, np.uint16
    else:
        raise ValueError(f"Unsupported raw image encoding: {message.encoding}")

    bytes_per_pixel = np.dtype(dtype).itemsize * channels
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    pixels = row[:, : width * bytes_per_pixel].copy()
    frame = pixels.view(dtype).reshape(height, width, channels)
    if channels == 1:
        if dtype == np.uint16:
            maximum = int(frame.max()) if frame.size else 0
            scale = 255.0 / maximum if maximum else 1.0
            frame = cv2.convertScaleAbs(frame, alpha=scale)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def get_playback_fps(count, first_timestamp, last_timestamp):
    duration = (last_timestamp - first_timestamp) / 1e9
    return (max(1.0, count / duration) if duration > 0 else 30.0), duration


def prepare_output(topic, frame, bag_path, fps):
    height, width = frame.shape[:2]
    fisheye = calibration_for(topic, FISHEYE_CALIBRATIONS)
    stereo = calibration_for(topic, STEREO_CALIBRATIONS)
    mode = "raw"
    size = (width, height)
    rectify = None

    if fisheye is not None:
        lens_width = width // 2
        scale_x = lens_width / fisheye["calibration_width"]
        scale_y = height / 800.0
        intrinsic = fisheye["K"].copy()
        intrinsic[0, 0] *= scale_x
        intrinsic[1, 1] *= scale_y
        intrinsic[0, 2] *= scale_x
        intrinsic[1, 2] *= scale_y
        output_intrinsic = intrinsic.copy()
        output_intrinsic[0, 0] *= 0.8
        output_intrinsic[1, 1] *= 0.8
        rectify = cv2.omnidir.initUndistortRectifyMap(
            intrinsic,
            fisheye["D"],
            fisheye["xi"],
            np.eye(3),
            output_intrinsic,
            (lens_width, height),
            cv2.CV_16SC2,
            cv2.omnidir.RECTIFY_PERSPECTIVE,
        )
        mode = "fisheye-left"
        size = (lens_width, height)
    elif stereo is not None:
        lens_width = width // 2
        scale_x = lens_width / stereo["calibration_width"]
        scale_y = height / 800.0
        intrinsic = stereo["K"].copy()
        intrinsic[0, 0] *= scale_x
        intrinsic[1, 1] *= scale_y
        intrinsic[0, 2] *= scale_x
        intrinsic[1, 2] *= scale_y
        rectify = (intrinsic, stereo["D"])
        mode = "stereo-right"
        size = (lens_width, height)

    clean_name = topic.strip("/").replace("/", "_")
    output_path = bag_path / f"{clean_name}_{mode.replace('-', '_')}.mp4"
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f"Could not open video writer for {output_path}")
    return writer, output_path, mode, rectify


def transform_frame(frame, mode, rectify):
    if mode == "fisheye-left":
        left_lens = frame[:, : frame.shape[1] // 2]
        map_x, map_y = rectify
        return cv2.remap(
            left_lens,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
    if mode == "stereo-right":
        right_lens = frame[:, frame.shape[1] // 2 :]
        intrinsic, distortion = rectify
        return cv2.undistort(right_lens, intrinsic, distortion)
    return frame


def extract_bag(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory does not exist: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    image_types = {
        topic: type_name
        for topic, type_name in type_map.items()
        if type_name in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
    }
    if not image_types:
        raise RuntimeError("The bag has no sensor_msgs/Image or CompressedImage topics.")

    image_classes = {topic: get_message(type_name) for topic, type_name in image_types.items()}
    stats = {}
    while reader.has_next():
        topic, _, timestamp = reader.read_next()
        if topic not in image_classes:
            continue
        if topic not in stats:
            stats[topic] = [0, timestamp, timestamp]
        stats[topic][0] += 1
        stats[topic][2] = timestamp

    if not stats:
        raise RuntimeError("Image topics are registered, but contain no messages.")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writers = {}
    try:
        while reader.has_next():
            topic, raw_data, _ = reader.read_next()
            if topic not in stats:
                continue
            message = deserialize_message(raw_data, image_classes[topic])
            frame = decode_frame(message)
            if topic not in writers:
                fps, duration = get_playback_fps(*stats[topic])
                writer, output_path, mode, rectify = prepare_output(
                    topic, frame, bag_path, fps
                )
                writers[topic] = (writer, output_path, mode, rectify)
                print(
                    f"[{topic}] {stats[topic][0]} frames, {duration:.2f}s, "
                    f"{fps:.2f} fps; mode={mode}; output={output_path}"
                )
            writer, _, mode, rectify = writers[topic]
            writer.write(transform_frame(frame, mode, rectify))
    finally:
        for writer, _, _, _ in writers.values():
            writer.release()

    missing = sorted(set(stats) - set(writers))
    if missing:
        raise RuntimeError(f"Could not create videos for image topics: {', '.join(missing)}")
    print(f"[INFO] Exported {len(writers)} videos into {bag_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    args = parser.parse_args()
    extract_bag(args.bag.expanduser().resolve())


if __name__ == "__main__":
    main()
