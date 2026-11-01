#!/usr/bin/env python3
"""Export a ROS 2 compressed camera stream as MP4 plus exact frame timestamps.
Source /opt/ros/jazzy/setup.bash before running. MP4 uses the median source
frame interval; use the CSV for exact sensor alignment, including any gaps.
"""
import argparse
import csv
import json
import sqlite3
import statistics
import subprocess
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('--topic', default='/camera/pi/image_raw/compressed')
    args = p.parse_args()
    bag = args.bag.resolve()
    files = sorted(bag.glob('*.db3'))
    if not files:
        p.error('No .db3 files in bag directory')
    rows = []
    for db in files:
        with sqlite3.connect(db.as_uri() + '?mode=ro', uri=True) as conn:
            topic = conn.execute('SELECT id,type FROM topics WHERE name=?', (args.topic,)).fetchone()
            if topic is None:
                continue
            if topic[1] != 'sensor_msgs/msg/CompressedImage':
                p.error('Selected topic must be sensor_msgs/msg/CompressedImage')
            for receipt, data in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic[0],)):
                msg = deserialize_message(data, CompressedImage)
                stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                rows.append((receipt, stamp, bytes(msg.data)))
    rows.sort(key=lambda row: row[0])
    if len(rows) < 2:
        p.error('At least two camera frames required')
    intervals = [b[1]-a[1] for a,b in zip(rows, rows[1:]) if b[1] > a[1]]
    if not intervals:
        p.error('No increasing camera timestamps')
    fps = 1e9 / statistics.median(intervals)
    output = bag / 'ExtractedVideo'
    output.mkdir(exist_ok=True)
    stem = args.topic.strip('/').replace('/', '_')
    video = output / (stem + '.mp4')
    timestamps = output / (stem + '_timestamps.csv')
    if video.exists() or timestamps.exists():
        p.error('Export exists; refusing to overwrite')
    first = cv2.imdecode(np.frombuffer(rows[0][2], dtype=np.uint8), cv2.IMREAD_COLOR)
    if first is None:
        raise RuntimeError('First compressed frame cannot be decoded')
    height, width = first.shape[:2]
    command = ['ffmpeg','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','bgr24','-video_size',f'{width}x{height}','-framerate',str(fps),'-i','pipe:0','-an','-c:v','mpeg4','-vtag','mp4v','-q:v','2','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        with timestamps.open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['frame_index','source_stamp_ns','bag_receipt_ns','source_elapsed_s'])
            for i, (receipt, stamp, data) in enumerate(rows):
                frame = first if i == 0 else cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.shape != first.shape:
                    raise RuntimeError(f'Invalid frame or changed dimensions at frame {i}')
                process.stdin.write(frame.tobytes())
                writer.writerow([i, stamp, receipt, (stamp-rows[0][1])/1e9])
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError('FFmpeg encoding failed')
    except BaseException:
        process.kill()
        process.wait()
        raise
    report = {'topic': args.topic, 'frames':len(rows), 'width':width,'height':height,'playback_fps':fps,'source_span_s':(rows[-1][1]-rows[0][1])/1e9,'timing':'Constant frame rate MP4; exact source and receipt timestamps are in CSV.'}
    (output/(stem+'_export.json')).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print('Video:', video)
    print('Timestamps:', timestamps)


if __name__ == '__main__':
    main()
