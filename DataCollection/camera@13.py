#!/usr/bin/env python3
"""
Go1 Jetson .13 Comprehensive Data Collector
Dynamically discovers and records all active sensor streams (Compressed video, IMU, Odometry, etc.) to a ROS 2 bag.
Usage: python3 data@13.py --host 192.168.123.13
"""
import argparse
import base64
import queue
import time
import array
from datetime import datetime
from pathlib import Path

def convert(value):
    if isinstance(value, list):
        return [convert(v) for v in value]
    if not isinstance(value, dict):
        return value
    if set(value.keys()) == {'secs', 'nsecs'}:
        return {'sec': value['secs'], 'nanosec': value['nsecs']}
    res = {}
    for k, v in value.items():
        if k == 'seq':
            continue
        res[k] = convert(v)
    return res

class JetsonDataCollector:
    def __init__(self, host='192.168.123.13', port=9090):
        self.host = host
        self.port = port
        board_id = host.split('.')[-1]
        self.bag_path = Path('../Bags/go1_bags').expanduser() / datetime.now().strftime(f'%Y%m%d_%H%M%S_jetson_{board_id}_alldata')
        
        self.ros = None
        self.node = None
        self.writer = None
        self.subs = []
        self.specs = {}
        self.messages = queue.Queue(maxsize=10000)
        self.is_recording = False

        # Broadly accepting standard sensor and navigation types to capture everything active on .13
        self.target_types = [
            'sensor_msgs/CompressedImage',
            'sensor_msgs/Image',
            'sensor_msgs/Imu',
            'sensor_msgs/JointState',
            'sensor_msgs/CameraInfo',
            'sensor_msgs/PointCloud2',
            'nav_msgs/Odometry',
            'geometry_msgs/Twist'
        ]

    def run(self):
        import roslibpy, rclpy, rosbag2_py
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import serialize_message
        from rosidl_runtime_py.set_message import set_message_fields

        print(f"Connecting to Jetson at {self.host}:{self.port}...")
        self.ros = roslibpy.Ros(host=self.host, port=self.port)
        self.ros.run(timeout=10)
        
        roster = roslibpy.Service(self.ros, '/rosapi/topics', 'rosapi/Topics').call(roslibpy.ServiceRequest({}), timeout=10)
        
        # Filter available topics, prioritizing compressed streams to protect WiFi bandwidth
        topics = {n: k for n, k in zip(roster['topics'], roster['types']) 
                  if any(t in k for t in ['CompressedImage', 'Imu', 'Odometry', 'JointState', 'CameraInfo', 'Twist']) 
                  and 'raw' not in n}
        
        if not topics:
            print("No matching sensor topics found. Ensure nodes and rosbridge are running.")
            return

        rclpy.init()
        self.node = rclpy.create_node(f'go1_collector_{self.host.split(".")[-1]}')
        self.bag_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr')
        )

        print(f"Discovered {len(topics)} sensor topics:")
        for i, (name, kind) in enumerate(sorted(topics.items()), 1):
            print(f" - {name} ({kind})")
            ros2_type = kind.replace('/', '/msg/', 1)
            try: 
                msg_class = get_message(ros2_type)
            except Exception: 
                print(f"   Skipping {name}, missing ROS 2 message type {ros2_type}")
                continue
                
            self.writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=ros2_type, serialization_format='cdr'))
            self.specs[name] = (msg_class, kind)

            def make_callback(topic_name):
                return lambda data: self.messages.put_nowait((topic_name, data, time.time_ns())) if not self.messages.full() else None
            
            sub = roslibpy.Topic(self.ros, name, kind, throttle_rate=0, queue_length=50)
            sub.subscribe(make_callback(name))
            self.subs.append(sub)

        self.is_recording = True
        print(f"\nRecording to: {self.bag_path}\nPress Ctrl+C to stop.")

        frames_written = 0
        try:
            while self.is_recording and self.ros.is_connected:
                while not self.messages.empty():
                    name, data, rcv_ns = self.messages.get()
                    if name not in self.specs:
                        continue
                    cls, ros1_type = self.specs[name]
                    
                    try:
                        clean_data = convert(data)

                        # FAST DECODE: base64 -> byte array for heavy data (Images)
                        if 'data' in clean_data and isinstance(clean_data['data'], str):
                            raw_bytes = base64.b64decode(clean_data['data'])
                            clean_data['data'] = array.array('B', raw_bytes)

                        msg = cls()
                        set_message_fields(msg, clean_data)
                        self.writer.write(name, serialize_message(msg), rcv_ns)
                        frames_written += 1
                        if frames_written % 50 == 0:
                            print(f"Recorded {frames_written} messages...", end='\r')
                    except Exception as e:
                        print(f"Error packing message on {name}: {e}")
                        
                rclpy.spin_once(self.node, timeout_sec=0.01)
        except KeyboardInterrupt:
            print(f"\nStopping collection. Total messages recorded: {frames_written}")
        finally:
            self.stop()

    def stop(self):
        self.is_recording = False
        for sub in self.subs:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        if self.writer:
            self.writer.close()
        if self.node:
            import rclpy
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        if self.ros:
            self.ros.terminate()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='192.168.123.13', help="Jetson IP")
    args = parser.parse_args()
    JetsonDataCollector(host=args.host).run()