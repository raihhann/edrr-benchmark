#!/usr/bin/env python3
"""
Go1 Jetson Vision Collector
Records raw Image and CompressedImage topics from a Go1 Jetson board to a ROS 2 bag.
"""
import argparse
import base64
import queue
import time
import array  # Added for high-speed byte parsing
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

class JetsonVisionCollector:
    def __init__(self, host='192.168.123.14', port=9090):
        self.host = host
        self.port = port
        board_id = host.split('.')[-1]
        self.bag_path = Path('../Bags/go1_bags').expanduser() / datetime.now().strftime(f'%Y%m%d_%H%M%S_jetson_{board_id}_vision')
        
        self.ros = None
        self.node = None
        self.writer = None
        self.subs = []
        self.specs = {}
        self.messages = queue.Queue(maxsize=3000)
        self.is_recording = False

    def run(self):
        import roslibpy, rclpy, rosbag2_py
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import serialize_message
        from rosidl_runtime_py.set_message import set_message_fields

        print(f"Connecting to Jetson at {self.host}:{self.port}...")
        self.ros = roslibpy.Ros(host=self.host, port=self.port)
        self.ros.run(timeout=10)
        
        roster = roslibpy.Service(self.ros, '/rosapi/topics', 'rosapi/Topics').call(roslibpy.ServiceRequest({}), timeout=10)
        
        # Capture raw image, compressed image, and camera info streams
        topics = {n: k for n, k in zip(roster['topics'], roster['types']) 
                  if k in ['sensor_msgs/Image', 'sensor_msgs/CompressedImage', 'sensor_msgs/CameraInfo']}
        
        if not topics:
            print("No image topics found. Ensure camera node and bridge are running.")
            return

        rclpy.init()
        self.node = rclpy.create_node(f'go1_jetson_{self.host.split(".")[-1]}_collector')
        self.bag_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr')
        )

        for i, (name, kind) in enumerate(sorted(topics.items()), 1):
            ros2_type = kind.replace('/', '/msg/', 1)
            try: 
                msg_class = get_message(ros2_type)
            except Exception: 
                print(f"Skipping {name}, missing ROS 2 message type {ros2_type}")
                continue
                
            self.writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=ros2_type, serialization_format='cdr'))
            self.specs[name] = (msg_class, kind)

            def make_callback(topic_name):
                return lambda data: self.messages.put_nowait((topic_name, data, time.time_ns())) if not self.messages.full() else None
            
            sub = roslibpy.Topic(self.ros, name, kind, throttle_rate=0, queue_length=50)
            sub.subscribe(make_callback(name))
            self.subs.append(sub)

        self.is_recording = True
        print(f"Discovered {len(topics)} vision topics. Recording to: {self.bag_path}\nPress Ctrl+C to stop.")

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

                        # FAST DECODE: Use array.array instead of list() to prevent CPU bottleneck
                        if 'data' in clean_data and isinstance(clean_data['data'], str):
                            raw_bytes = base64.b64decode(clean_data['data'])
                            clean_data['data'] = array.array('B', raw_bytes)

                        msg = cls()
                        set_message_fields(msg, clean_data)
                        self.writer.write(name, serialize_message(msg), rcv_ns)
                        frames_written += 1
                        if frames_written % 10 == 0:
                            print(f"Recorded {frames_written} frames...", end='\r')
                    except Exception as e:
                        print(f"Error packing message on {name}: {e}")
                        
                rclpy.spin_once(self.node, timeout_sec=0.01)
        except KeyboardInterrupt:
            print(f"\nStopping vision collection. Total frames recorded: {frames_written}")
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
    parser.add_argument('--host', type=str, default='192.168.123.14', help="Jetson IP")
    args = parser.parse_args()
    JetsonVisionCollector(host=args.host).run()