import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
import websocket
import json
import threading
import time
import argparse
import base64
import os
from datetime import datetime

# ROS 2 Message Types
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from std_msgs.msg import Header

class RealSenseRecorder:
    def __init__(self, host, output_dir):
        self.url = f"ws://{host}:9090"
        self.ws = None
        self.is_connected = False
        self.msg_count = 0
        
        # Explicitly subscribe to target streams to wake up the RealSense node
        self.target_topics = {
            "/camera/color/image_raw": "sensor_msgs/Image",
            "/camera/depth/image_rect_raw": "sensor_msgs/Image",
            "/camera/color/camera_info": "sensor_msgs/CameraInfo",
            "/camera/depth/camera_info": "sensor_msgs/CameraInfo"
        }
        
        # Setup ROS 2 Bag Writer
        os.makedirs(output_dir, exist_ok=True)
        bag_name = datetime.now().strftime("%Y%m%d_%H%M%S_realsense_capture")
        self.bag_path = os.path.join(output_dir, bag_name)
        
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer.open(storage_options, converter_options)
        print(f"[INFO] Opened ROS 2 Bag for writing: {self.bag_path}")

        # Register topics in the bag
        for topic, msg_type in self.target_topics.items():
            topic_info = rosbag2_py.TopicMetadata(
                id=0,
                name=topic,
                type=msg_type,
                serialization_format='cdr'
            )
            self.writer.create_topic(topic_info)

    def connect(self):
        print(f"[INFO] Connecting to {self.url}...")
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close
        )
        
        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()
        
        while not self.is_connected:
            time.sleep(0.1)

    def _on_open(self, ws):
        self.is_connected = True
        print("[INFO] Websocket connected. Requesting streams...")
        for topic, msg_type in self.target_topics.items():
            subscribe_msg = {
                "op": "subscribe",
                "topic": topic,
                "type": msg_type
            }
            self.ws.send(json.dumps(subscribe_msg))
            print(f"  -> Subscribed to {topic}")

    def _on_message(self, ws, message):
        data = json.loads(message)
        
        if data.get("op") == "publish":
            topic = data.get("topic")
            payload = data.get("msg", {})
            
            try:
                # 1. Parse Image Topics
                if "image" in topic:
                    msg = Image()
                    msg.header = self._parse_header(payload['header'])
                    msg.height = payload['height']
                    msg.width = payload['width']
                    msg.encoding = payload['encoding']
                    msg.is_bigendian = int(payload['is_bigendian'])
                    msg.step = payload['step']
                    # Rosbridge sends binary data as Base64 encoded string
                    msg.data = base64.b64decode(payload['data'])
                
                # 2. Parse CameraInfo Topics (Fixing D vs d)
                elif "camera_info" in topic:
                    msg = CameraInfo()
                    msg.header = self._parse_header(payload['header'])
                    msg.height = payload['height']
                    msg.width = payload['width']
                    msg.distortion_model = payload['distortion_model']
                    
                    # Convert ROS 1 uppercase arrays to ROS 2 lowercase arrays
                    msg.d = payload.get('D', payload.get('d', []))
                    msg.k = payload.get('K', payload.get('k', []))
                    msg.r = payload.get('R', payload.get('r', []))
                    msg.p = payload.get('P', payload.get('p', []))
                    
                    msg.binning_x = payload['binning_x']
                    msg.binning_y = payload['binning_y']
                    
                    roi_data = payload.get('roi', {})
                    roi = RegionOfInterest()
                    roi.x_offset = roi_data.get('x_offset', 0)
                    roi.y_offset = roi_data.get('y_offset', 0)
                    roi.height = roi_data.get('height', 0)
                    roi.width = roi_data.get('width', 0)
                    roi.do_rectify = roi_data.get('do_rectify', False)
                    msg.roi = roi

                # 3. Serialize and write to bag
                timestamp = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
                self.writer.write(topic, serialize_message(msg), timestamp)
                self.msg_count += 1
                
                if self.msg_count % 30 == 0:
                    print(f"\r[INFO] Recorded {self.msg_count} RealSense messages...", end="")

            except Exception as e:
                print(f"\n[ERROR] Failed to pack message on {topic}: {e}")

    def _parse_header(self, header_data):
        h = Header()
        h.stamp.sec = header_data['stamp']['secs']
        h.stamp.nanosec = header_data['stamp']['nsecs']
        h.frame_id = header_data['frame_id']
        return h

    def _on_error(self, ws, error):
        print(f"\n[ERROR] Websocket error: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        self.is_connected = False
        print("\n[INFO] Websocket closed.")

    def stop(self):
        if self.ws:
            self.ws.close()
        print(f"\n[INFO] Capture complete. Saved {self.msg_count} messages.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record RealSense streams to ROS 2 bag")
    parser.add_argument("--host", type=str, default="192.168.123.14", help="IP of the Jetson board")
    parser.add_argument("--out", type=str, default="../Bags/go1_bags", help="Output directory for bag files")
    args = parser.parse_args()

    rclpy.init()

    recorder = RealSenseRecorder(host=args.host, output_dir=args.out)
    
    try:
        recorder.connect()
        print("[INFO] Listening for streams. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[INFO] Stopping capture... waiting 5 seconds for final buffer flush.")
        time.sleep(5)  # Allow any last messages to be processed
        recorder.stop()
    finally:
        # Safely shut down only if it's still running
        if rclpy.ok():
            rclpy.shutdown()