#!/usr/bin/env python3
"""
Go1 Data Collection Library
Continuously records all onboard Raspberry Pi sensor data to a ROS 2 bag.
"""
import argparse, base64, json, queue, time
from datetime import datetime
from pathlib import Path

def convert(value, topic_type=""):
    if isinstance(value, list): return [convert(v, topic_type) for v in value]
    if not isinstance(value, dict): return value
    if set(value) == {'secs', 'nsecs'}: return {'sec': value['secs'], 'nanosec': value['nsecs']}
    res = {}
    for k, v in value.items():
        if k == 'seq': continue
        out_k = k.lower() if k in ('D', 'K', 'R', 'P') else k
        if k == 'data' and isinstance(v, str) and topic_type != 'std_msgs/String':
            try: v = list(base64.b64decode(v, validate=True))
            except: pass
        res[out_k] = convert(v, topic_type)
    return res

def restore_float_nulls(values, message, path='message'):
    from rosidl_runtime_py.convert import get_message_slot_types
    from rosidl_runtime_py.import_message import import_message_from_namespaced_type
    from rosidl_parser.definition import AbstractNestedType, BasicType, NamespacedType

    def restore(val, slot, field):
        if isinstance(slot, BasicType):
            if val is None:
                if slot.typename in ('float', 'double', 'float32', 'float64'): return float('nan')
                raise ValueError(f'Null in {field}')
            return val
        if val is None: raise ValueError(f'Null in {field}')
        if isinstance(slot, AbstractNestedType): return [restore(item, slot.value_type, f'{field}[{i}]') for i, item in enumerate(val)]
        if isinstance(slot, NamespacedType): return restore_float_nulls(val, import_message_from_namespaced_type(slot)(), field)
        return val

    slots = get_message_slot_types(message)
    return {n: restore(v, slots[n], f"{path}.{n}") for n, v in values.items()}

class Go1DataCollector:
    def __init__(self, host='192.168.123.161', port=9090):
        self.host, self.port = host, port
        self.bag_path = Path('../Bags/go1_bags').expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_rpi_data')
        self.ros = self.node = self.writer = None
        self.subs, self.specs = [], {}
        self.messages = queue.Queue(maxsize=2000)
        self.is_recording = False

    def run(self):
        import roslibpy, rclpy, rosbag2_py
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import serialize_message
        from rosidl_runtime_py.set_message import set_message_fields
        from std_msgs.msg import String

        self.ros = roslibpy.Ros(host=self.host, port=self.port)
        self.ros.run(timeout=10)
        
        roster = roslibpy.Service(self.ros, '/rosapi/topics', 'rosapi/Topics').call(roslibpy.ServiceRequest({}), timeout=10)
        topics = {n: k for n, k in zip(roster['topics'], roster['types']) if not n.startswith(('/rosout', '/connected_clients'))}
        
        rclpy.init()
        self.node = rclpy.create_node('go1_rpi_collector')
        self.bag_path.parent.mkdir(parents=True, exist_ok=True)
        
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))

        for i, (name, kind) in enumerate(sorted(topics.items()), 1):
            native = kind.startswith(('sensor_msgs/', 'nav_msgs/', 'geometry_msgs/', 'tf2_msgs/', 'std_msgs/'))
            ros2_type = kind.replace('/', '/msg/', 1) if native else 'std_msgs/msg/String'
            
            try: 
                msg_class = get_message(ros2_type)
            except Exception: 
                # Fallback: If laptop lacks the specific ROS 2 message definition, record as JSON string
                native = False
                ros2_type = 'std_msgs/msg/String'
                msg_class = get_message(ros2_type)
                
            self.writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=ros2_type, serialization_format='cdr'))
            self.specs[name] = (msg_class, native, kind)

            def make_callback(topic_name):
                return lambda data: self.messages.put_nowait((topic_name, data, time.time_ns())) if not self.messages.full() else None
            
            sub = roslibpy.Topic(self.ros, name, kind, throttle_rate=0, queue_length=100)
            sub.subscribe(make_callback(name))
            self.subs.append(sub)

        self.is_recording = True
        print(f"Discovered {len(topics)} topics. Recording to: {self.bag_path}\nPress Ctrl+C to stop.")

        try:
            while self.is_recording and self.ros.is_connected:
                while not self.messages.empty():
                    name, data, rcv_ns = self.messages.get()
                    if name not in self.specs: continue
                    cls, native, ros1_type = self.specs[name]
                    if native:
                        msg = cls()
                        try: set_message_fields(msg, restore_float_nulls(convert(data, ros1_type), msg, name))
                        except: continue
                    else:
                        msg = String(data=json.dumps({'ros1_type': ros1_type, 'message': data}))
                    self.writer.write(name, serialize_message(msg), rcv_ns)
                rclpy.spin_once(self.node, timeout_sec=0.01)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.stop()

    def stop(self):
        self.is_recording = False
        for sub in self.subs: sub.unsubscribe()
        if self.writer: self.writer.close()
        if self.node: 
            import rclpy
            self.node.destroy_node()
            if rclpy.ok():             # Add this check to prevent double-shutdown
                rclpy.shutdown()
        if self.ros: self.ros.terminate()

if __name__ == '__main__':
    Go1DataCollector().run()