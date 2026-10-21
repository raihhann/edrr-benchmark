#!/usr/bin/env python3
"""Navigate the Go1 while recording discovered rosbridge topics into one ROS 2 bag.

Run from any directory, for example:
    python3 nav_and_collect_v3.py --distance 3.0 --speed 0.5 --skip-realsense
    python3 nav_and_collect_v3.py --distance 3.0 --speed 0.5 --skip-lidar
"""
import argparse
from collections import Counter
import sys
import array
import base64
import json
import math
import queue
import threading
import time
import statistics
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "DataCollection"))
from sync_capture import source_stamp_ns, ntp_offset, compressed_message
DEFAULT_SOURCES = (
    "192.168.123.161",
    "192.168.123.13",
    "192.168.123.14",
    "192.168.123.15",
)
SKIP_TOPICS = ("/rosout", "/connected_clients")
BYTE_ARRAY_TYPES = {
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage",
    "sensor_msgs/PointCloud2",
}

IMAGE_TYPES = {'sensor_msgs/Image', 'sensor_msgs/CompressedImage'}


def select_recording_topics(topics, transport='auto'):
    """Prefer one compressed stream, falling back per camera rather than per host."""
    selected, raw_fallbacks = {}, []
    for name, kind in topics.items():
        if any(part in name for part in ('/theora', '/compressedDepth',
                                         '/parameter_descriptions', '/parameter_updates')):
            continue
        if kind == 'sensor_msgs/Image':
            compressed = name + '/compressed'
            if '/depth/' in name or 'depth_' in name or transport == 'raw':
                selected[name] = kind
            elif topics.get(compressed) == 'sensor_msgs/CompressedImage':
                selected[compressed] = topics[compressed]
            elif transport == 'compressed':
                raise RuntimeError('Compressed image required but missing: ' + compressed)
            else:
                selected[name] = kind
                raw_fallbacks.append(name)
        elif kind == 'sensor_msgs/CompressedImage':
            base = name.rsplit('/', 1)[0]
            if name.endswith('/compressed') and topics.get(base) != 'sensor_msgs/Image':
                selected[name] = kind
        else:
            selected[name] = kind
    return selected, raw_fallbacks


def raw_image_message(data):
    """Decode bytes directly, avoiding recursive conversion of a million pixels."""
    from sensor_msgs.msg import Image
    message = Image()
    stamp = source_stamp_ns(data)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
    message.header.frame_id = data.get('header', {}).get('frame_id', '')
    for key in ('height', 'width', 'encoding', 'is_bigendian', 'step'):
        setattr(message, key, data[key])
    payload = data['data']
    raw = base64.b64decode(payload, validate=True) if isinstance(payload, str) else bytes(payload)
    if len(raw) != message.height * message.step:
        raise ValueError('Raw image size does not equal height * step')
    message.data = array.array('B', raw)
    return message


def ros1_to_ros2_type(type_name):
    package, separator, message = type_name.partition("/")
    if not separator:
        raise ValueError(f"Invalid ROS 1 message type: {type_name}")
    return f"{package}/msg/{message}"


def convert_rosbridge(value, ros1_type=""):
    if isinstance(value, list):
        return [convert_rosbridge(item, ros1_type) for item in value]
    if not isinstance(value, dict):
        return value
    if set(value) == {"secs", "nsecs"}:
        return {"sec": value["secs"], "nanosec": value["nsecs"]}

    result = {}
    for key, item in value.items():
        if key == "seq":
            continue
        output_key = key.lower() if key in {"D", "K", "R", "P"} else key
        if key == "data" and ros1_type in BYTE_ARRAY_TYPES:
            if isinstance(item, str):
                item = array.array("B", base64.b64decode(item, validate=True))
            elif isinstance(item, list):
                item = array.array("B", item)
        result[output_key] = convert_rosbridge(item, ros1_type)
    return result


def restore_float_nulls(values, message, path="message"):
    from rosidl_parser.definition import AbstractNestedType, BasicType, NamespacedType
    from rosidl_runtime_py.convert import get_message_slot_types
    from rosidl_runtime_py.import_message import import_message_from_namespaced_type

    slots = get_message_slot_types(message)

    def restore(value, slot, field):
        if isinstance(slot, BasicType):
            if value is None:
                if slot.typename in {"float", "double", "float32", "float64"}:
                    return float("nan")
                raise ValueError(f"Null in non-floating-point field {field}")
            return value
        if value is None:
            raise ValueError(f"Null in {field}")
        if isinstance(slot, AbstractNestedType):
            return [
                restore(item, slot.value_type, f"{field}[{index}]")
                for index, item in enumerate(value)
            ]
        if isinstance(slot, NamespacedType):
            nested = import_message_from_namespaced_type(slot)()
            return restore_float_nulls(value, nested, field)
        return value

    return {
        name: restore(value, slots[name], f"{path}.{name}")
        for name, value in values.items()
    }


def source_slug(host, port):
    return f"{host.replace('.', '_').replace(':', '_')}_{port}"


class MultiSourceRecorder:
    def __init__(self, ros_by_host, source_hosts, source_port, output_dir, queue_size,
                 skip_topics=SKIP_TOPICS, camera_transport='auto'):
        self.ros_by_host = ros_by_host
        self.source_hosts = source_hosts
        self.source_port = source_port
        self.skip_topics = skip_topics
        self.queue = queue.Queue(maxsize=queue_size)
        self.subscriptions = []
        self.specs = {}
        self.topic_map = {}
        self.writer = None
        self.rclpy = None
        self.node = None
        self.messages_written = 0
        self.messages_failed = 0
        self.messages_dropped = 0
        self.last_drop_warning = 0
        self.drop_lock = threading.Lock()
        self.camera_transport = camera_transport
        self.topic_counts = Counter()
        self.topic_timing = {}
        self.image_topics = set()
        self.required_live_topics = set()
        self.last_written_monotonic = {}
        self.status = 'preparing'
        self.error = None
        self.closed = False
        self.source_hosts_with_images = set()

        output_dir.mkdir(parents=True, exist_ok=True)
        self.bag_path = output_dir / datetime.now().strftime(
            "%Y%m%d_%H%M%S_go1_navigation_capture"
        )

    def discover(self):
        import roslibpy

        found = []
        for host in self.source_hosts:
            if host not in self.ros_by_host or not self.ros_by_host[host].is_connected:
                raise RuntimeError('Requested recording source is disconnected: '+host)
            ros = self.ros_by_host[host]
            try:
                result = roslibpy.Service(
                    ros, "/rosapi/topics", "rosapi/Topics"
                ).call(roslibpy.ServiceRequest({}), timeout=5)
                roster = {name: kind for name, kind in zip(result['topics'], result['types'])
                          if not name.startswith(self.skip_topics)}
                selected, fallbacks = select_recording_topics(roster, self.camera_transport)
                for name in fallbacks:
                    print(f'[WARN] {host}: recording raw fallback {name}; no compressed topic is advertised. '
                          'Raw transfer may reduce FPS or increase latency.', flush=True)
                for name, type_name in selected.items():
                    if name.startswith(self.skip_topics):
                        continue
                    found.append((host, name, type_name))
            except Exception as error:
                raise RuntimeError(f'Cannot discover recording source {host}: {error}') from error

        if not found:
            raise RuntimeError("No recordable topics were discovered on reachable hosts.")

        name_counts = {}
        for _, name, _ in found:
            name_counts[name] = name_counts.get(name, 0) + 1

        reserved_names = set(name_counts)
        for host, name, type_name in found:
            output_name = name
            if name_counts[name] > 1:
                root = f"/source_{source_slug(host, self.source_port)}/{name.lstrip('/')}"
                output_name = root
                suffix = 2
                while output_name in reserved_names or output_name in self.topic_map.values():
                    output_name = f"{root}_{suffix}"
                    suffix += 1
            self.topic_map[(host, name)] = output_name
            self.specs[output_name] = self._make_spec(type_name)
            if type_name in IMAGE_TYPES:
                self.image_topics.add(output_name)
                self.source_hosts_with_images.add(host)
            print(f'[INFO] Selected {host} {name} [{type_name}] -> {output_name}', flush=True)

        return found

    @staticmethod
    def _make_spec(type_name):
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        try:
            ros2_type = ros1_to_ros2_type(type_name)
            return get_message(ros2_type), ros2_type, type_name, True
        except (ImportError, AttributeError, ModuleNotFoundError, ValueError):
            return String, "std_msgs/msg/String", type_name, False

    def open(self, discovered):
        import rosbag2_py
        import rclpy

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("go1_navigation_data_recorder")

        self.clock_reference = ntp_offset("192.168.123.161")
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        self.timing_file = (self.bag_path / "timing.jsonl").open("w")
        (self.bag_path / "clock_reference.json").write_text(json.dumps(self.clock_reference, indent=2))
        registered = set()
        for index, (host, name, _) in enumerate(discovered, start=1):
            output_name = self.topic_map[(host, name)]
            _, ros2_type, _, _ = self.specs[output_name]
            if output_name in registered:
                continue
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=index,
                    name=output_name,
                    type=ros2_type,
                    serialization_format="cdr",
                )
            )
            registered.add(output_name)

    def subscribe(self, discovered):
        import roslibpy

        for host, name, type_name in discovered:
            # Set high queue length for PointCloud2 to prevent buffer starvation
            q_len = 200 if "PointCloud2" in type_name else 50
            topic = roslibpy.Topic(
                self.ros_by_host[host], name, type_name, throttle_rate=0, queue_length=q_len
            )
            output_name = self.topic_map[(host, name)]
            topic.subscribe(self._callback(output_name))
            self.subscriptions.append(topic)

    def _callback(self, topic_name):
        def receive(data):
            try:
                self.queue.put_nowait((topic_name, data, time.time_ns(), time.monotonic()))
            except queue.Full:
                with self.drop_lock:
                    self.messages_dropped += 1

        return receive

    def drain(self):
        from rclpy.serialization import serialize_message
        from rosidl_runtime_py.set_message import set_message_fields

        drain_deadline = time.monotonic() + 0.02
        while time.monotonic() < drain_deadline:
            try:
                topic_name, data, received_ns, received_monotonic = self.queue.get_nowait()
            except queue.Empty:
                break

            message_class, _, ros1_type, native = self.specs[topic_name]
            source_ns = source_stamp_ns(data)
            corrected_receipt = round(received_ns + self.clock_reference["offset_s"] * 1e9)
            age_ms = (corrected_receipt - source_ns) / 1e6 if source_ns else None
            try:
                message = message_class()
                if ros1_type == "sensor_msgs/CompressedImage":
                    message = compressed_message(data)
                elif ros1_type == 'sensor_msgs/Image':
                    message = raw_image_message(data)
                elif native:
                    normalized = convert_rosbridge(data, ros1_type)
                    fields = restore_float_nulls(normalized, message, topic_name)
                    set_message_fields(message, fields)
                else:
                    from std_msgs.msg import String

                    message = String(
                        data=json.dumps(
                            {"ros1_type": ros1_type, "message": data},
                            separators=(",", ":"),
                        )
                    )
                bag_stamp = source_ns or corrected_receipt
                self.writer.write(topic_name, serialize_message(message), bag_stamp)
                self.timing_file.write(json.dumps({"topic": topic_name, "source_ns": source_ns, "arrival_ns": received_ns, "bag_timestamp_ns": bag_stamp, "age_ms": age_ms, "seq": data.get("header", {}).get("seq")}) + "\n")
                self.messages_written += 1
                self.topic_counts[topic_name] += 1
                self.last_written_monotonic[topic_name] = received_monotonic
                stat = self.topic_timing.setdefault(topic_name, {'first_arrival_ns': received_ns,
                    'last_arrival_ns': received_ns, 'first_source_ns': source_ns,
                    'last_source_ns': source_ns, 'last_seq': None, 'sequence_gaps': 0,
                    'age_ms': [], 'missing_source_stamps': 0})
                seq = data.get('header', {}).get('seq')
                if seq is not None and stat['last_seq'] is not None and seq > stat['last_seq']:
                    stat['sequence_gaps'] += max(0, seq - stat['last_seq'] - 1)
                stat.update(last_arrival_ns=received_ns, last_source_ns=source_ns, last_seq=seq)
                if age_ms is not None: stat['age_ms'].append(age_ms)
                elif topic_name in self.image_topics: stat['missing_source_stamps'] += 1
            except Exception as error:
                self.messages_failed += 1
                if self.messages_failed == 1 or self.messages_failed % 100 == 0:
                    print(
                        f"[ERROR] Could not record {topic_name} "
                        f"({self.messages_failed} failures so far): {error}"
                    )
                raise RuntimeError(f'Recording failed for {topic_name}: {error}') from error

        if self.messages_dropped - self.last_drop_warning >= 50:
            print(f"[WARN] Queue overflow dropped {self.messages_dropped} messages.")
            self.last_drop_warning = self.messages_dropped

    def configure_requirements(self, allow_no_video=False):
        if not self.image_topics and not allow_no_video:
            raise RuntimeError('No camera image topics selected. Start the cameras, or explicitly use --allow-no-video.')
        # Do not silently turn a requested Jetson camera source into a telemetry-only run.
        if not allow_no_video:
            missing = [host for host in self.source_hosts
                       if host in DEFAULT_SOURCES[1:] and host not in self.source_hosts_with_images]
            if missing: raise RuntimeError('No image topics on requested camera sources: '+str(missing))
        self.required_live_topics = set(self.image_topics)
        for (host, original), output in self.topic_map.items():
            if original in ('/go1_controller/odom', '/go1_controller/imu/data'):
                self.required_live_topics.add(output)

    def check_health(self, require_ready=True):
        if self.messages_failed or self.messages_dropped:
            raise RuntimeError(f'Recording data loss: failed={self.messages_failed}, dropped={self.messages_dropped}')
        disconnected = [host for host, ros in self.ros_by_host.items() if not ros.is_connected]
        if disconnected: raise RuntimeError('Recording sources disconnected: '+str(disconnected))
        missing = [topic for topic in self.required_live_topics if self.topic_counts[topic] <
                   (2 if topic in self.image_topics else 1)]
        if missing:
            if require_ready: raise RuntimeError('Required streams have not delivered messages: '+str(sorted(missing)))
            return False
        now = time.monotonic()
        stale = [topic for topic in self.required_live_topics
                 if now-self.last_written_monotonic.get(topic, 0) > (2. if topic in self.image_topics else .5)]
        if stale:
            if require_ready: raise RuntimeError('Required streams are stale/backlogged: '+str(sorted(stale)))
            return False
        for topic in self.image_topics:
            stat = self.topic_timing[topic]
            if stat['missing_source_stamps']:
                raise RuntimeError('Camera has no usable source timestamps: '+topic)
            if stat['age_ms'] and abs(stat['age_ms'][-1]) > 1000:
                raise RuntimeError(f'Camera clock/delivery delay exceeds 1 second: {topic}. '
                                   'Raw fallback may require robot-side compression for a usable run.')
        return True

    def wait_until_ready(self, timeout=15.):
        deadline = time.monotonic()+timeout
        stable_since = None
        while time.monotonic() < deadline:
            self.drain()
            if self.check_health(require_ready=False):
                if stable_since is None: stable_since = time.monotonic()
                if time.monotonic()-stable_since >= 1.:
                    print('[INFO] Camera/telemetry preflight passed; actual messages verified.', flush=True)
                    return
            else:
                stable_since = None
            time.sleep(.002)
        self.check_health()
        raise RuntimeError('Preflight timeout')

    def close(self):
        if self.closed: return
        self.closed = True
        for subscription in self.subscriptions:
            try:
                subscription.unsubscribe()
            except Exception as error:
                print(f"[WARN] Could not unsubscribe: {error}")
        flush_deadline = time.monotonic() + 5
        while not self.queue.empty() and time.monotonic() < flush_deadline:
            try: self.drain()
            except Exception as error:
                self.status, self.error = 'aborted', str(error)
                break
        if hasattr(self, "timing_file"):
            self.timing_file.close()
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        if self.queue.qsize():
            print(f"[WARN] {self.queue.qsize()} messages were not written at shutdown")
        if self.bag_path.exists():
            report = {'status': self.status, 'error': self.error,
                'camera_transport': self.camera_transport, 'source_hosts': self.source_hosts,
                'clock_reference': getattr(self, 'clock_reference', None),
                'bag_time': 'source header time; Pi-corrected receipt time for messages without headers',
                'hardware_synchronization': False, 'messages_written': self.messages_written,
                'conversion_failures': self.messages_failed, 'local_queue_drops': self.messages_dropped,
                'unprocessed_queue_messages': self.queue.qsize(), 'topics': {}}
            for topic, (_, _, original_type, _) in self.specs.items():
                stat = self.topic_timing.get(topic, {})
                count = self.topic_counts[topic]
                def rate(first, last): return (count-1)*1e9/(last-first) if count>1 and last>first else None
                ages = sorted(stat.get('age_ms', []))
                report['topics'][topic] = {'ros1_type': original_type, 'messages': count,
                    'image': topic in self.image_topics,
                    'arrival_hz': rate(stat.get('first_arrival_ns',0),stat.get('last_arrival_ns',0)),
                    'source_hz': rate(stat.get('first_source_ns',0),stat.get('last_source_ns',0)),
                    'source_sequence_gaps': stat.get('sequence_gaps',0),
                    'median_age_ms': statistics.median(ages) if ages else None,
                    'max_age_ms': max(ages) if ages else None}
            report['topics_without_messages'] = [topic for topic in self.specs if not self.topic_counts[topic]]
            report['image_topics_with_sequence_gaps'] = [topic for topic in self.image_topics
                if self.topic_timing.get(topic,{}).get('sequence_gaps',0)]
            if report['status']=='complete' and (report['topics_without_messages'] or
                report['image_topics_with_sequence_gaps'] or self.messages_dropped or self.queue.qsize()):
                report['status']='partial'
            (self.bag_path/'capture_report.json').write_text(json.dumps(report,indent=2))
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()


class Go1Navigator:
    def __init__(self, ros, host, port):
        self.ros = ros
        self.host = host
        self.port = port
        self.command = None
        self.odom_sub = None
        self.latest_odom = None

    def connect(self):
        import roslibpy

        if not self.ros.is_connected:
            raise RuntimeError(f"Navigation host {self.host}:{self.port} is disconnected.")
        self.command = roslibpy.Topic(
            self.ros, "/go1_controller/cmd_vel", "geometry_msgs/Twist"
        )
        self.odom_sub = roslibpy.Topic(
            self.ros, "/go1_controller/odom", "nav_msgs/Odometry"
        )
        self.odom_sub.subscribe(lambda message: setattr(self, "latest_odom", message))

    def pose(self):
        position = self.latest_odom["pose"]["pose"]["position"]
        orientation = self.latest_odom["pose"]["pose"]["orientation"]
        heading = math.atan2(
            2.0 * (orientation["w"] * orientation["z"] + orientation["x"] * orientation["y"]),
            1.0 - 2.0 * (orientation["y"] ** 2 + orientation["z"] ** 2),
        )
        return position["x"], position["y"], heading

    def set_walking_mode(self):
        import roslibpy

        result = roslibpy.Service(
            self.ros, "/set_mode", "go1_legged_msgs/SetMode"
        ).call(roslibpy.ServiceRequest({"mode": 2, "gait_type": 1}), timeout=5)
        if not result or not result.get("success"):
            raise RuntimeError("Robot rejected walking mode. Confirm safe positioning.")

    def publish_velocity(self, speed, yaw):
        import roslibpy

        if self.command is None:
            return
        self.command.publish(
            roslibpy.Message(
                {
                    "linear": {"x": float(speed), "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": float(yaw)},
                }
            )
        )

    def stop(self):
        if self.command is None:
            return
        for _ in range(10):
            try:
                self.publish_velocity(0.0, 0.0)
            except Exception:
                break
            time.sleep(0.05)

    def close(self):
        self.stop()
        time.sleep(0.1)
        if self.odom_sub is not None:
            try:
                self.odom_sub.unsubscribe()
            except Exception:
                pass
        if self.command is not None:
            try:
                self.command.unadvertise()
            except Exception:
                pass


def safe_terminate_ros(ros_instance):
    try:
        if ros_instance.is_connected:
            ros_instance.close()
    except Exception:
        pass
    try:
        ros_instance.terminate()
    except Exception:
        pass


def parse_args():
    parser = argparse.ArgumentParser(
        description="Navigate Go1 and record discovered rosbridge topics including LiDAR."
    )
    parser.add_argument("--record-only", action="store_true", help="Stationary capture; never create a motion publisher")
    parser.add_argument("--duration", type=float, default=30., help="Stationary recording duration in seconds")
    parser.add_argument("--distance", type=float, default=2.0, help="Target distance in metres")
    parser.add_argument("--speed", type=float, default=0.2, help="Forward speed in m/s")
    parser.add_argument("--host", default="192.168.123.161", help="Robot/navigation rosbridge IP")
    parser.add_argument("--port", type=int, default=9090, help="Robot/navigation rosbridge port")
    parser.add_argument(
        "--sources",
        default=",".join(DEFAULT_SOURCES),
        help="Comma-separated source IPs (Pi .161, Jetsons .13, .14, .15)",
    )
    parser.add_argument("--source-port", type=int, default=9090, help="Data-source rosbridge port")
    parser.add_argument(
        "--out",
        type=Path,
        default=PROJECT_ROOT / "Bags" / "go1_bags",
        help="Directory to save the ROS 2 bag",
    )
    parser.add_argument("--queue-size", type=int, default=10000, help="Internal queue size (10000 recommended for LiDAR)")
    parser.add_argument('--camera-transport', choices=('auto','compressed','raw'), default='auto',
                        help='auto prefers compressed and records raw when compression is absent; never skips a whole camera source')
    parser.add_argument('--startup-timeout', type=float, default=15.,
                        help='Seconds to wait for actual camera and telemetry messages before movement')
    parser.add_argument('--allow-no-video', action='store_true',
                        help='Explicitly permit requested camera hosts with no image topics')
    parser.add_argument("--odom-timeout", type=float, default=20.0, help="Odometry timeout")
    parser.add_argument(
        "--skip-realsense",
        action="store_true",
        help="Skip recording Intel RealSense topics",
    )
    parser.add_argument(
        "--skip-lidar",
        action="store_true",
        help="Skip recording RoboSense LiDAR topics (/rslidar_points, /rslidar_packets)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.distance) or args.distance < 0:
        parser.error("--distance must be non-negative")
    if not math.isfinite(args.speed):
        parser.error("--speed must be finite")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error('--duration must be in (0,3600]')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('--startup-timeout must be positive')
    if args.queue_size <= 0 or args.port <= 0 or args.source_port <= 0:
        parser.error('queue-size and ports must be positive')
    args.sources = list(dict.fromkeys(h.strip() for h in args.sources.split(",") if h.strip()))
    return args


def main():
    args = parse_args()
    import roslibpy

    skip_prefixes = list(SKIP_TOPICS)
    if args.skip_realsense:
        skip_prefixes.extend([
            "/camera/color", "/camera/depth", "/camera/infra",
            "/camera/extrinsics", "/camera/gyro", "/camera/accel", "/camera/motion"
        ])
    if args.skip_lidar:
        skip_prefixes.extend(["/rslidar_points", "/rslidar_packets"])
    skip_prefixes = tuple(skip_prefixes)

    endpoints = {}
    endpoint_keys = {(h, args.source_port) for h in args.sources}
    endpoint_keys.add((args.host, args.port))
    recorder = None
    navigator = None
    attempted_connections = []

    try:
        for host, port in sorted(endpoint_keys):
            ros = roslibpy.Ros(host=host, port=port)
            attempted_connections.append(ros)
            print(f"[INFO] Connecting to rosbridge {host}:{port}...")
            try:
                ros.run(timeout=5)
                if not ros.is_connected:
                    raise RuntimeError(f'Requested rosbridge {host}:{port} is unreachable')
                endpoints[(host, port)] = ros
            except Exception as e:
                raise RuntimeError(f'Connection error on required source {host}:{port}: {e}') from e

        nav_key = (args.host, args.port)
        if nav_key not in endpoints or not endpoints[nav_key].is_connected:
            raise RuntimeError(f"Navigation host {args.host}:{args.port} is unreachable.")

        active_sources = [h for h in args.sources if (h, args.source_port) in endpoints]
        source_ros_by_host = {h: endpoints[(h, args.source_port)] for h in active_sources}

        recorder = MultiSourceRecorder(
            source_ros_by_host,
            active_sources,
            args.source_port,
            args.out.expanduser().resolve(),
            args.queue_size,
            skip_topics=skip_prefixes,
            camera_transport=args.camera_transport,
        )
        discovered = recorder.discover()
        recorder.configure_requirements(args.allow_no_video)
        recorder.open(discovered)
        recorder.subscribe(discovered)
        print(f"[INFO] Recording {len(discovered)} source topics to {recorder.bag_path}")
        recorder.wait_until_ready(args.startup_timeout)

        if args.record_only:
            until = time.monotonic() + args.duration
            print(f"[INFO] Stationary source-timed recording for {args.duration:.1f}s; no motion commands.")
            while time.monotonic() < until:
                recorder.drain()
                recorder.check_health()
                time.sleep(.002)
            recorder.status = 'complete'
            return

        navigator = Go1Navigator(endpoints[(args.host, args.port)], args.host, args.port)
        navigator.connect()

        odom_deadline = time.monotonic() + args.odom_timeout
        while navigator.latest_odom is None and time.monotonic() < odom_deadline:
            recorder.drain()
            recorder.check_health()
            time.sleep(0.02)
        if navigator.latest_odom is None:
            raise RuntimeError("Timed out waiting for /go1_controller/odom.")

        navigator.set_walking_mode()
        start_x, start_y, start_heading = navigator.pose()
        print(f"[INFO] Walking {args.distance:.2f} m at {args.speed:.2f} m/s.")

        try:
            while True:
                if not endpoints[nav_key].is_connected:
                    raise RuntimeError("Navigation rosbridge host disconnected.")

                recorder.drain()
                recorder.check_health()
                current_x, current_y, current_heading = navigator.pose()
                dx = current_x - start_x
                dy = current_y - start_y
                distance = math.hypot(dx, dy)
                remaining = args.distance - distance

                if distance >= args.distance or remaining <= 0.02:
                    print(f"\n[INFO] Target reached: {distance:.2f} m.")
                    break

                decel_zone = 0.4
                effective_speed = max(0.08, args.speed * (remaining / decel_zone)) if remaining < decel_zone else args.speed

                lateral_drift = -dx * math.sin(start_heading) + dy * math.cos(start_heading)
                heading_error = math.atan2(
                    math.sin(current_heading - start_heading),
                    math.cos(current_heading - start_heading),
                )
                yaw = max(-0.15, min(0.15, -0.8 * heading_error - 0.3 * lateral_drift))

                navigator.publish_velocity(effective_speed, yaw)
                print(f"\r[INFO] Progress: {distance:.2f} / {args.distance:.2f} m", end="")
                time.sleep(0.05)
        finally:
            navigator.stop()

        print("\n[INFO] Holding 5 seconds to flush high-bandwidth buffers...")
        flush_deadline = time.monotonic() + 5.0
        while time.monotonic() < flush_deadline:
            recorder.drain()
            time.sleep(0.05)
        recorder.status = 'complete'

    except KeyboardInterrupt:
        if recorder is not None: recorder.status = 'interrupted'
        print("\n[INFO] Interrupted by user.")
    except Exception as error:
        if recorder is not None:
            recorder.status, recorder.error = 'aborted', str(error)
        raise
    finally:
        if navigator is not None:
            navigator.close()
        if recorder is not None:
            recorder.close()
            print(
                f"[INFO] Bag finalized: {recorder.bag_path}\n"
                f"       Wrote: {recorder.messages_written} | Failed: {recorder.messages_failed} | Dropped: {recorder.messages_dropped}"
            )
        connection_list = attempted_connections
        for ros in connection_list:
            if ros.is_connected:
                ros.close()
        if connection_list:
            safe_terminate_ros(connection_list[0])


if __name__ == "__main__":
    main()
