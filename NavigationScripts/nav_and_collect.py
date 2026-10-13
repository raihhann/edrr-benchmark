#!/usr/bin/env python3
"""Navigate the Go1 while recording discovered rosbridge topics into one ROS 2 bag.

Run from any directory, for example:
    python3 "Navigation Scripts/navigate_and_collect.py" --distance 3.0 --speed 0.5 --skip-realsense

By default this connects to the robot Pi (192.168.123.161) and the Jetsons
already used by the project collectors (192.168.123.13, .14, and .15). The
existing bags identify .13 as the front/chin camera source, .15 as the
rear-down camera source, and .14 as the RealSense color/depth source.
Every source must be reachable before the robot is commanded to move.
Duplicate topic names from different hosts are kept separate under
/source_<host>/ in the bag.
"""
import argparse
import array
import base64
import json
import math
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
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
    def __init__(self, ros_by_host, source_hosts, source_port, output_dir, queue_size, skip_topics=SKIP_TOPICS):
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

        output_dir.mkdir(parents=True, exist_ok=True)
        self.bag_path = output_dir / datetime.now().strftime(
            "%Y%m%d_%H%M%S_go1_navigation_capture"
        )

    def discover(self):
        import roslibpy

        found = []
        for host in self.source_hosts:
            if host not in self.ros_by_host or not self.ros_by_host[host].is_connected:
                continue
            ros = self.ros_by_host[host]
            try:
                result = roslibpy.Service(
                    ros, "/rosapi/topics", "rosapi/Topics"
                ).call(roslibpy.ServiceRequest({}), timeout=5)
                for name, type_name in zip(result["topics"], result["types"]):
                    if name.startswith(self.skip_topics):
                        continue
                    found.append((host, name, type_name))
            except Exception as error:
                print(f"[WARN] Failed to query topics from {host}: {error}")

        if not found:
            raise RuntimeError("No recordable topics were discovered on the reachable source hosts.")

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

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
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
            topic = roslibpy.Topic(
                self.ros_by_host[host], name, type_name, throttle_rate=0, queue_length=100
            )
            output_name = self.topic_map[(host, name)]
            topic.subscribe(self._callback(output_name))
            self.subscriptions.append(topic)

    def _callback(self, topic_name):
        def receive(data):
            try:
                self.queue.put_nowait((topic_name, data, time.time_ns()))
            except queue.Full:
                with self.drop_lock:
                    self.messages_dropped += 1

        return receive

    def drain(self):
        from rclpy.serialization import serialize_message
        from rosidl_runtime_py.set_message import set_message_fields

        while True:
            try:
                topic_name, data, received_ns = self.queue.get_nowait()
            except queue.Empty:
                break

            message_class, _, ros1_type, native = self.specs[topic_name]
            try:
                message = message_class()
                if native:
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
                self.writer.write(topic_name, serialize_message(message), received_ns)
                self.messages_written += 1
            except Exception as error:
                self.messages_failed += 1
                if self.messages_failed == 1 or self.messages_failed % 100 == 0:
                    print(
                        f"[ERROR] Could not record {topic_name} "
                        f"({self.messages_failed} failures so far): {error}"
                    )

        if self.messages_dropped - self.last_drop_warning >= 100:
            print(f"[WARN] Queue overflow has dropped {self.messages_dropped} messages.")
            self.last_drop_warning = self.messages_dropped

    def close(self):
        for subscription in self.subscriptions:
            try:
                subscription.unsubscribe()
            except Exception as error:
                print(f"[WARN] Could not unsubscribe from a source topic: {error}")
        self.drain()
        if self.writer is not None:
            self.writer.close()
            self.writer = None
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
            raise RuntimeError("Robot rejected walking mode. Confirm it is safe to walk.")

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
        # Active brake burst: 10 pulses to ensure dog controller drops to 0 m/s
        for _ in range(10):
            try:
                self.publish_velocity(0.0, 0.0)
            except Exception as error:
                print(f"[ERROR] Could not send stop command: {error}")
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
    """Safely terminate roslibpy instances avoiding Python 3.12 autobahn manager bugs."""
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
        description="Navigate the Go1 and record all selected rosbridge sources into one bag."
    )
    parser.add_argument("--distance", type=float, default=3.0, help="Target distance in metres (default: 3.0)")
    parser.add_argument("--speed", type=float, default=0.2, help="Forward speed in m/s (default: 0.2)")
    parser.add_argument("--host", default="192.168.123.161", help="Robot/navigation rosbridge IP")
    parser.add_argument("--port", type=int, default=9090, help="Robot/navigation rosbridge port")
    parser.add_argument(
        "--sources",
        default=",".join(DEFAULT_SOURCES),
        help="Comma-separated data-source IPs (default: Pi .161 and Jetsons .13, .14, .15)",
    )
    parser.add_argument("--source-port", type=int, default=9090, help="Data-source rosbridge port")
    parser.add_argument(
        "--out",
        type=Path,
        default=PROJECT_ROOT / "Bags" / "go1_bags",
        help="Directory in which to create the single ROS 2 bag",
    )
    parser.add_argument("--queue-size", type=int, default=5000, help="Maximum queued messages")
    parser.add_argument("--odom-timeout", type=float, default=20.0, help="Odometry wait timeout")
    parser.add_argument(
        "--skip-realsense",
        action="store_true",
        help="Do not record Intel RealSense topics (color, depth, infra, etc.)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.distance) or args.distance < 0:
        parser.error("--distance must be finite and non-negative")
    if not math.isfinite(args.speed):
        parser.error("--speed must be finite")
    if args.port <= 0 or args.source_port <= 0 or args.queue_size <= 0:
        parser.error("ports and --queue-size must be positive")
    if args.odom_timeout <= 0:
        parser.error("--odom-timeout must be positive")
    args.sources = list(dict.fromkeys(host.strip() for host in args.sources.split(",") if host.strip()))
    if not args.sources:
        parser.error("--sources must contain at least one IP")
    return args


def main():
    args = parse_args()
    import roslibpy

    # Build the list of topics to skip dynamically
    skip_prefixes = list(SKIP_TOPICS)
    if args.skip_realsense:
        skip_prefixes.extend([
            "/camera/color",
            "/camera/depth",
            "/camera/infra",
            "/camera/extrinsics",
            "/camera/gyro",
            "/camera/accel",
            "/camera/motion"
        ])
    skip_prefixes = tuple(skip_prefixes)

    endpoints = {}
    endpoint_keys = {(host, args.source_port) for host in args.sources}
    endpoint_keys.add((args.host, args.port))
    recorder = None
    navigator = None

    try:
        for host, port in sorted(endpoint_keys):
            ros = roslibpy.Ros(host=host, port=port)
            print(f"[INFO] Connecting to rosbridge {host}:{port}...")
            try:
                ros.run(timeout=5)
                if not ros.is_connected:
                    print(f"[WARN] Could not connect to {host}:{port}, skipping.")
                    continue
                endpoints[(host, port)] = ros
            except Exception as e:
                print(f"[WARN] Error connecting to {host}:{port}: {e}, skipping.")

        nav_key = (args.host, args.port)
        if nav_key not in endpoints or not endpoints[nav_key].is_connected:
            raise RuntimeError(f"Required navigation host {args.host}:{args.port} is unreachable.")

        active_sources = [host for host in args.sources if (host, args.source_port) in endpoints]
        source_ros_by_host = {
            host: endpoints[(host, args.source_port)] for host in active_sources
        }

        recorder = MultiSourceRecorder(
            source_ros_by_host,
            active_sources,
            args.source_port,
            args.out.expanduser().resolve(),
            args.queue_size,
            skip_topics=skip_prefixes,
        )
        discovered = recorder.discover()
        recorder.open(discovered)
        recorder.subscribe(discovered)
        print(f"[INFO] Recording {len(discovered)} source topics to {recorder.bag_path}")

        navigator = Go1Navigator(endpoints[(args.host, args.port)], args.host, args.port)
        navigator.connect()

        odom_deadline = time.monotonic() + args.odom_timeout
        while navigator.latest_odom is None and time.monotonic() < odom_deadline:
            recorder.drain()
            time.sleep(0.02)
        if navigator.latest_odom is None:
            raise RuntimeError("Timed out waiting for /go1_controller/odom.")

        navigator.set_walking_mode()
        start_x, start_y, start_heading = navigator.pose()
        print(f"[INFO] Walking {args.distance:.2f} m with nominal speed {args.speed:.2f} m/s.")

        try:
            while True:
                if not endpoints[nav_key].is_connected:
                    raise RuntimeError("Navigation rosbridge host disconnected.")

                recorder.drain()
                current_x, current_y, current_heading = navigator.pose()
                dx = current_x - start_x
                dy = current_y - start_y
                distance = math.hypot(dx, dy)
                remaining = args.distance - distance

                if distance >= args.distance or remaining <= 0.02:
                    print(f"\n[INFO] Target reached: {distance:.2f} m.")
                    break

                # Dynamic Braking Ramp (decelerate over the last 0.4 m)
                decel_zone = 0.4
                if remaining < decel_zone:
                    effective_speed = max(0.08, args.speed * (remaining / decel_zone))
                else:
                    effective_speed = args.speed

                lateral_drift = -dx * math.sin(start_heading) + dy * math.cos(start_heading)
                heading_error = math.atan2(
                    math.sin(current_heading - start_heading),
                    math.cos(current_heading - start_heading),
                )
                yaw = max(-0.15, min(0.15, -0.8 * heading_error - 0.3 * lateral_drift))

                navigator.publish_velocity(effective_speed, yaw)
                print(
                    f"\r[INFO] Progress: {distance:.2f} / {args.distance:.2f} m (Speed: {effective_speed:.2f} m/s)",
                    end="",
                )
                time.sleep(0.05)  # 20 Hz control loop for tighter responsiveness

        finally:
            navigator.stop()

        # Buffer Flush Phase: 5 seconds to clear remaining network queue
        print("\n[INFO] Holding for 5 seconds to flush data buffers...")
        flush_deadline = time.monotonic() + 5.0
        while time.monotonic() < flush_deadline:
            recorder.drain()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] Capture/navigation interrupted.")
    finally:
        if navigator is not None:
            navigator.close()

        if recorder is not None:
            recorder.close()
            print(
                f"[INFO] Bag finalized at {recorder.bag_path}; "
                f"wrote {recorder.messages_written} messages, "
                f"failed {recorder.messages_failed}, dropped {recorder.messages_dropped}."
            )

        for ros in endpoints.values():
            safe_terminate_ros(ros)


if __name__ == "__main__":
    main()