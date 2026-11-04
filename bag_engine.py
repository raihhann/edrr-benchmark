#!/usr/bin/env python3
"""
ROS 2 SQLite3 (.db3) Bag Parsing, CDR Deserialization, and Synchronization Engine.
Supports multi-robot bags (Unitree Go1, TurtleBot3, and generic ROS 2 SQLite3 bags)
without requiring rclpy or compiled custom ROS message packages.
"""

import bisect
import json
import math
import os
import sqlite3
import struct
import threading
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


class CDRReader:
    """Fast Common Data Representation (CDR) deserializer for ROS 2 messages."""

    __slots__ = ("data", "endian", "pos", "length")

    def __init__(self, data: bytes):
        self.data = data
        self.length = len(data)
        if self.length < 4:
            self.endian = "<"
            self.pos = self.length
        else:
            self.endian = "<" if data[1] == 1 else ">"
            self.pos = 4

    def align(self, n: int) -> None:
        rel = self.pos - 4
        rem = rel % n
        if rem:
            self.pos += n - rem

    def u8(self) -> int:
        v = self.data[self.pos]
        self.pos += 1
        return v

    def i8(self) -> int:
        v = struct.unpack_from(self.endian + "b", self.data, self.pos)[0]
        self.pos += 1
        return v

    def bool(self) -> bool:
        return bool(self.u8())

    def u16(self) -> int:
        self.align(2)
        v = struct.unpack_from(self.endian + "H", self.data, self.pos)[0]
        self.pos += 2
        return v

    def i16(self) -> int:
        self.align(2)
        v = struct.unpack_from(self.endian + "h", self.data, self.pos)[0]
        self.pos += 2
        return v

    def i32(self) -> int:
        self.align(4)
        v = struct.unpack_from(self.endian + "i", self.data, self.pos)[0]
        self.pos += 4
        return v

    def u32(self) -> int:
        self.align(4)
        v = struct.unpack_from(self.endian + "I", self.data, self.pos)[0]
        self.pos += 4
        return v

    def i64(self) -> int:
        self.align(8)
        v = struct.unpack_from(self.endian + "q", self.data, self.pos)[0]
        self.pos += 8
        return v

    def u64(self) -> int:
        self.align(8)
        v = struct.unpack_from(self.endian + "Q", self.data, self.pos)[0]
        self.pos += 8
        return v

    def f32(self) -> float:
        self.align(4)
        v = struct.unpack_from(self.endian + "f", self.data, self.pos)[0]
        self.pos += 4
        return float(v)

    def f64(self) -> float:
        self.align(8)
        v = struct.unpack_from(self.endian + "d", self.data, self.pos)[0]
        self.pos += 8
        return float(v)

    def f64_array(self, count: int) -> List[float]:
        if count <= 0:
            return []
        self.align(8)
        v = struct.unpack_from(f"{self.endian}{count}d", self.data, self.pos)
        self.pos += 8 * count
        return [float(x) for x in v]

    def f32_array(self, count: int) -> List[float]:
        if count <= 0:
            return []
        self.align(4)
        v = struct.unpack_from(f"{self.endian}{count}f", self.data, self.pos)
        self.pos += 4 * count
        return [float(x) for x in v]

    def string(self) -> str:
        length = self.u32()
        if length <= 1 or self.pos + length > self.length:
            self.pos = min(self.length, self.pos + max(0, length))
            return ""
        s = self.data[self.pos : self.pos + length - 1].decode("utf-8", errors="replace")
        self.pos += length
        return s

    def string_seq(self) -> List[str]:
        count = self.u32()
        if count > 10000:
            return []
        return [self.string() for _ in range(count)]

    def f64_seq(self) -> List[float]:
        count = self.u32()
        if count > 100000:
            return []
        return self.f64_array(count)

    def f32_seq(self) -> List[float]:
        count = self.u32()
        if count > 200000:
            return []
        return self.f32_array(count)

    def header(self) -> Dict[str, Any]:
        sec = self.i32()
        nanosec = self.u32()
        frame_id = self.string()
        return {"sec": sec, "nanosec": nanosec, "frame_id": frame_id}

    def vec3(self) -> Dict[str, float]:
        return {"x": self.f64(), "y": self.f64(), "z": self.f64()}

    def quat(self) -> Dict[str, float]:
        return {"x": self.f64(), "y": self.f64(), "z": self.f64(), "w": self.f64()}


def quat_to_euler_deg(q: Dict[str, float]) -> Tuple[float, float, float]:
    """Convert quaternion dict {x, y, z, w} to (roll, pitch, yaw) in degrees."""
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def clean_floats(obj: Any) -> Any:
    """Recursively replace NaN/Inf floats with None for valid JSON serialization."""
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return round(obj, 6)
    if isinstance(obj, dict):
        return {k: clean_floats(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean_floats(v) for v in obj]
    return obj


def decode_cdr_message(
    msg_type: str, data: bytes, full_arrays: bool = False
) -> Dict[str, Any]:
    """Decode a CDR-serialized ROS 2 message into a structured dictionary."""
    try:
        r = CDRReader(data)
        if msg_type == "std_msgs/msg/String":
            s = r.string()
            stripped = s.strip()
            if stripped.startswith("{") and stripped.endswith("}"):
                try:
                    parsed = json.loads(stripped)
                    return {"json": parsed}
                except Exception:
                    pass
            return {"data": s if full_arrays else s[:600]}

        elif msg_type == "sensor_msgs/msg/Temperature":
            hdr = r.header()
            temp = r.f64()
            var = r.f64()
            return {"header": hdr, "temperature": temp, "variance": var}

        elif msg_type == "geometry_msgs/msg/Twist":
            lin = r.vec3()
            ang = r.vec3()
            return {"linear": lin, "angular": ang}

        elif msg_type == "geometry_msgs/msg/TwistStamped":
            hdr = r.header()
            lin = r.vec3()
            ang = r.vec3()
            return {"header": hdr, "linear": lin, "angular": ang}

        elif msg_type == "nav_msgs/msg/Odometry":
            hdr = r.header()
            child = r.string()
            pos = r.vec3()
            ori = r.quat()
            _pose_cov = r.f64_array(36)
            lin = r.vec3()
            ang = r.vec3()
            _twist_cov = r.f64_array(36)
            roll, pitch, yaw = quat_to_euler_deg(ori)
            return {
                "header": hdr,
                "child_frame_id": child,
                "position": pos,
                "orientation": ori,
                "euler_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
                "linear": lin,
                "angular": ang,
            }

        elif msg_type == "sensor_msgs/msg/Imu":
            hdr = r.header()
            ori = r.quat()
            _ori_cov = r.f64_array(9)
            ang_vel = r.vec3()
            _ang_cov = r.f64_array(9)
            lin_acc = r.vec3()
            _lin_cov = r.f64_array(9)
            roll, pitch, yaw = quat_to_euler_deg(ori)
            return {
                "header": hdr,
                "orientation": ori,
                "euler_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
                "angular_velocity": ang_vel,
                "linear_acceleration": lin_acc,
            }

        elif msg_type == "sensor_msgs/msg/JointState":
            hdr = r.header()
            names = r.string_seq()
            pos = r.f64_seq()
            vel = r.f64_seq()
            eff = r.f64_seq()
            return {
                "header": hdr,
                "name": names,
                "position": pos,
                "velocity": vel,
                "effort": eff,
            }

        elif msg_type == "sensor_msgs/msg/BatteryState":
            hdr = r.header()
            voltage = r.f32()
            temperature = r.f32()
            current = r.f32()
            charge = r.f32()
            capacity = r.f32()
            design_cap = r.f32()
            percentage = r.f32()
            status = r.u8()
            health = r.u8()
            tech = r.u8()
            present = r.bool()
            cell_v = r.f32_seq()
            # Normalize percentage if in 0..1 range
            pct_display = percentage * 100.0 if 0.0 <= percentage <= 1.05 else percentage
            return {
                "header": hdr,
                "voltage": voltage,
                "current": current,
                "percentage": pct_display,
                "raw_percentage": percentage,
                "temperature": temperature,
                "charge": charge,
                "capacity": capacity,
                "design_capacity": design_cap,
                "power_supply_status": status,
                "power_supply_health": health,
                "present": present,
                "cell_voltage": cell_v[:12],
            }

        elif msg_type == "sensor_msgs/msg/MagneticField":
            hdr = r.header()
            mag = r.vec3()
            _cov = r.f64_array(9)
            return {"header": hdr, "magnetic_field": mag}

        elif msg_type == "turtlebot3_msgs/msg/SensorState":
            hdr = r.header()
            bumper = r.u8()
            cliff = r.f32()
            sonar = r.f32()
            illum = r.f32()
            led = r.u8()
            button = r.u8()
            torque = r.bool()
            l_enc = r.i32()
            r_enc = r.i32()
            batt = r.f32()
            return {
                "header": hdr,
                "bumper": bumper,
                "cliff": cliff,
                "sonar": sonar,
                "illumination": illum,
                "led": led,
                "button": button,
                "torque": torque,
                "left_encoder": l_enc,
                "right_encoder": r_enc,
                "battery": batt,
            }

        elif msg_type == "sensor_msgs/msg/LaserScan":
            hdr = r.header()
            amin = r.f32()
            amax = r.f32()
            ainc = r.f32()
            tinc = r.f32()
            stime = r.f32()
            rmin = r.f32()
            rmax = r.f32()
            ranges = r.f32_seq()
            intensities = r.f32_seq()
            valid_ranges = [
                x for x in ranges if not math.isnan(x) and not math.isinf(x) and rmin <= x <= rmax and x > 0.01
            ]
            res = {
                "header": hdr,
                "angle_min": amin,
                "angle_max": amax,
                "angle_increment": ainc,
                "range_min": rmin,
                "range_max": rmax,
                "count": len(ranges),
                "valid_count": len(valid_ranges),
                "min_dist": min(valid_ranges) if valid_ranges else None,
                "max_dist": max(valid_ranges) if valid_ranges else None,
            }
            if full_arrays:
                res["ranges"] = ranges
                res["intensities"] = intensities[: len(ranges)]
            return res

        elif msg_type == "sensor_msgs/msg/PointCloud2":
            hdr = r.header()
            height = r.u32()
            width = r.u32()
            fcount = r.u32()
            fields = []
            for _ in range(min(fcount, 32)):
                fields.append(
                    {
                        "name": r.string(),
                        "offset": r.u32(),
                        "datatype": r.u8(),
                        "count": r.u32(),
                    }
                )
            be = r.bool()
            point_step = r.u32()
            row_step = r.u32()
            dlen = r.u32()
            data_offset = r.pos
            res = {
                "header": hdr,
                "width": width,
                "height": height,
                "point_step": point_step,
                "row_step": row_step,
                "num_points": width * height,
                "fields": [f["name"] for f in fields],
            }
            if full_arrays and dlen > 0 and point_step >= 12 and data_offset + dlen <= len(data):
                floats_per_pt = point_step // 4
                num_pts = dlen // point_step
                pts = np.frombuffer(
                    data, dtype=np.float32, count=num_pts * floats_per_pt, offset=data_offset
                ).reshape(num_pts, floats_per_pt)[:, :3]
                mask = np.isfinite(pts).all(axis=1) & (np.abs(pts).sum(axis=1) > 1e-4)
                valid_pts = pts[mask]
                if len(valid_pts) > 2500:
                    step = max(1, len(valid_pts) // 2500)
                    valid_pts = valid_pts[::step]
                res["points"] = np.round(valid_pts, 3).tolist()
                res["valid_points"] = int(mask.sum())
            return res

        elif msg_type == "sensor_msgs/msg/CameraInfo":
            hdr = r.header()
            h = r.u32()
            w = r.u32()
            dm = r.string()
            D = r.f64_seq()
            K = r.f64_array(9)
            return {
                "header": hdr,
                "width": w,
                "height": h,
                "distortion_model": dm,
                "fx": K[0] if len(K) >= 9 else 0.0,
                "fy": K[4] if len(K) >= 9 else 0.0,
                "cx": K[2] if len(K) >= 9 else 0.0,
                "cy": K[5] if len(K) >= 9 else 0.0,
                "D": D[:5],
            }

        elif msg_type == "sensor_msgs/msg/CompressedImage":
            hdr = r.header()
            fmt = r.string()
            blen = r.u32()
            return {
                "header": hdr,
                "format": fmt,
                "compressed_bytes": blen,
            }

        elif msg_type == "sensor_msgs/msg/Image":
            hdr = r.header()
            h = r.u32()
            w = r.u32()
            enc = r.string()
            be = r.u8()
            step = r.u32()
            blen = r.u32()
            return {
                "header": hdr,
                "width": w,
                "height": h,
                "encoding": enc,
                "step": step,
                "raw_bytes": blen,
            }

        elif msg_type == "tf2_msgs/msg/TFMessage":
            count = r.u32()
            tfs = []
            for _ in range(min(count, 64)):
                hdr = r.header()
                child = r.string()
                tr = r.vec3()
                rot = r.quat()
                roll, pitch, yaw = quat_to_euler_deg(rot)
                tfs.append(
                    {
                        "parent": hdr["frame_id"],
                        "child": child,
                        "translation": tr,
                        "yaw_deg": yaw,
                    }
                )
            return {"transforms": tfs}

        elif msg_type == "diagnostic_msgs/msg/DiagnosticArray":
            hdr = r.header()
            scount = r.u32()
            statuses = []
            for _ in range(min(scount, 32)):
                lvl = r.u8()
                name = r.string()
                msg = r.string()
                hw = r.string()
                vcount = r.u32()
                vals = {}
                for _ in range(min(vcount, 50)):
                    k = r.string()
                    v = r.string()
                    vals[k] = v
                statuses.append(
                    {"level": lvl, "name": name, "message": msg, "hardware_id": hw, "values": vals}
                )
            return {"header": hdr, "statuses": statuses}

        elif msg_type == "rcl_interfaces/msg/Log":
            sec = r.i32()
            nanosec = r.u32()
            lvl = r.u8()
            name = r.string()
            msg = r.string()
            file = r.string()
            func = r.string()
            line = r.u32()
            return {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "level": lvl,
                "name": name,
                "msg": msg,
                "function": func,
                "line": line,
            }

        return {"raw_size_bytes": len(data), "type": msg_type}
    except Exception as exc:
        return {"decode_error": str(exc), "raw_size_bytes": len(data)}


def categorize_topic(name: str, msg_type: str) -> str:
    """Categorize a ROS topic for UI grouping and visualization."""
    if msg_type in ("sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"):
        if "depth" in name.lower():
            return "camera_depth"
        return "camera_rgb"
    if msg_type == "sensor_msgs/msg/PointCloud2":
        return "pointcloud"
    if msg_type == "sensor_msgs/msg/LaserScan":
        return "laserscan"
    if msg_type == "nav_msgs/msg/Odometry":
        return "odometry"
    if msg_type == "sensor_msgs/msg/Imu":
        return "imu"
    if msg_type in ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"):
        return "cmd_vel"
    if msg_type == "sensor_msgs/msg/JointState":
        return "joints"
    if msg_type == "sensor_msgs/msg/BatteryState":
        return "battery"
    if msg_type == "sensor_msgs/msg/Temperature":
        return "temperature"
    if msg_type == "sensor_msgs/msg/MagneticField":
        return "magnetic"
    if msg_type == "turtlebot3_msgs/msg/SensorState":
        return "sensor_state"
    if msg_type == "tf2_msgs/msg/TFMessage":
        return "tf"
    if msg_type == "sensor_msgs/msg/CameraInfo":
        return "camera_info"
    if msg_type in ("diagnostic_msgs/msg/DiagnosticArray", "rcl_interfaces/msg/Log"):
        return "diagnostics"
    if msg_type == "std_msgs/msg/String":
        if any(k in name for k in ("state", "motor", "diagnostics", "metadata")):
            return "state_json"
        return "string"
    return "other"


def format_camera_label(topic_name: str, msg_type: str) -> str:
    """Create a human-friendly camera label from a ROS camera topic name."""
    parts = [p for p in topic_name.strip("/").split("/") if p not in ("ma_raihan", "cameras", "camera")]
    is_depth = "depth" in topic_name.lower()
    is_raw = msg_type == "sensor_msgs/msg/Image"
    if not parts:
        return topic_name
    cam_pos = parts[0].replace("_", " ").title()
    if is_depth:
        return f"{cam_pos} (Depth)"
    if is_raw:
        return f"{cam_pos} (Raw RGB)"
    return f"{cam_pos} (RGB)"


COLORMAP_LOOKUP = {
    "turbo": cv2.COLORMAP_TURBO,
    "inferno": cv2.COLORMAP_INFERNO,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "bone": cv2.COLORMAP_BONE,
    "jet": cv2.COLORMAP_JET,
}


class BagSession:
    """Manages an opened ROS 2 bag (.db3 file or bag folder) with fast indexed queries."""

    def __init__(self, path: str):
        self.source_path = os.path.abspath(path)
        if os.path.isdir(self.source_path):
            self.bag_dir = self.source_path
            self.bag_name = os.path.basename(self.source_path.rstrip("/"))
            db_files = sorted(Path(self.bag_dir).glob("*.db3"))
            if not db_files:
                raise ValueError(f"No .db3 files found inside directory: {self.source_path}")
            self.db_path = str(db_files[0])
        elif os.path.isfile(self.source_path) and self.source_path.endswith(".db3"):
            self.db_path = self.source_path
            self.bag_dir = os.path.dirname(self.source_path)
            self.bag_name = os.path.basename(self.bag_dir) or os.path.basename(self.source_path)
            if self.bag_name in ("sample_bags", "uploaded_bags", "Bags", "go1_bags"):
                self.bag_name = os.path.splitext(os.path.basename(self.source_path))[0]
        else:
            raise ValueError(f"Path must be a ROS 2 bag folder or .db3 file: {self.source_path}")

        self._lock = threading.Lock()
        self._frame_cache: OrderedDict[Tuple[int, str], Tuple[bytes, str]] = OrderedDict()
        self._frame_cache_max = 256
        self._decoded_cache: OrderedDict[int, Dict[str, Any]] = OrderedDict()
        self._decoded_cache_max = 1024

        self.topics: Dict[int, Dict[str, Any]] = {}
        self.topic_by_name: Dict[str, Dict[str, Any]] = {}
        # topic_id -> (rel_times_list, msg_ids_list)
        self.topic_index: Dict[int, Tuple[List[float], List[int]]] = {}

        self.start_ns: int = 0
        self.end_ns: int = 0
        self.duration_s: float = 0.0
        self.total_messages: int = 0
        self.robot_profile: str = "Generic ROS 2 Robot"
        self.extra_reports: Dict[str, Any] = {}
        self.timeseries: Dict[str, Any] = {}

        self._load_and_index()

    def _get_conn(self) -> sqlite3.Connection:
        conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True, check_same_thread=False)
        return conn

    def _load_and_index(self) -> None:
        conn = self._get_conn()
        cur = conn.cursor()

        # 1. Load topics
        cur.execute("SELECT id, name, type, serialization_format FROM topics")
        for tid, name, msg_type, ser_fmt in cur.fetchall():
            cat = categorize_topic(name, msg_type)
            info = {
                "id": tid,
                "name": name,
                "type": msg_type,
                "serialization_format": ser_fmt,
                "category": cat,
                "message_count": 0,
                "hz": 0.0,
                "first_t": None,
                "last_t": None,
            }
            if cat in ("camera_rgb", "camera_depth"):
                info["label"] = format_camera_label(name, msg_type)
                info["is_depth"] = cat == "camera_depth"
            self.topics[tid] = info
            self.topic_by_name[name] = info

        # Detect robot type
        all_names = " ".join(self.topic_by_name.keys()).lower()
        if "go1" in all_names or "rear_down" in all_names or "calf_joint" in all_names:
            self.robot_profile = "Unitree Go1 Quadruped"
        elif "turtlebot" in all_names or "/scan" in self.topic_by_name or "/sensor_state" in self.topic_by_name:
            self.robot_profile = "TurtleBot3 Mobile Robot"

        # 2. Find global start/end timestamps
        cur.execute("SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages")
        row = cur.fetchone()
        if not row or row[0] is None:
            conn.close()
            return
        self.start_ns, self.end_ns, self.total_messages = int(row[0]), int(row[1]), int(row[2])
        self.duration_s = max(0.001, (self.end_ns - self.start_ns) / 1e9)

        # 3. Build timestamp -> msg_id index per topic
        cur.execute("SELECT id, topic_id, timestamp FROM messages ORDER BY timestamp ASC")
        temp_index: Dict[int, Tuple[List[float], List[int]]] = {
            tid: ([], []) for tid in self.topics
        }
        for msg_id, tid, ts in cur.fetchall():
            if tid not in temp_index:
                continue
            rel_t = (ts - self.start_ns) / 1e9
            t_list, id_list = temp_index[tid]
            t_list.append(round(rel_t, 5))
            id_list.append(msg_id)

        for tid, (t_list, id_list) in temp_index.items():
            self.topic_index[tid] = (t_list, id_list)
            tinfo = self.topics[tid]
            cnt = len(t_list)
            tinfo["message_count"] = cnt
            if cnt > 0:
                tinfo["first_t"] = t_list[0]
                tinfo["last_t"] = t_list[-1]
                span = t_list[-1] - t_list[0]
                tinfo["hz"] = round((cnt - 1) / span, 2) if cnt > 1 and span > 1e-3 else 0.0

        # 4. Load optional companion JSON files (capture_report.json, experiment.json, etc.)
        for fname in ("capture_report.json", "experiment.json", "audit.json", "clock_reference.json"):
            fpath = os.path.join(self.bag_dir, fname)
            if os.path.isfile(fpath):
                try:
                    with open(fpath, "r", encoding="utf-8") as f:
                        self.extra_reports[fname] = json.load(f)
                except Exception:
                    pass

        # 5. Pre-extract time-series telemetry for fast synchronized charting
        self._extract_timeseries(conn)
        conn.close()

    def _extract_timeseries(self, conn: sqlite3.Connection) -> None:
        cur = conn.cursor()
        odom_series: Dict[str, Any] = {}
        imu_series: Dict[str, Any] = {}
        cmd_vel_series: Dict[str, Any] = {}
        joints_series: Dict[str, Any] = {}
        battery_series: Dict[str, Any] = {}
        temp_series: Dict[str, Any] = {}
        mag_series: Dict[str, Any] = {}
        sensor_state_series: Dict[str, Any] = {}
        high_state_series: Dict[str, Any] = {}

        # Select all non-image, non-pointcloud, non-laserscan messages for telemetry series
        telemetry_cats = {
            "odometry",
            "imu",
            "cmd_vel",
            "joints",
            "battery",
            "temperature",
            "magnetic",
            "sensor_state",
            "state_json",
        }
        telemetry_tids = [
            tid for tid, info in self.topics.items() if info["category"] in telemetry_cats and info["message_count"] > 0
        ]
        if not telemetry_tids:
            self.timeseries = {}
            return

        placeholders = ",".join("?" for _ in telemetry_tids)
        cur.execute(
            f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp ASC",
            telemetry_tids,
        )

        for tid, ts, data in cur.fetchall():
            tinfo = self.topics[tid]
            tname = tinfo["name"]
            ttype = tinfo["type"]
            cat = tinfo["category"]
            rel_t = round((ts - self.start_ns) / 1e9, 4)

            decoded = decode_cdr_message(ttype, data, full_arrays=False)
            if "decode_error" in decoded:
                continue

            if cat == "odometry":
                s = odom_series.setdefault(
                    tname,
                    {"t": [], "x": [], "y": [], "z": [], "yaw": [], "vx": [], "vy": [], "wz": []},
                )
                s["t"].append(rel_t)
                s["x"].append(round(decoded["position"]["x"], 4))
                s["y"].append(round(decoded["position"]["y"], 4))
                s["z"].append(round(decoded["position"]["z"], 4))
                s["yaw"].append(round(decoded["euler_deg"]["yaw"], 2))
                s["vx"].append(round(decoded["linear"]["x"], 4))
                s["vy"].append(round(decoded["linear"]["y"], 4))
                s["wz"].append(round(decoded["angular"]["z"], 4))

            elif cat == "imu":
                s = imu_series.setdefault(
                    tname,
                    {
                        "t": [],
                        "roll": [],
                        "pitch": [],
                        "yaw": [],
                        "ax": [],
                        "ay": [],
                        "az": [],
                        "gx": [],
                        "gy": [],
                        "gz": [],
                    },
                )
                s["t"].append(rel_t)
                s["roll"].append(round(decoded["euler_deg"]["roll"], 2))
                s["pitch"].append(round(decoded["euler_deg"]["pitch"], 2))
                s["yaw"].append(round(decoded["euler_deg"]["yaw"], 2))
                s["ax"].append(round(decoded["linear_acceleration"]["x"], 3))
                s["ay"].append(round(decoded["linear_acceleration"]["y"], 3))
                s["az"].append(round(decoded["linear_acceleration"]["z"], 3))
                s["gx"].append(round(decoded["angular_velocity"]["x"], 3))
                s["gy"].append(round(decoded["angular_velocity"]["y"], 3))
                s["gz"].append(round(decoded["angular_velocity"]["z"], 3))

            elif cat == "cmd_vel":
                s = cmd_vel_series.setdefault(tname, {"t": [], "lx": [], "ly": [], "az": []})
                s["t"].append(rel_t)
                s["lx"].append(round(decoded["linear"]["x"], 4))
                s["ly"].append(round(decoded["linear"]["y"], 4))
                s["az"].append(round(decoded["angular"]["z"], 4))

            elif cat == "joints":
                names = decoded.get("name", [])
                s = joints_series.setdefault(
                    tname,
                    {"t": [], "names": names, "position": {}, "velocity": {}, "effort": {}},
                )
                if not s["names"] and names:
                    s["names"] = names
                s["t"].append(rel_t)
                pos = decoded.get("position", [])
                vel = decoded.get("velocity", [])
                eff = decoded.get("effort", [])
                for idx, jname in enumerate(names):
                    if idx < len(pos):
                        s["position"].setdefault(jname, []).append(round(pos[idx], 4))
                    if idx < len(vel):
                        s["velocity"].setdefault(jname, []).append(round(vel[idx], 4))
                    if idx < len(eff):
                        s["effort"].setdefault(jname, []).append(round(eff[idx], 4))

            elif cat == "battery":
                s = battery_series.setdefault(
                    tname, {"t": [], "voltage": [], "current": [], "percentage": []}
                )
                s["t"].append(rel_t)
                s["voltage"].append(round(decoded.get("voltage", 0.0), 3))
                s["current"].append(round(decoded.get("current", 0.0), 3))
                s["percentage"].append(round(decoded.get("percentage", 0.0), 2))

            elif cat == "temperature":
                short_label = tname.strip("/").replace("ma_raihan/", "").replace("temperature/", "")
                s = temp_series.setdefault(tname, {"label": short_label, "t": [], "temp": []})
                s["t"].append(rel_t)
                s["temp"].append(round(decoded.get("temperature", 0.0), 2))

            elif cat == "magnetic":
                s = mag_series.setdefault(tname, {"t": [], "x": [], "y": [], "z": []})
                s["t"].append(rel_t)
                mf = decoded.get("magnetic_field", {"x": 0.0, "y": 0.0, "z": 0.0})
                s["x"].append(round(mf["x"], 5))
                s["y"].append(round(mf["y"], 5))
                s["z"].append(round(mf["z"], 5))

            elif cat == "sensor_state":
                s = sensor_state_series.setdefault(
                    tname,
                    {"t": [], "left_encoder": [], "right_encoder": [], "battery": [], "sonar": []},
                )
                s["t"].append(rel_t)
                s["left_encoder"].append(decoded.get("left_encoder", 0))
                s["right_encoder"].append(decoded.get("right_encoder", 0))
                s["battery"].append(round(decoded.get("battery", 0.0), 3))
                s["sonar"].append(round(decoded.get("sonar", 0.0), 3))

            elif cat == "state_json" and "high_state" in tname and "raw" not in tname:
                j = decoded.get("json", {})
                st = j.get("message", {}).get("state", {})
                ff = st.get("footForceEst", [])
                if len(ff) == 4:
                    s = high_state_series.setdefault(
                        tname,
                        {"t": [], "foot_FR": [], "foot_FL": [], "foot_RR": [], "foot_RL": [], "bodyHeight": []},
                    )
                    s["t"].append(rel_t)
                    s["foot_FR"].append(ff[0])
                    s["foot_FL"].append(ff[1])
                    s["foot_RR"].append(ff[2])
                    s["foot_RL"].append(ff[3])
                    s["bodyHeight"].append(round(float(st.get("bodyHeight", 0.0)), 4))

        self.timeseries = clean_floats(
            {
                "odometry": odom_series,
                "imu": imu_series,
                "cmd_vel": cmd_vel_series,
                "joints": joints_series,
                "battery": battery_series,
                "temperature": temp_series,
                "magnetic": mag_series,
                "sensor_state": sensor_state_series,
                "high_state": high_state_series,
            }
        )

    def get_summary(self) -> Dict[str, Any]:
        """Return bag metadata, topic list, camera stream index, and telemetry time series."""
        cameras = []
        spatial_topics = []
        topic_list = []

        for tid, info in sorted(self.topics.items(), key=lambda x: x[1]["name"]):
            t_list, id_list = self.topic_index.get(tid, ([], []))
            entry = dict(info)
            topic_list.append(entry)

            if info["category"] in ("camera_rgb", "camera_depth") and len(id_list) > 0:
                cameras.append(
                    {
                        "id": tid,
                        "name": info["name"],
                        "label": info["label"],
                        "type": info["type"],
                        "is_depth": info["is_depth"],
                        "message_count": info["message_count"],
                        "hz": info["hz"],
                        "frames": {"t": t_list, "id": id_list},
                    }
                )
            elif info["category"] in ("pointcloud", "laserscan") and len(id_list) > 0:
                spatial_topics.append(
                    {
                        "id": tid,
                        "name": info["name"],
                        "type": info["type"],
                        "category": info["category"],
                        "message_count": info["message_count"],
                        "hz": info["hz"],
                    }
                )

        # Extract timeline events from experiment.json if present
        events = []
        exp = self.extra_reports.get("experiment.json", {})
        if isinstance(exp, dict) and "events" in exp:
            for ev in exp.get("events", []):
                tns = ev.get("time_ns")
                if tns:
                    rel_t = round((tns - self.start_ns) / 1e9, 3)
                    if 0 <= rel_t <= self.duration_s:
                        events.append(
                            {
                                "t": rel_t,
                                "event": ev.get("event", "event"),
                                "details": {
                                    k: v for k, v in ev.items() if k not in ("time_ns", "event", "bag")
                                },
                            }
                        )

        return clean_floats(
            {
                "bag_name": self.bag_name,
                "source_path": self.source_path,
                "db_path": self.db_path,
                "robot_profile": self.robot_profile,
                "start_ns": self.start_ns,
                "end_ns": self.end_ns,
                "duration_s": round(self.duration_s, 3),
                "total_messages": self.total_messages,
                "topic_count": len(topic_list),
                "topics": topic_list,
                "cameras": cameras,
                "spatial_topics": spatial_topics,
                "events": events,
                "timeseries": self.timeseries,
            }
        )

    def find_msg_at_time(self, topic_id: int, rel_t: float) -> Optional[Tuple[int, float, int]]:
        """Binary search for the latest message at or before rel_t (in seconds)."""
        if topic_id not in self.topic_index:
            return None
        t_list, id_list = self.topic_index[topic_id]
        if not t_list:
            return None
        idx = bisect.bisect_right(t_list, rel_t) - 1
        if idx < 0:
            idx = 0
        return id_list[idx], t_list[idx], idx

    def get_image_by_msg_id(self, msg_id: int, colormap: str = "turbo") -> Optional[Tuple[bytes, str]]:
        """Fetch and decode/render an image message by SQLite message ID."""
        cache_key = (msg_id, colormap)
        with self._lock:
            if cache_key in self._frame_cache:
                self._frame_cache.move_to_end(cache_key)
                return self._frame_cache[cache_key]

        conn = self._get_conn()
        cur = conn.cursor()
        cur.execute("SELECT topic_id, data FROM messages WHERE id = ?", (msg_id,))
        row = cur.fetchone()
        conn.close()
        if not row:
            return None

        tid, data = row
        tinfo = self.topics.get(tid)
        if not tinfo:
            return None
        msg_type = tinfo["type"]

        result: Optional[Tuple[bytes, str]] = None
        r = CDRReader(data)

        if msg_type == "sensor_msgs/msg/CompressedImage":
            _hdr = r.header()
            fmt = r.string().lower()
            blen = r.u32()
            raw_img_bytes = data[r.pos : r.pos + blen]

            if "jpeg" in fmt or "jpg" in fmt or raw_img_bytes[:2] == b"\xff\xd8":
                result = (bytes(raw_img_bytes), "image/jpeg")
            elif "png" in fmt or raw_img_bytes[:4] == b"\x89PNG":
                # Check if 16-bit depth PNG
                if "16uc1" in fmt or tinfo.get("is_depth"):
                    arr = np.frombuffer(raw_img_bytes, dtype=np.uint8)
                    depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
                    if depth is not None and depth.dtype == np.uint16:
                        valid = depth > 0
                        if valid.any():
                            vmin = float(np.percentile(depth[valid], 2))
                            vmax = float(np.percentile(depth[valid], 98))
                            if vmax <= vmin:
                                vmax = vmin + 1.0
                            norm = np.clip((depth.astype(np.float32) - vmin) / (vmax - vmin), 0.0, 1.0)
                            u8_img = (norm * 255.0).astype(np.uint8)
                        else:
                            u8_img = np.zeros_like(depth, dtype=np.uint8)

                        if colormap == "raw":
                            colored = cv2.cvtColor(u8_img, cv2.COLOR_GRAY2BGR)
                        else:
                            cmap_id = COLORMAP_LOOKUP.get(colormap, cv2.COLORMAP_TURBO)
                            colored = cv2.applyColorMap(u8_img, cmap_id)
                            colored[~valid] = (12, 15, 22)

                        ok, enc_buf = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 90])
                        if ok:
                            result = (enc_buf.tobytes(), "image/jpeg")
                if result is None:
                    result = (bytes(raw_img_bytes), "image/png")

        elif msg_type == "sensor_msgs/msg/Image":
            _hdr = r.header()
            h = r.u32()
            w = r.u32()
            enc = r.string().lower()
            _be = r.u8()
            step = r.u32()
            blen = r.u32()
            raw_pixels = data[r.pos : r.pos + blen]
            if w > 0 and h > 0 and len(raw_pixels) >= h * step:
                if enc in ("bgr8", "rgb8"):
                    arr = np.frombuffer(raw_pixels, dtype=np.uint8).reshape(h, step)[:, : w * 3].reshape(h, w, 3)
                    if enc == "rgb8":
                        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    ok, enc_buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 88])
                    if ok:
                        result = (enc_buf.tobytes(), "image/jpeg")
                elif enc == "mono8":
                    arr = np.frombuffer(raw_pixels, dtype=np.uint8).reshape(h, step)[:, :w]
                    ok, enc_buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 88])
                    if ok:
                        result = (enc_buf.tobytes(), "image/jpeg")
                elif enc in ("16uc1", "mono16"):
                    arr = np.frombuffer(raw_pixels, dtype=np.uint16).reshape(h, step // 2)[:, :w]
                    valid = arr > 0
                    u8_img = np.zeros_like(arr, dtype=np.uint8)
                    if valid.any():
                        vmax = max(1.0, float(np.percentile(arr[valid], 98)))
                        u8_img = np.clip(arr.astype(np.float32) / vmax * 255.0, 0, 255).astype(np.uint8)
                    colored = cv2.applyColorMap(u8_img, COLORMAP_LOOKUP.get(colormap, cv2.COLORMAP_TURBO))
                    ok, enc_buf = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 88])
                    if ok:
                        result = (enc_buf.tobytes(), "image/jpeg")

        if result is not None:
            with self._lock:
                self._frame_cache[cache_key] = result
                if len(self._frame_cache) > self._frame_cache_max:
                    self._frame_cache.popitem(last=False)

        return result

    def get_sync_snapshot(self, rel_t: float) -> Dict[str, Any]:
        """
        Return the synchronized state across all topics at playback time `rel_t` (seconds).
        Includes spatial sensors (LaserScan, PointCloud2) and all decoded topic states.
        """
        rel_t = max(0.0, min(self.duration_s, rel_t))
        topics_snapshot: Dict[str, Any] = {}
        pointclouds: Dict[str, Any] = {}
        laserscans: Dict[str, Any] = {}

        missing_msg_ids: List[int] = []
        msg_id_meta: Dict[int, Tuple[int, float, int]] = {}

        for tid, tinfo in self.topics.items():
            found = self.find_msg_at_time(tid, rel_t)
            if found is None:
                continue
            msg_id, msg_t, msg_idx = found
            cat = tinfo["category"]
            tname = tinfo["name"]
            ttype = tinfo["type"]

            if cat in ("camera_rgb", "camera_depth"):
                topics_snapshot[tname] = {
                    "msg_id": msg_id,
                    "seq_idx": msg_idx,
                    "total_msgs": tinfo["message_count"],
                    "t": msg_t,
                    "delta_ms": round((rel_t - msg_t) * 1000.0, 1),
                    "type": ttype,
                    "category": cat,
                    "data": {"frame_index": msg_idx, "msg_id": msg_id, "label": tinfo.get("label", tname)},
                }
                continue

            msg_id_meta[msg_id] = (tid, msg_t, msg_idx)
            with self._lock:
                if msg_id not in self._decoded_cache:
                    missing_msg_ids.append(msg_id)

        if missing_msg_ids:
            placeholders = ",".join("?" for _ in missing_msg_ids)
            conn = self._get_conn()
            cur = conn.cursor()
            cur.execute(
                f"SELECT id, topic_id, data FROM messages WHERE id IN ({placeholders})",
                missing_msg_ids,
            )
            rows = cur.fetchall()
            conn.close()

            for msg_id, tid, data in rows:
                tinfo = self.topics[tid]
                ttype = tinfo["type"]
                cat = tinfo["category"]
                need_full = cat in ("pointcloud", "laserscan")
                decoded = clean_floats(decode_cdr_message(ttype, data, full_arrays=need_full))
                with self._lock:
                    self._decoded_cache[msg_id] = decoded
                    if len(self._decoded_cache) > self._decoded_cache_max:
                        self._decoded_cache.popitem(last=False)

        for msg_id, (tid, msg_t, msg_idx) in msg_id_meta.items():
            tinfo = self.topics[tid]
            tname = tinfo["name"]
            ttype = tinfo["type"]
            cat = tinfo["category"]
            with self._lock:
                decoded = self._decoded_cache.get(msg_id, {})

            if cat == "pointcloud" and "points" in decoded:
                pts = decoded["points"]
                pointclouds[tname] = {
                    "t": msg_t,
                    "delta_ms": round((rel_t - msg_t) * 1000.0, 1),
                    "frame_id": decoded.get("header", {}).get("frame_id", ""),
                    "valid_points": decoded.get("valid_points", len(pts)),
                    "points": pts,
                }
                summary_data = {k: v for k, v in decoded.items() if k != "points"}
            elif cat == "laserscan" and "ranges" in decoded:
                ranges = decoded["ranges"]
                intensities = decoded.get("intensities", [])
                laserscans[tname] = {
                    "t": msg_t,
                    "delta_ms": round((rel_t - msg_t) * 1000.0, 1),
                    "angle_min": decoded["angle_min"],
                    "angle_max": decoded["angle_max"],
                    "angle_increment": decoded["angle_increment"],
                    "range_min": decoded["range_min"],
                    "range_max": decoded["range_max"],
                    "ranges": ranges,
                    "intensities": intensities,
                }
                summary_data = {k: v for k, v in decoded.items() if k not in ("ranges", "intensities")}
            else:
                summary_data = decoded

            topics_snapshot[tname] = {
                "msg_id": msg_id,
                "seq_idx": msg_idx,
                "total_msgs": tinfo["message_count"],
                "t": msg_t,
                "delta_ms": round((rel_t - msg_t) * 1000.0, 1),
                "type": ttype,
                "category": cat,
                "data": summary_data,
            }

        return {
            "t": round(rel_t, 4),
            "pointclouds": pointclouds,
            "laserscans": laserscans,
            "topics": topics_snapshot,
        }
