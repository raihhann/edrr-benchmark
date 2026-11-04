#!/usr/bin/env python3
"""
ROS 2 Multi-Robot Synchronized Bag Visualization Server.
Provides REST & streaming endpoints for loading/uploading .db3 bag files or folders,
serving synchronized multi-camera video frames, spatial sensor snapshots (PointCloud2,
LaserScan, Odometry), telemetry time-series, and live all-topic inspection.
"""

import argparse
import json
import mimetypes
import os
import re
import shutil
import sys
import threading
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

from bag_engine import BagSession

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
SAMPLE_BAGS_DIR = os.path.join(BASE_DIR, "sample_bags")
UPLOADED_BAGS_DIR = os.path.join(BASE_DIR, "uploaded_bags")

os.makedirs(SAMPLE_BAGS_DIR, exist_ok=True)
os.makedirs(UPLOADED_BAGS_DIR, exist_ok=True)

SEARCH_ROOTS = [
    SAMPLE_BAGS_DIR,
    UPLOADED_BAGS_DIR,
    "/home/fsti/ROS/Go1_Project/Bags/go1_bags",
    "/home/fsti/ROS/TurtleBot/Bags",
]


class AppState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_bag: Optional[BagSession] = None
        self.current_summary: Optional[Dict[str, Any]] = None

    def load_bag(self, path: str) -> Dict[str, Any]:
        session = BagSession(path)
        summary = session.get_summary()
        with self.lock:
            self.current_bag = session
            self.current_summary = summary
        return summary


STATE = AppState()


def discover_bags() -> List[Dict[str, Any]]:
    """Discover ROS 2 bag directories and .db3 files across known roots."""
    discovered: List[Dict[str, Any]] = []
    seen_realpaths = set()

    for root in SEARCH_ROOTS:
        if not os.path.isdir(root):
            continue
        try:
            entries = sorted(os.listdir(root))
        except OSError:
            continue

        for entry in entries:
            full_path = os.path.join(root, entry)
            if os.path.isdir(full_path):
                try:
                    db3s = [f for f in os.listdir(full_path) if f.endswith(".db3")]
                except OSError:
                    continue
                if not db3s:
                    continue
                db_file = os.path.join(full_path, sorted(db3s)[0])
                try:
                    st = os.stat(db_file)
                    real_key = (st.st_dev, st.st_ino)
                except OSError:
                    real_key = os.path.realpath(db_file)
                if real_key in seen_realpaths:
                    continue
                seen_realpaths.add(real_key)

                size_mb = round(os.path.getsize(db_file) / (1024 * 1024), 1)
                robot_hint = "Unitree Go1" if "go1" in entry.lower() else (
                    "TurtleBot3" if "walk" in entry.lower() or "turtle" in root.lower() else "ROS 2 Bag"
                )
                discovered.append(
                    {
                        "name": entry,
                        "path": full_path,
                        "db_file": db3s[0],
                        "size_mb": size_mb,
                        "robot_hint": robot_hint,
                        "source": "Uploaded" if root == UPLOADED_BAGS_DIR else "Discovered",
                    }
                )
            elif entry.endswith(".db3") and os.path.isfile(full_path):
                try:
                    st = os.stat(full_path)
                    real_key = (st.st_dev, st.st_ino)
                except OSError:
                    real_key = os.path.realpath(full_path)
                if real_key in seen_realpaths:
                    continue
                seen_realpaths.add(real_key)
                size_mb = round(os.path.getsize(full_path) / (1024 * 1024), 1)
                discovered.append(
                    {
                        "name": os.path.splitext(entry)[0],
                        "path": full_path,
                        "db_file": entry,
                        "size_mb": size_mb,
                        "robot_hint": "ROS 2 Bag (.db3)",
                        "source": "Uploaded" if root == UPLOADED_BAGS_DIR else "Discovered",
                    }
                )

    return discovered


def sanitize_name(name: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9._-]+", "_", name.strip())
    return cleaned.strip("._") or "uploaded_bag"


class BagRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep console output clean; only log errors or load actions
        pass

    def _send_json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_error_json(self, message: str, status: int = 400) -> None:
        self._send_json({"error": message}, status=status)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        route = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if route == "/api/bags":
            bags = discover_bags()
            current_path = STATE.current_bag.source_path if STATE.current_bag else None
            self._send_json({"bags": bags, "current_path": current_path})
            return

        if route == "/api/state":
            with STATE.lock:
                summary = STATE.current_summary
            if summary is None:
                self._send_json({"loaded": False, "bags": discover_bags()})
            else:
                self._send_json({"loaded": True, "bag": summary, "bags": discover_bags()})
            return

        if route == "/api/sync":
            with STATE.lock:
                bag = STATE.current_bag
            if bag is None:
                self._send_error_json("No bag currently loaded", 404)
                return
            try:
                t_val = float(query.get("t", ["0.0"])[0])
            except ValueError:
                t_val = 0.0
            snapshot = bag.get_sync_snapshot(t_val)
            self._send_json(snapshot)
            return

        if route == "/api/frame":
            with STATE.lock:
                bag = STATE.current_bag
            if bag is None:
                self._send_error_json("No bag currently loaded", 404)
                return
            colormap = query.get("colormap", ["turbo"])[0]
            msg_id_str = query.get("id", [None])[0]
            if msg_id_str is not None:
                try:
                    msg_id = int(msg_id_str)
                except ValueError:
                    self._send_error_json("Invalid frame id", 400)
                    return
            else:
                topic_id_str = query.get("topic_id", [None])[0]
                t_str = query.get("t", ["0.0"])[0]
                if topic_id_str is None:
                    self._send_error_json("Missing id or topic_id", 400)
                    return
                found = bag.find_msg_at_time(int(topic_id_str), float(t_str))
                if not found:
                    self._send_error_json("No frame found", 404)
                    return
                msg_id = found[0]

            res = bag.get_image_by_msg_id(msg_id, colormap=colormap)
            if res is None:
                self._send_error_json("Could not decode image frame", 404)
                return
            img_bytes, content_type = res
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(img_bytes)))
            self.send_header("Cache-Control", "public, max-age=3600")
            self.end_headers()
            self.wfile.write(img_bytes)
            return

        # Static file serving
        if route == "/" or route == "":
            route = "/index.html"
        safe_rel = os.path.normpath(route.lstrip("/"))
        if safe_rel.startswith(".."):
            self._send_error_json("Forbidden", 403)
            return
        file_path = os.path.join(STATIC_DIR, safe_rel)
        if not os.path.isfile(file_path):
            self._send_error_json("Not found", 404)
            return

        ctype, _ = mimetypes.guess_type(file_path)
        if not ctype:
            ctype = "application/octet-stream"
        with open(file_path, "rb") as f:
            content = f.read()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        route = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if route == "/api/load":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8"))
                target_path = body.get("path", "").strip()
                if not target_path:
                    self._send_error_json("Missing bag path", 400)
                    return
                summary = STATE.load_bag(target_path)
                self._send_json({"loaded": True, "bag": summary, "bags": discover_bags()})
            except Exception as exc:
                self._send_error_json(f"Failed to load bag: {exc}", 400)
            return

        if route == "/api/upload_file":
            # Stream binary upload directly to disk inside uploaded_bags/<bag_name>/<file_name>
            bag_name = sanitize_name(query.get("bag_name", ["uploaded_bag"])[0])
            file_name = os.path.basename(query.get("file_name", ["bag_0.db3"])[0])
            file_name = sanitize_name(file_name)
            activate = query.get("activate", ["1"])[0] == "1"

            target_dir = os.path.join(UPLOADED_BAGS_DIR, bag_name)
            os.makedirs(target_dir, exist_ok=True)
            dest_path = os.path.join(target_dir, file_name)

            length = int(self.headers.get("Content-Length", "0"))
            remaining = length
            chunk_size = 1024 * 1024  # 1MB chunks
            try:
                with open(dest_path, "wb") as out_f:
                    while remaining > 0:
                        chunk = self.rfile.read(min(chunk_size, remaining))
                        if not chunk:
                            break
                        out_f.write(chunk)
                        remaining -= len(chunk)

                if activate:
                    summary = STATE.load_bag(target_dir)
                    self._send_json(
                        {
                            "uploaded": True,
                            "loaded": True,
                            "path": target_dir,
                            "bag": summary,
                            "bags": discover_bags(),
                        }
                    )
                else:
                    self._send_json({"uploaded": True, "loaded": False, "path": target_dir})
            except Exception as exc:
                self._send_error_json(f"Upload failed: {exc}", 500)
            return

        self._send_error_json("Unknown endpoint", 404)


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS 2 Multi-Robot Bag Visualization Server")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface to bind")
    parser.add_argument("--port", type=int, default=8765, help="Port number")
    parser.add_argument("--bag", default="", help="Initial bag directory or .db3 file to load")
    args = parser.parse_args()

    # Auto-load initial bag if available
    initial_bag = args.bag
    if not initial_bag:
        discovered = discover_bags()
        if discovered:
            initial_bag = discovered[0]["path"]

    if initial_bag and os.path.exists(initial_bag):
        try:
            summary = STATE.load_bag(initial_bag)
            print(
                f"[BagStudio] Loaded initial bag: {summary['bag_name']} "
                f"({summary['robot_profile']}, {summary['duration_s']}s, {len(summary['cameras'])} cameras)"
            )
        except Exception as exc:
            print(f"[BagStudio] Warning: Could not load initial bag {initial_bag}: {exc}")

    server = ThreadingHTTPServer((args.host, args.port), BagRequestHandler)
    print(f"[BagStudio] Server running at http://{args.host}:{args.port}")
    sys.stdout.flush()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[BagStudio] Shutting down...")
        server.server_close()


if __name__ == "__main__":
    main()
