#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
PORT="${1:-8765}"
echo "Starting ROS 2 Multi-Robot Bag Visualization Studio on http://127.0.0.1:${PORT} ..."
exec python3 server.py --host 127.0.0.1 --port "${PORT}"
