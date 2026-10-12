#!/usr/bin/env python3
"""Inspect ROS 2 SQLite bags for topic coverage, timing, gaps, and image streams.

Usage:
    python3 Verification/verify_bag_insights.py Bags/go1_bags/<bag_folder>
"""
import argparse
import sqlite3
import sys
from pathlib import Path


EXPECTED_TOPICS = {
    "/go1_controller/odom",
    "/go1_controller/imu/data",
    "/go1_controller/state",
    "/go1_controller/low_state",
    "/motor_states",
    "/joint_states",
    "/battery_state",
}
IMAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage",
}


def matches_topic(actual, expected):
    return actual == expected or actual.endswith(expected)


def inspect_bag(bag_path):
    if bag_path.is_file() and bag_path.suffix == ".db3":
        databases = [bag_path]
    elif bag_path.is_dir():
        databases = sorted(bag_path.glob("*.db3"))
    else:
        databases = []
    if not databases:
        raise FileNotFoundError(f"No ROS 2 SQLite .db3 bag found at {bag_path}")

    topics = {}
    bag_start = None
    bag_end = None
    for database in databases:
        with sqlite3.connect(database) as connection:
            columns = {
                row[1] for row in connection.execute("PRAGMA table_info(messages)")
            }
            if not {"id", "topic_id", "timestamp"}.issubset(columns):
                raise RuntimeError(f"Unexpected rosbag messages schema in {database}")

            local_topics = {
                row[0]: {"name": row[1], "type": row[2]}
                for row in connection.execute("SELECT id, name, type FROM topics")
            }
            local_stats = {
                topic["name"]: {
                    "type": topic["type"],
                    "count": 0,
                    "first": None,
                    "last": None,
                    "min": None,
                    "max": None,
                    "max_gap": 0,
                    "gap_sum": 0,
                    "gap_count": 0,
                    "out_of_order": 0,
                }
                for topic in local_topics.values()
            }
            for topic_id, timestamp in connection.execute(
                "SELECT topic_id, timestamp FROM messages ORDER BY id"
            ):
                if topic_id not in local_topics:
                    continue
                topic = local_topics[topic_id]
                name = topic["name"]
                stat = local_stats.setdefault(
                    name,
                    {
                        "type": topic["type"],
                        "count": 0,
                        "first": None,
                        "last": None,
                        "min": None,
                        "max": None,
                        "max_gap": 0,
                        "gap_sum": 0,
                        "gap_count": 0,
                        "out_of_order": 0,
                    },
                )
                if stat["type"] != topic["type"]:
                    raise RuntimeError(f"Conflicting message types for topic {name}")
                if stat["count"]:
                    gap = timestamp - stat["last"]
                    if gap < 0:
                        stat["out_of_order"] += 1
                    else:
                        stat["max_gap"] = max(stat["max_gap"], gap)
                        stat["gap_sum"] += gap
                        stat["gap_count"] += 1
                stat["count"] += 1
                stat["first"] = timestamp if stat["first"] is None else stat["first"]
                stat["last"] = timestamp
                stat["min"] = timestamp if stat["min"] is None else min(stat["min"], timestamp)
                stat["max"] = timestamp if stat["max"] is None else max(stat["max"], timestamp)
                bag_start = timestamp if bag_start is None else min(bag_start, timestamp)
                bag_end = timestamp if bag_end is None else max(bag_end, timestamp)

        for name, local in local_stats.items():
            if name not in topics:
                topics[name] = local
                continue
            combined = topics[name]
            if combined["type"] != local["type"]:
                raise RuntimeError(f"Conflicting message types for topic {name}")
            if not local["count"]:
                continue
            if not combined["count"]:
                topics[name] = local
                continue
            boundary_gap = local["first"] - combined["last"]
            if boundary_gap >= 0 and combined["count"] and local["count"]:
                combined["max_gap"] = max(combined["max_gap"], boundary_gap)
                combined["gap_sum"] += boundary_gap
                combined["gap_count"] += 1
            elif boundary_gap < 0:
                combined["out_of_order"] += 1
            combined["count"] += local["count"]
            combined["first"] = min(combined["first"], local["first"])
            combined["last"] = max(combined["last"], local["last"])
            combined["min"] = min(combined["min"], local["min"])
            combined["max"] = max(combined["max"], local["max"])
            combined["max_gap"] = max(combined["max_gap"], local["max_gap"])
            combined["gap_sum"] += local["gap_sum"]
            combined["gap_count"] += local["gap_count"]
            combined["out_of_order"] += local["out_of_order"]

    return topics, bag_start, bag_end, len(databases)


def format_time(timestamp):
    if timestamp is None:
        return "n/a"
    return f"{timestamp / 1e9:.3f}"


def report(bag_path, topics, bag_start, bag_end, database_count):
    total_messages = sum(topic["count"] for topic in topics.values())
    duration = ((bag_end - bag_start) / 1e9) if bag_start is not None else 0.0
    print(f"Bag: {bag_path}")
    print(
        f"Storage files: {database_count} | Topics: {len(topics)} | "
        f"Messages: {total_messages} | Duration: {duration:.2f}s"
    )
    if bag_start is not None:
        print(
            f"Timestamp span: {format_time(bag_start)}s - "
            f"{format_time(bag_end)}s (ROS bag timestamps)"
        )
    print()
    heading = (
        f"{'Topic':<52} {'Messages':>10} {'Rate Hz':>9} "
        f"{'Avg gap s':>10} {'Max gap s':>10} {'Out-of-order':>12}  Type"
    )
    print(heading)
    print("-" * len(heading))
    for name, topic in sorted(topics.items()):
        span = (
            (topic["max"] - topic["min"]) / 1e9
            if topic["count"] and topic["min"] is not None and topic["max"] is not None
            else 0.0
        )
        rate = (topic["count"] - 1) / span if span > 0 else 0.0
        average_gap = (
            topic["gap_sum"] / topic["gap_count"] / 1e9
            if topic["gap_count"]
            else 0.0
        )
        max_gap = topic["max_gap"] / 1e9
        print(
            f"{name:<52.52} {topic['count']:>10} {rate:>9.2f} "
            f"{average_gap:>10.3f} {max_gap:>10.3f} "
            f"{topic['out_of_order']:>12}  {topic['type']}"
        )
        if topic["type"] in IMAGE_TYPES:
            print(" " * 4 + "Image stream: dimensions/encoding require decoding message payloads.")

    missing = [
        expected
        for expected in sorted(EXPECTED_TOPICS)
        if not any(matches_topic(actual, expected) for actual in topics)
    ]
    empty = sorted(name for name, topic in topics.items() if topic["count"] == 0)
    print("\nInsights:")
    if not topics or total_messages == 0:
        print("  - Bag has no recorded messages.")
    else:
        print(f"  - Average overall message rate: {total_messages / duration:.2f} Hz" if duration else
              "  - Overall rate unavailable (zero timestamp duration).")
        print(f"  - Topics with no messages: {len(empty)}")
    if missing:
        print("  - Expected robot topics missing:")
        for name in missing:
            print(f"      {name}")
    else:
        print("  - All expected robot topics are present.")
    if any(topic["out_of_order"] for topic in topics.values()):
        print("  - Some topic timestamps move backward in storage order.")
    if any(topic["max_gap"] > 1_000_000_000 for topic in topics.values()):
        print("  - At least one topic has a recorded interval over 1 second.")
    return bool(missing or total_messages == 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or .db3 file")
    parser.add_argument(
        "--expected",
        action="append",
        default=None,
        help="Additional required topic; may be specified more than once",
    )
    args = parser.parse_args()
    if args.expected:
        EXPECTED_TOPICS.update(args.expected)
    bag_path = args.bag.expanduser().resolve()
    try:
        topics, bag_start, bag_end, database_count = inspect_bag(bag_path)
    except (OSError, sqlite3.Error, RuntimeError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 2
    return int(report(bag_path, topics, bag_start, bag_end, database_count))


if __name__ == "__main__":
    sys.exit(main())
