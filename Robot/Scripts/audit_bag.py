#!/usr/bin/env python3
"""Decode every recorded message on the robot, including TurtleBot custom types."""
import argparse
import collections
import json
import math
from pathlib import Path
import sqlite3
import statistics
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from control import valid_name


def main():
    p = argparse.ArgumentParser()
    p.add_argument('name')
    a = p.parse_args()
    bag = Path(__file__).resolve().parent.parent/'Bags'/valid_name(a.name)
    report = {'bag': str(bag), 'topics': {}, 'sqlite': [], 'decode_failures': [],
              'hardware_trigger_sync': False, 'source_stamps_preserved': True}
    poses, commands = [], []
    for db in sorted(bag.glob('*.db3')):
        conn = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
        report['sqlite'].append(conn.execute('pragma quick_check').fetchone()[0])
        for tid, name, typ in conn.execute('select id,name,type from topics').fetchall():
            s = report['topics'].setdefault(name, {'type': typ, 'count': 0, 'source_stamps_ns': [],
                                                 'receipt_stamps_ns': [], 'age_ms': [],
                                                 'frames': set(), 'invalid_images': 0})
            try:
                cls = get_message(typ)
            except Exception as error:
                report['decode_failures'].append({'topic': name, 'error': str(error)})
                continue
            for stamp, raw in conn.execute('select timestamp,data from messages where topic_id=? order by id', (tid,)):
                try:
                    msg = deserialize_message(raw, cls)
                    s['count'] += 1
                    s['receipt_stamps_ns'].append(stamp)
                    if hasattr(msg, 'header'):
                        ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                        if ns:
                            s['source_stamps_ns'].append(ns)
                            s['age_ms'].append((stamp-ns)/1e6)
                        s['frames'].add(msg.header.frame_id)
                    if typ == 'sensor_msgs/msg/Image':
                        s['shape'] = [msg.height, msg.width, msg.step]
                        s['encoding'] = msg.encoding
                        s['invalid_images'] += int(not msg.height or not msg.width or len(msg.data) != msg.step*msg.height)
                        if 'preview' not in s and msg.encoding in ('rgb8', 'bgr8'):
                            data = bytearray(msg.data)
                            if msg.encoding == 'bgr8':
                                data[0::3], data[2::3] = data[2::3], data[0::3]
                            if msg.step != msg.width*3:
                                data = b''.join(data[y*msg.step:y*msg.step+msg.width*3] for y in range(msg.height))
                            filename = name.strip('/').replace('/', '_')+'_preview.ppm'
                            (bag/filename).write_bytes(('P6\n%d %d\n255\n' % (msg.width, msg.height)).encode()+data)
                            s['preview'] = filename
                    if typ == 'sensor_msgs/msg/CompressedImage':
                        s['format'] = msg.format
                        s['compressed_size_bytes'] = len(msg.data)
                        s['invalid_images'] += int(not len(msg.data))
                    if typ == 'sensor_msgs/msg/CameraInfo':
                        s['calibrated'] = bool(msg.k[0] > 0 and msg.k[4] > 0)
                        s['K'], s['distortion_model'] = list(msg.k), msg.distortion_model
                    if typ == 'sensor_msgs/msg/LaserScan':
                        valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
                        s['rays_per_scan'], s['frame'] = len(msg.ranges), msg.header.frame_id
                        s['valid_range_samples'] = s.get('valid_range_samples', 0)+len(valid)
                        if valid:
                            s['min_observed_range_m'] = min(s.get('min_observed_range_m', float('inf')), min(valid))
                    if typ == 'sensor_msgs/msg/JointState':
                        s['joint_names'], s['position_count'], s['velocity_count'], s['effort_count'] = (
                            list(msg.name), len(msg.position), len(msg.velocity), len(msg.effort))
                    if typ == 'sensor_msgs/msg/BatteryState':
                        s['voltage_V'] = msg.voltage if math.isfinite(msg.voltage) else None
                        s['percentage'] = msg.percentage if math.isfinite(msg.percentage) else None
                    if typ == 'nav_msgs/msg/Odometry':
                        poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
                    if name == '/ma_raihan/driver_cmd_vel':
                        commands.append([msg.linear.x, msg.angular.z])
                    if name == '/ma_raihan/opencr/registers':
                        pairs = {p.key: p.value for status in msg.status for p in status.values}
                        s['last_register_snapshot'] = pairs
                        ticks = s.setdefault('uptime_values', [])
                        ticks.append(int(pairs['uptime_ms']))
                except Exception as error:
                    report['decode_failures'].append({'topic': name, 'error': str(error)})
        conn.close()
    for name, s in report['topics'].items():
        times = s.pop('source_stamps_ns') or s['receipt_stamps_ns']
        receipts = s.pop('receipt_stamps_ns')
        ages = s.pop('age_ms')
        s['frames'] = sorted(s['frames'])
        s['source_hz'] = (len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None
        gaps = [(b-a)/1e6 for a,b in zip(times,times[1:])]
        s['timestamp_backwards'] = sum(v<0 for v in gaps)
        s['max_source_gap_ms'] = max(gaps) if gaps else None
        s['median_receipt_minus_source_ms'] = statistics.median(ages) if ages else None
        s['peak_abs_receipt_minus_source_ms'] = max(map(abs, ages)) if ages else None
        if 'uptime_values' in s:
            ticks = s.pop('uptime_values')
            s['uptime_advanced'] = bool(len(ticks)>1 and ticks[-1]>ticks[0])
    report['odom_displacement_m'] = math.hypot(poses[-1][0]-poses[0][0], poses[-1][1]-poses[0][1]) if poses else None
    report['driver_commands_all_zero'] = all(v == [0.,0.] for v in commands) if commands else None
    report['last_driver_command'] = commands[-1] if commands else None
    report['total_messages'] = sum(s['count'] for s in report['topics'].values())
    report['storage_and_decode_ok'] = bool(report['sqlite']) and all(v=='ok' for v in report['sqlite']) and not report['decode_failures']
    (bag/'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({'audit': str(bag/'audit.json'), 'ok': report['storage_and_decode_ok'],
                      'messages': report['total_messages'], 'topics': len(report['topics']),
                      'driver_commands_all_zero': report['driver_commands_all_zero']}))


if __name__ == '__main__':
    main()
