#!/usr/bin/env python3
"""On-robot native ROS 2 recording and optional bounded straight-line navigation."""
import argparse
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from control import valid_name, yaw, relative, command

ROOT = Path(__file__).resolve().parent.parent
BASE_REQUIRED = ['/odom', '/imu', '/magnetic_field', '/joint_states', '/sensor_state', '/battery_state']


def write_json(path, value):
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False))
    tmp.replace(path)


class Monitor(Node):
    def __init__(self):
        super().__init__('ma_turtlebot_capture_monitor')
        self.stats, self.subs, self.types = {}, {}, {}
        self.odom, self.pose, self.odom_received = None, None, 0.
        self.scan, self.scan_received = None, 0.
        self.last_pose = None
        self.pose_error = None
        self.scan_sub = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom,
                                                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.discover()
        self.create_timer(1., self.discover)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        try:
            pose = p.x, p.y, yaw(msg.pose.pose.orientation)
            if not all(math.isfinite(v) for v in pose):
                raise ValueError('Non-finite odometry')
            now = time.monotonic()
            if self.last_pose:
                old, old_t = self.last_pose
                if math.hypot(pose[0]-old[0], pose[1]-old[1]) > max(.06, (now-old_t)*.5+.02):
                    self.pose_error = 'Odometry jump/reset'
            self.last_pose = pose, now
            self.odom, self.pose, self.odom_received = msg, pose, now
        except Exception as error:
            self.pose_error = str(error)

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if topic in self.subs or len(types) != 1:
                continue
            pubs = self.get_publishers_info_by_topic(topic)
            if not pubs:
                continue
            try:
                cls = get_message(types[0])
                durable = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
                qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
                header_first = next(iter(cls.get_fields_and_field_types()), '') == 'header'
                self.types[topic] = types[0]
                self.stats[topic] = {'count': 0, 'bytes': 0, 'first': None, 'last': None,
                                     'last_mono': None, 'ages_ms': collections.deque(maxlen=10000)}
                self.subs[topic] = self.create_subscription(cls, topic, self.callback(topic, header_first), qos, raw=True)
                if types[0] == 'sensor_msgs/msg/LaserScan' and self.scan_sub is None:
                    def scan_cb(msg):
                        self.scan, self.scan_received = msg, time.monotonic()
                    self.scan_sub = self.create_subscription(cls, topic, scan_cb, qos)
            except Exception as error:
                self.get_logger().warning('Cannot monitor %s: %s' % (topic, error))

    def callback(self, topic, header_first):
        def receive(data):
            now, wall = time.monotonic(), time.time()
            s = self.stats[topic]
            s['count'] += 1
            s['bytes'] += len(data)
            s['first'] = wall if s['first'] is None else s['first']
            s['last'], s['last_mono'] = wall, now
            if header_first and len(data) >= 12 and data[:2] in (b'\x00\x01', b'\x00\x00'):
                sec, nsec = struct.unpack_from('<iI' if data[1] else '>iI', data, 4)
                if 0 <= nsec < 1000000000 and sec > 0:
                    s['ages_ms'].append((wall-sec-nsec/1e9)*1000)
        return receive

    def required(self, extra):
        sensors = [n for n, typ in self.types.items() if typ in
                   ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/LaserScan',
                    'sensor_msgs/msg/PointCloud2')]
        # Require actual camera/depth/laser publishers, not optional unused transports.
        return sorted(set(BASE_REQUIRED+sensors+extra))

    def health(self, required, strict=True):
        problems = []
        now = time.monotonic()
        for topic in required:
            s = self.stats.get(topic)
            if not s or s['count'] < 2:
                problems.append(topic+': no actual messages')
                continue
            period = 1.5 if self.types[topic] in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
                                                'sensor_msgs/msg/PointCloud2') else .5
            if now-s['last_mono'] > period:
                problems.append(topic+': stale')
            if s['ages_ms'] and abs(s['ages_ms'][-1]) > 500:
                problems.append(topic+': source timestamp differs by >500 ms')
        if self.pose_error:
            problems.append(self.pose_error)
        if strict and problems:
            raise RuntimeError('; '.join(problems))
        return not problems

    def front_clear(self):
        if self.scan is None:
            return
        if time.monotonic()-self.scan_received > .5:
            raise RuntimeError('Laser scan is stale')
        values = []
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min+i*self.scan.angle_increment
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(20):
                if math.isinf(distance) and distance > 0:
                    values.append(float('inf'))
                elif math.isfinite(distance) and self.scan.range_min <= distance <= self.scan.range_max:
                    values.append(distance)
        if not values or min(values) < .35:
            raise RuntimeError('Obstacle/unknown front scan within stopping threshold')


def audit_counts(bag):
    import sqlite3
    counts, integrity = {}, []
    for db in bag.glob('*.db3'):
        c = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
        integrity.append(c.execute('pragma quick_check').fetchone()[0])
        for name, typ, count in c.execute('select t.name,t.type,count(m.id) from topics t left join messages m on t.id=m.topic_id group by t.id'):
            entry = counts.setdefault(name, {'type': typ, 'messages': 0})
            entry['messages'] += count
        c.close()
    return counts, integrity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['record', 'navigate'])
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--distance', type=float, default=1.)
    parser.add_argument('--speed', type=float, default=.1)
    parser.add_argument('--bag-name')
    parser.add_argument('--startup-timeout', type=float, default=20.)
    parser.add_argument('--require-topic', action='append', default=[])
    args = parser.parse_args()
    if not (math.isfinite(args.duration) and 0 < args.duration <= 3600 and
            math.isfinite(args.distance) and 0 < args.distance <= 20 and
            math.isfinite(args.speed) and .02 <= args.speed <= .26 and
            math.isfinite(args.startup_timeout) and 1 <= args.startup_timeout <= 120):
        parser.error('duration (0,3600] s; distance (0,20] m; speed [0.02,0.26] m/s; startup timeout [1,120] s')
    name = valid_name(args.bag_name or datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8])
    bag = ROOT/'Bags'/name
    if bag.exists():
        parser.error('Bag name exists; refusing overwrite: '+str(bag))
    (ROOT/'Runs').mkdir(exist_ok=True)
    lock = (ROOT/'Runs/capture.lock').open('a')
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    motion_lock = None
    if args.mode == 'navigate':
        motion_lock = (ROOT/'Runs/motion.lock').open('a')
        fcntl.flock(motion_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    (ROOT/'Runs/runner.pid').write_text(str(os.getpid()))
    report = {'name': name, 'mode': args.mode, 'arguments': vars(args), 'status': 'preparing',
              'started_utc': datetime.now(timezone.utc).isoformat(), 'events': [],
              'recording_location': 'robot', 'hardware_trigger_sync': False,
              'timestamp_semantics': 'Native rosbag2 receipt time; original source headers preserved'}
    cancelled = []
    def stop_signal(signum, frame):
        cancelled.append(signum)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node, pub, proc, log = Monitor(), None, None, None
    code = 0

    def event(kind, **fields):
        obj = {'event': kind, 'time_ns': time.time_ns(), **fields}
        report['events'].append(obj)
        print(json.dumps(obj), flush=True)

    def tick(timeout=.02, health=False):
        if cancelled:
            raise KeyboardInterrupt()
        if proc is not None and proc.poll() is not None:
            raise RuntimeError('rosbag recorder exited unexpectedly')
        rclpy.spin_once(node, timeout_sec=timeout)
        if health:
            node.health(required)

    try:
        deadline = time.monotonic()+args.startup_timeout
        while time.monotonic() < deadline and not all(t in node.types for t in BASE_REQUIRED):
            tick(.1)
        node.discover()
        required = node.required(args.require_topic)
        report['required_topics'] = required
        report['topic_types_at_start'] = node.types.copy()
        report['optional_hardware'] = {'laser_observed': any(t.endswith('/LaserScan') for t in node.types.values()),
                                       'camera_observed': any(t.endswith(('/Image', '/CompressedImage')) for t in node.types.values())}
        for topic in node.types:
            if topic in required:
                node.get_logger().info('Required actual stream: '+topic)
        runfile = ROOT/'Runs'/(name+'.json')
        write_json(runfile, report)
        cmd = ['ros2', 'bag', 'record', '-a', '--include-hidden-topics', '-s', 'sqlite3',
               '--max-cache-size', '67108864', '--node-name', 'ma_native_bag', '-o', str(bag)]
        log = (ROOT/'Logs'/(name+'_recorder.log')).open('w')
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        stable = None
        deadline = time.monotonic()+args.startup_timeout
        while time.monotonic() < deadline:
            tick(.01)
            subscribed = all(any(i.node_name == 'ma_native_bag' for i in node.get_subscriptions_info_by_topic(t))
                             for t in required)
            if subscribed and node.health(required, False):
                stable = stable or time.monotonic()
                if time.monotonic()-stable >= 1.5:
                    break
            else:
                stable = None
        else:
            node.health(required)
            raise RuntimeError('Recorder subscription/readiness timeout')
        event('recording_ready', bag=str(bag), required_topics=required)
        if args.mode == 'record':
            until = time.monotonic()+args.duration
            while time.monotonic() < until:
                tick(.02, True)
        else:
            if node.pose is None or time.monotonic()-node.odom_received > .3:
                raise RuntimeError('Fresh odometry is required for movement')
            if node.count_publishers('/ma_raihan/nav_cmd_vel'):
                raise RuntimeError('Another navigation/teleop publisher is active')
            outputs = node.get_publishers_info_by_topic('/ma_raihan/driver_cmd_vel')
            if len(outputs) != 1 or outputs[0].node_name != 'ma_turtlebot_command_guard':
                raise RuntimeError('Expected exactly one command guard at the driver input')
            pub = node.create_publisher(TwistStamped, '/ma_raihan/nav_cmd_vel', 1)
            start, previous, last = node.pose, 0., time.monotonic()
            nav_deadline = last+max(10., args.distance/args.speed*3+5)
            event('motion_started', target_m=args.distance, speed_m_s=args.speed, start_pose=start)
            next_command, next_report = last, last
            while True:
                tick(.005, True)
                now = time.monotonic()
                if now > nav_deadline:
                    raise RuntimeError('Navigation wall-clock timeout')
                if now-node.odom_received > .3:
                    raise RuntimeError('Odometry stopped updating')
                node.front_clear()
                forward, lateral, heading = relative(start, node.pose)
                if now < next_command:
                    continue
                velocity, turn, done = command(args.distance, args.speed, forward, lateral, heading,
                                               previous, now-last)
                msg = TwistStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.linear.x, msg.twist.angular.z = velocity, turn
                pub.publish(msg)
                previous, last, next_command = velocity, now, now+.05
                if now >= next_report:
                    event('motion_progress', forward_m=forward, lateral_m=lateral,
                          heading_deg=math.degrees(heading), command_m_s=velocity)
                    next_report = now+1.
                if done:
                    event('target_reached', forward_m=forward, lateral_m=lateral)
                    break
        if pub:
            for _ in range(10):
                z = TwistStamped(); z.header.stamp = node.get_clock().now().to_msg(); pub.publish(z)
                tick(.02)
        until = time.monotonic()+1.
        while time.monotonic() < until:
            tick(.02, True)
        report['status'] = 'complete'
    except KeyboardInterrupt:
        report['status'], report['error'], code = 'interrupted', 'User signal/disconnected SSH', 130
    except Exception as error:
        report['status'], report['error'], code = 'aborted', str(error), 1
        event('error', detail=str(error))
    finally:
        if pub:
            for _ in range(10):
                z = TwistStamped(); z.header.stamp = node.get_clock().now().to_msg(); pub.publish(z)
                time.sleep(.02)
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
                report['status'], report['error'], code = 'aborted', 'Recorder did not finalize normally', 1
        if log:
            log.close()
        report['monitor_topics'] = {}
        for topic, s in node.stats.items():
            ages = sorted(s['ages_ms'])
            report['monitor_topics'][topic] = {k: v for k, v in s.items() if k not in ('ages_ms', 'last_mono')}
            report['monitor_topics'][topic].update(
                median_age_ms=ages[len(ages)//2] if ages else None,
                max_abs_age_ms=max(map(abs, ages)) if ages else None)
        if bag.exists():
            counts, integrity = audit_counts(bag)
            report['bag_topics'], report['sqlite_integrity'] = counts, integrity
            report['missing_required_in_bag'] = [t for t in report.get('required_topics', [])
                                                  if counts.get(t, {}).get('messages', 0) < 2]
            if report['status'] == 'complete' and (not integrity or any(v != 'ok' for v in integrity)
                    or report['missing_required_in_bag'] or not (bag/'metadata.yaml').exists()):
                report['status'], report['error'], code = 'partial', 'Final bag validation failed', 1
            write_json(bag/'experiment.json', report)
        write_json(ROOT/'Runs'/(name+'.json'), report)
        event('bag_finalized', path=str(bag), name=name, status=report['status'])
        node.destroy_node()
        rclpy.shutdown()
        if motion_lock is not None:
            fcntl.flock(motion_lock.fileno(), fcntl.LOCK_UN)
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    return code


if __name__ == '__main__':
    sys.exit(main())
