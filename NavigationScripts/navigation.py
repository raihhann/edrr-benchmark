#!/usr/bin/env python3
"""
Go1 Navigation Library
Commands the robot to walk a set distance at a set speed.
"""
import argparse
import math
import time
import roslibpy

class Go1Navigator:
    def __init__(self, host='192.168.123.161', port=9090):
        self.host, self.port = host, port
        self.ros = roslibpy.Ros(host=self.host, port=self.port)
        self.cmd_topic = '/go1_controller/cmd_vel'
        self.odom_topic = '/go1_controller/odom'
        self.command = self.odom_sub = self.latest_odom = None
        self.is_connected = False

    def connect(self):
        print(f"Connecting to Go1 at {self.host}:{self.port}...")
        self.ros.run(timeout=10)
        if not self.ros.is_connected:
            raise RuntimeError("Failed to connect to rosbridge.")
        self.command = roslibpy.Topic(self.ros, self.cmd_topic, 'geometry_msgs/Twist')
        self.odom_sub = roslibpy.Topic(self.ros, self.odom_topic, 'nav_msgs/Odometry')
        self.odom_sub.subscribe(lambda msg: setattr(self, 'latest_odom', msg))
        self.is_connected = True
        
        print("Waiting for odometry data...")
        while self.latest_odom is None:
            time.sleep(0.1)
        print("Connected and receiving odometry.")

    def _get_current_pose(self):
        p, q = self.latest_odom['pose']['pose']['position'], self.latest_odom['pose']['pose']['orientation']
        return p['x'], p['y'], math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']), 1.0 - 2.0 * (q['y']**2 + q['z']**2))

    def set_walking_mode(self):
        print("Requesting walking mode...")
        result = roslibpy.Service(self.ros, '/set_mode', 'go1_legged_msgs/SetMode').call(
            roslibpy.ServiceRequest({'mode': 2, 'gait_type': 1}), timeout=5)
        if not result or not result.get('success'):
            raise RuntimeError("Robot rejected walking mode. Is it standing?")

    def stop(self):
        if self.command and self.is_connected:
            print("\nSending stop command...")
            # Publish the stop command a few times with a tiny delay
            # to guarantee the robot receives it before we disconnect
            for _ in range(5):
                self.command.publish(roslibpy.Message({
                    'linear': {'x': 0., 'y': 0., 'z': 0.}, 
                    'angular': {'x': 0., 'y': 0., 'z': 0.}
                }))
                time.sleep(0.1)

    def walk(self, target_distance, speed):
        if not self.is_connected: self.connect()
        self.set_walking_mode()
        start_x, start_y, start_heading = self._get_current_pose()
        
        try:
            while True:
                curr_x, curr_y, curr_heading = self._get_current_pose()
                dx, dy = curr_x - start_x, curr_y - start_y
                dist = math.hypot(dx, dy)
                
                lat_drift = -dx * math.sin(start_heading) + dy * math.cos(start_heading)
                head_err = math.atan2(math.sin(curr_heading - start_heading), math.cos(curr_heading - start_heading))
                
                if dist >= target_distance:
                    print(f"\nTarget reached: {dist:.2f}m")
                    break

                yaw_cmd = max(-0.15, min(0.15, -0.8 * head_err - 0.3 * lat_drift))
                self.command.publish(roslibpy.Message({
                    'linear': {'x': float(speed), 'y': 0., 'z': 0.},
                    'angular': {'x': 0., 'y': 0., 'z': float(yaw_cmd)}
                }))
                print(f"\rProgress: {dist:.2f} / {target_distance}m", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nWalk interrupted.")
        finally:
            self.stop()

    def disconnect(self):
        self.stop()
        if self.command: self.command.unadvertise()
        if self.odom_sub: self.odom_sub.unsubscribe()
        self.ros.terminate()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=2.0)
    parser.add_argument('--speed', type=float, default=0.2)
    args = parser.parse_args()
    nav = Go1Navigator()
    try:
        nav.walk(args.distance, args.speed)
    finally:
        nav.disconnect()