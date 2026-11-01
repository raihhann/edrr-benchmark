#!/usr/bin/env python3
"""Independent command freshness guard. Publishes zeros on stale/invalid input."""
import math
import time
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class Guard(Node):
    def __init__(self):
        super().__init__('ma_turtlebot_command_guard')
        self.declare_parameter('input_topic', '/ma_raihan/nav_cmd_vel')
        self.declare_parameter('output_topic', '/ma_raihan/driver_cmd_vel')
        self.last = None
        self.command = Twist()
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 1)
        self.sub = self.create_subscription(TwistStamped, self.get_parameter('input_topic').value,
                                            self.receive, 1)
        self.timer = self.create_timer(.02, self.tick)

    def receive(self, msg):
        t = msg.twist
        values = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (not all(math.isfinite(v) for v in values) or not -.05 <= age <= .25
                or abs(t.linear.x) > .26 or abs(t.angular.z) > .8
                or any(abs(v) > 1e-9 for v in [t.linear.y, t.linear.z, t.angular.x, t.angular.y])):
            self.command, self.last = Twist(), None
            return
        self.command, self.last = t, time.monotonic()

    def tick(self):
        self.pub.publish(self.command if self.last is not None and time.monotonic()-self.last < .3
                         else Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: stopped.append(signum))
    node = Guard()
    try:
        while not stopped and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            time.sleep(.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
