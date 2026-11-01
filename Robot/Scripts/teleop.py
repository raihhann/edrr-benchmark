#!/usr/bin/env python3
"""Guarded keyboard teleop: release/key timeout stops; no persistent velocity."""
import argparse,math,fcntl,select,signal,sys,termios,time,tty
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--speed',type=float,default=.08)
parser.add_argument('--turn-speed',type=float,default=.35)
args=parser.parse_args()
if not math.isfinite(args.speed) or not 0 < args.speed <= .26:parser.error('--speed must be in (0, 0.26] m/s')
if not math.isfinite(args.turn_speed) or not 0 < args.turn_speed <= .8:parser.error('--turn-speed must be in (0, 0.8] rad/s')
root=Path(__file__).resolve().parent.parent
lock=(root/'Runs/motion.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=Node('ma_turtlebot_keyboard');stopped=[]
for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,lambda s,f:stopped.append(s))
if node.count_publishers('/ma_raihan/nav_cmd_vel'):raise SystemExit('Another motion publisher is active')
pub=node.create_publisher(TwistStamped,'/ma_raihan/nav_cmd_vel',1)
settings=termios.tcgetattr(sys.stdin);velocity=(0.,0.);last_key=0.
def send(v):
 m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.twist.linear.x,m.twist.angular.z=v;pub.publish(m)
print(f'W/S: forward/back {args.speed:g} m/s; A/D: turn {args.turn_speed:g} rad/s; SPACE stop; X exit. Key timeout: .2 seconds.',flush=True)
try:
 tty.setraw(sys.stdin.fileno())
 while not stopped:
  if select.select([sys.stdin],[],[],.04)[0]:
   key=sys.stdin.read(1).lower()
   if key in ['x','\x03']:break
   velocity={'w':(args.speed,0.),'s':(-args.speed,0.),'a':(0.,args.turn_speed),'d':(0.,-args.turn_speed)}.get(key,(0.,0.));last_key=time.monotonic()
  send(velocity if time.monotonic()-last_key<.2 else (0.,0.))
  rclpy.spin_once(node,timeout_sec=0.)
finally:
 for i in range(10):send((0.,0.));time.sleep(.02)
 termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
 node.destroy_node();rclpy.shutdown();fcntl.flock(lock,fcntl.LOCK_UN)
