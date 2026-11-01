#!/usr/bin/env python3
"""Isolated ROS domain/topic guard test; no connection to robot command topics."""

def main():
 import json,os,signal,subprocess,time
 from pathlib import Path
 os.environ['ROS_DOMAIN_ID']='99'
 os.environ['ROS_LOCALHOST_ONLY']='1'
 import rclpy
 from rclpy.node import Node
 from rclpy.signals import SignalHandlerOptions
 from geometry_msgs.msg import Twist,TwistStamped
 root=Path(__file__).resolve().parent.parent
 if not (root/'Scripts/guard.py').exists():root=root/'Robot'
 log=(root/'Logs/guard_isolated_test.log').open('w')
 proc=subprocess.Popen(['python3',str(root/'Scripts/guard.py'),'--ros-args',
  '-p','input_topic:=/ma_guard_test/input','-p','output_topic:=/ma_guard_test/output'],stdout=log,stderr=subprocess.STDOUT)
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
 node=Node('ma_guard_isolated_test');seen=[]
 sub=node.create_subscription(Twist,'/ma_guard_test/output',lambda m:seen.append((time.monotonic(),m.linear.x)),10)
 pub=node.create_publisher(TwistStamped,'/ma_guard_test/input',1)
 def spin(seconds):
  until=time.monotonic()+seconds
  while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
 def send(value,offset=0.):
  m=TwistStamped();stamp=node.get_clock().now().nanoseconds+int(offset*1e9)
  m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9);m.twist.linear.x=value;pub.publish(m)
 results={}
 try:
  spin(2.)
  seen.clear()
  for i in range(10):send(.1);spin(.04)
  assert any(v>.09 for t,v in seen),'Valid command not forwarded'
  results['fresh_command_forwarded']=True
  for value in [.26,-.26]:
   seen.clear()
   for i in range(8):send(value);spin(.04)
   assert any(abs(v-value)<1e-6 for t,v in seen),'Maximum command not forwarded'
  results['maximum_forward_reverse_accepted']=True
  last=time.monotonic();seen.clear();spin(.55)
  assert seen and seen[-1][1]==0.,'Timeout did not send zero'
  results['missing_command_stops_within_550ms']=True
  for name,value,offset in [('stale',.1,-2.),('future',.1,2.),('overspeed',.27,0.),('nan',float('nan'),0.)]:
   seen.clear()
   for i in range(5):send(value,offset);spin(.05)
   assert seen and all(v==0. for t,v in seen),name+' command accepted'
   results[name+'_rejected']=True
  results['all_passed']=True
 finally:
  node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
  log.close()
  path=root/'Inventory/guard_test.json';path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(results,indent=2))
  print(json.dumps(results))

if __name__ == '__main__':
 main()
