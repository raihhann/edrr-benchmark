#!/usr/bin/env python2
"""Require sustained, fresh data from all configured D455 streams."""
from __future__ import print_function
import argparse,json,sys,time
TOPICS=['/camera/color/image_raw','/camera/depth/image_rect_raw','/camera/gyro/sample','/camera/accel/sample']


def inspect(rows,now):
    missing=[];metrics={}
    for topic in TOPICS:
        values=list(rows.get(topic,[]))
        if len(values)<3:missing.append(topic);continue
        duration=values[-1][0]-values[0][0]
        rate=(len(values)-1)/duration if duration>0 else 0.
        minimum=12. if '/color/' in topic else 6. if '/depth/' in topic else 40.
        age=now-values[-1][1];gap=max([b[0]-a[0] for a,b in zip(values,values[1:])] or [0.])
        metrics[topic]=dict(count=len(values),hz=rate,last_source_age_ms=age*1000,max_gap_ms=gap*1000)
        if duration<3. or now-values[-1][0]>.5 or not -.05<=age<=.5 or rate<minimum or gap>.6:missing.append(topic)
    return missing,metrics


def main():
    import rospy
    from sensor_msgs.msg import Image,Imu
    parser=argparse.ArgumentParser();parser.add_argument('--timeout',type=float,default=45.)
    args=parser.parse_args();rospy.init_node('ma_realsense_readiness',anonymous=True,disable_signals=True)
    rows={};errors=[];subscriptions=[]
    def callback(topic):
        def receive(msg):
            if isinstance(msg,Image) and (msg.width*msg.height==0 or len(msg.data)!=msg.step*msg.height):
                errors.append('Invalid image '+topic);return
            rows.setdefault(topic,[]).append((time.time(),msg.header.stamp.to_sec(),int(msg.header.seq)))
        return receive
    for topic in TOPICS:
        subscriptions.append(rospy.Subscriber(topic,Image if '/image_' in topic else Imu,callback(topic),queue_size=1))
    deadline=time.time()+args.timeout
    while not rospy.is_shutdown() and time.time()<deadline:
        missing,metrics=inspect(rows,time.time())
        if not missing and not errors:
            print('RealSense sustained streams verified: '+json.dumps(metrics,sort_keys=True));return 0
        time.sleep(.1)
    missing,metrics=inspect(rows,time.time())
    print('RealSense not ready; missing/stale/unstable: '+str(missing)+'; '+str(errors[:3]),file=sys.stderr)
    return 1

if __name__=='__main__':sys.exit(main())
