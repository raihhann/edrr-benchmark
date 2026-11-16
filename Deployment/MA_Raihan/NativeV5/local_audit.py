#!/usr/bin/env python2
"""Decode a finalized native ROS1 bag locally, then create its SHA256 manifest."""
from __future__ import print_function
import collections
import hashlib
import json
import math
import os
import re
import sys
import time
import rosbag


def percentile(values,fraction):
    values=sorted(values)
    return values[int((len(values)-1)*fraction)] if values else None


def audit(folder):
    with open(os.path.join(folder,'run.json')) as handle:state=json.load(handle)
    start=state.get('assessment_start_ns');end=state.get('recording_stop_ns') or state.get('finished_ns')
    streams={};last={};metadata={};decode_failures=[];motor_counts=set();temperatures=[]
    bag_path=os.path.join(folder,'capture.bag')
    total=0;final_command=None
    with rosbag.Bag(bag_path,'r') as bag:
        duration=bag.get_end_time()-bag.get_start_time()
        for name,info in bag.get_type_and_topic_info().topics.items():
            streams[name]=dict(type=info.msg_type,messages=0,stable_messages=0,
                               source_stamps=[],receipt_stamps=[],age_ms=[],
                               sequence_missing=0,sequence_observed=0,sequence_resets=0,
                               duplicate_sequences=0,stamp_backwards=0,stamp_duplicates=0,
                               header_messages=0)
        for name,msg,receipt,connection in bag.read_messages(return_connection_header=True):
            total+=1;row=streams[name];row['messages']+=1
            receipt_ns=receipt.to_nsec()
            stable=bool(start and receipt_ns>=start and (not end or receipt_ns<=end))
            if hasattr(msg,'data') and name.endswith('/metadata'):
                try:metadata[name.rsplit('/',1)[0]]=json.loads(msg.data)
                except ValueError:pass
            try:
                kind=getattr(msg,'_type','')
                if kind=='sensor_msgs/CompressedImage':
                    import cv2
                    import numpy as np
                    image=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
                    if image is None or not image.size:raise ValueError('Image did not decode')
                elif kind=='sensor_msgs/PointCloud2':
                    if len(msg.data)!=msg.row_step*msg.height or msg.row_step<msg.width*msg.point_step:
                        raise ValueError('Point-cloud buffer dimensions are invalid')
                elif kind=='sensor_msgs/Image':
                    if len(msg.data)!=msg.step*msg.height:raise ValueError('Raw image buffer dimensions are invalid')
                if name=='/ma_raihan/robot/motor_states':
                    motor_counts.add(len(msg.motor_state))
                    for motor in msg.motor_state:
                        for field in ('q','dq','tauEst','temperature'):
                            value=getattr(motor,field)
                            if math.isnan(value) or math.isinf(value):raise ValueError('Invalid motor '+field)
                        temperatures.append(motor.temperature)
                if name=='/go1_controller/cmd_vel':
                    final_command=all(abs(v)<1e-9 for v in [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z])
            except Exception as error:
                if len(decode_failures)<100:decode_failures.append(dict(topic=name,message=str(error),number=row['messages']))
            if not stable:continue
            row['stable_messages']+=1;row['receipt_stamps'].append(receipt_ns)
            if not hasattr(msg,'header'):continue
            row['header_messages']+=1
            stamp=msg.header.stamp.to_nsec();sequence=int(msg.header.seq)
            key=(name,connection.get('callerid','unknown'))
            if key in last:
                previous,previous_stamp=last[key];delta=(sequence-previous)%(2**32)
                if 0<delta<1000000:row['sequence_missing']+=delta-1;row['sequence_observed']+=delta
                elif delta==0:row['duplicate_sequences']+=1
                else:row['sequence_resets']+=1
                if stamp<previous_stamp:row['stamp_backwards']+=1
                if stamp==previous_stamp:row['stamp_duplicates']+=1
            last[key]=(sequence,stamp)
            if stamp:
                row['source_stamps'].append(stamp);row['age_ms'].append((receipt_ns-stamp)/1e6)
        for name,row in streams.items():
            receipts=row.pop('receipt_stamps');stamps=row['source_stamps'];ages=row.pop('age_ms')
            row['receipt_hz']=(len(receipts)-1)*1e9/(receipts[-1]-receipts[0]) if len(receipts)>1 and receipts[-1]>receipts[0] else None
            row['source_hz']=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None
            row['max_gap_ms']=max([(b-a)/1e6 for a,b in zip(receipts,receipts[1:])] or [0.])
            if start and end and receipts:
                row['max_gap_ms']=max(row['max_gap_ms'],(receipts[0]-start)/1e6,(end-receipts[-1])/1e6)
            row['p95_age_ms']=percentile(ages,.95);row['max_age_ms']=max(ages) if ages else None
            row['min_age_ms']=min(ages) if ages else None
            row['sequence_loss_percent']=100.*row['sequence_missing']/row['sequence_observed'] if row['sequence_observed'] else None
            # Retain compact stamp vectors only for cross-controller camera/IMU association.
            if name!='/ma_raihan/robot/imu' and '/image_rect/compressed' not in name and name!='/camera/color/image_raw':row.pop('source_stamps')
    expected=set(state.get('initial_topics',{}))
    quiet={'/rosout','/rosout_agg','/client_count','/connected_clients','/go1_controller/cmd_vel'}
    # A fixed camera can publish only latched static transforms; dynamic /tf is then legitimately quiet.
    if streams.get('/tf_static',{}).get('messages',0)>0:quiet.add('/tf')
    silent=expected-set(streams)
    intentional=[]
    for base,meta in metadata.items():
        if meta.get('native_depth_available') is False:
            for name in list(silent):
                if name==base+'/depth/camera_info':intentional.append(name);silent.remove(name)
    return dict(schema_version=1,state=state,topics=streams,message_count=total,duration_s=duration,
                decoded_all=True,decode_failures=decode_failures,motor_counts=sorted(motor_counts),
                motor_temperature_range=[min(temperatures),max(temperatures)] if temperatures else None,
                final_command_zero=final_command,missing_sensor_topics=sorted(silent-quiet),
                quiet_topics=sorted(silent&quiet),intentionally_unavailable_topics=intentional,
                new_topics=sorted(set(state.get('final_topics',{}))-expected))


def main():
    name=sys.argv[1]
    if not re.match(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,99}$',name):raise ValueError('Invalid run id')
    folder=os.path.expanduser('~/MA_Raihan/captures/'+name)
    try:result=audit(folder)
    except Exception as error:result=dict(schema_version=1,decoded_all=False,error=str(error),topics={},message_count=0)
    with open(os.path.join(folder,'audit.json'),'w') as handle:json.dump(result,handle,indent=2)
    manifest={}
    for filename in os.listdir(folder):
        path=os.path.join(folder,filename)
        if filename=='manifest.json' or not os.path.isfile(path) or os.path.islink(path):continue
        digest=hashlib.sha256()
        with open(path,'rb') as handle:
            for chunk in iter(lambda:handle.read(1024*1024),b''):digest.update(chunk)
        manifest[filename]=dict(sha256=digest.hexdigest(),bytes=os.path.getsize(path))
    with open(os.path.join(folder,'manifest.json'),'w') as handle:json.dump(manifest,handle,indent=2)
    print(json.dumps(dict(messages=result['message_count'],decoded_all=result['decoded_all'],files=len(manifest))))


if __name__=='__main__':main()
