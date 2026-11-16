#!/usr/bin/env python2
"""Bounded recovery of managed Unitree cameras after USB device re-enumeration."""
from __future__ import print_function
import glob
import os
import re
import signal
import subprocess
import sys
import time
import rosgraph
import rosnode
import rospy

ROOT=os.path.expanduser('~/MA_Raihan')
BINARY=ROOT+'/ros_ws/devel/lib/ma_raihan_sensors/ma_camera'
RUNNING=True
attempts={}
seen={}
received={}


def log(message):print(time.strftime('%Y-%m-%d %H:%M:%S')+' '+message);sys.stdout.flush()
def stop(signum,frame):
    global RUNNING
    RUNNING=False


def managed(pid):
    try:
        with open('/proc/%d/cmdline'%pid,'rb') as handle:args=handle.read().split(b'\0')
        return bool(args and args[0].decode()==BINARY)
    except (IOError,ValueError):return False


def tick(role):
    active=set()
    publications=rosgraph.Master('/ma_camera_recovery').getSystemState()[0]
    for pidfile in glob.glob(ROOT+'/pids/camera_[0-9]*'):
        try:
            device=int(pidfile.rsplit('_',1)[1]);pid=int(open(pidfile).read())
            if not managed(pid):continue
            first=seen.setdefault(pid,time.time())
            node='/ma_camera_device_'+str(device)
            images=[name for name,nodes in publications if node in nodes and '/image_rect/compressed' in name]
            stalled=time.time()-first>20 and (not images or all(time.time()-received.get(topic,0)>8 for topic in images))
            if os.path.exists('/dev/video%d'%device) and not stalled:active.add(device);continue
            log('Disconnected or stalled /dev/video%d; terminating only managed camera PID %d'%(device,pid))
            os.kill(pid,signal.SIGTERM)
            deadline=time.time()+2
            while managed(pid) and time.time()<deadline and RUNNING:time.sleep(.1)
            if managed(pid):os.kill(pid,signal.SIGKILL)
            rosnode.cleanup_master_blacklist(rosgraph.Master('/ma_camera_recovery'),[node])
        except (IOError,OSError,ValueError) as error:log(str(error))
    for path in glob.glob('/dev/video[0-9]*'):
        device=int(path.rsplit('video',1)[1])
        if device in active:continue
        properties=subprocess.check_output(['udevadm','info','--query=property','--name='+path]).decode()
        if 'ID_VENDOR_ID=1bcf' not in properties or 'ID_MODEL_ID=2cd1' not in properties:continue
        recent=[t for t in attempts.get(device,[]) if time.time()-t<60]
        if len(recent)>=3:continue
        attempts[device]=recent+[time.time()]
        config=ROOT+'/calibration/device_%d.yaml'%device
        if not os.path.exists(config):
            # Same supported SDK mode as the existing MA_Raihan startup script.
            lines=['%YAML:1.0','---']
            for name,values in [('DeviceNode',[device]),('FrameSize',[928,400]),('RectifyFrameSize',[232,200]),
                                ('FrameRate',[30]),('Algorithm',[1]),('LogLevel',[1]),('Threshold',[190]),('Mindepth',[.2]),('Maxdepth',[3.])]:
                lines+= [name+': !!opencv-matrix','   rows: 1','   cols: '+str(len(values)),'   dt: d',
                          '   data: [ '+', '.join(str(v) for v in values)+' ]']
            with open(config,'w') as handle:handle.write('\n'.join(lines)+'\n')
        width,height=(116,100) if role=='14' else (232,200)
        args=[BINARY,'__name:=ma_camera_device_'+str(device),'_device:='+str(device),
              '_publish_stereo:='+os.environ.get('MA_STEREO','false'),
              '_depth_width:='+str(width),'_depth_height:='+str(height),
              '_sdk_config:='+config,'_calibration_directory:='+ROOT+'/calibration']
        with open(ROOT+'/logs/camera_%d.log'%device,'a') as handle:
            process=subprocess.Popen(args,stdin=open('/dev/null'),stdout=handle,stderr=subprocess.STDOUT,close_fds=True)
        with open(ROOT+'/pids/camera_%d'%device,'w') as handle:handle.write(str(process.pid))
        log('Opened current Unitree USB device %d as PID %d'%(device,process.pid))


if __name__=='__main__':
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    role=open(ROOT+'/role').read().strip()
    rospy.init_node('ma_camera_recovery',disable_signals=True)
    subscriptions=[]
    for camera in {'13':['front','chin'],'14':['left','right'],'15':['rear_down']}[role]:
        topic='/ma_raihan/cameras/'+camera+'/image_rect/compressed'
        def receive(message,topic=topic):received[topic]=time.time()
        subscriptions.append(rospy.Subscriber(topic,rospy.AnyMsg,receive,queue_size=1))
    while RUNNING:
        try:tick(role)
        except Exception as error:log('Recovery error: '+str(error))
        for i in range(20):
            if not RUNNING:break
            time.sleep(.1)
