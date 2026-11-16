#!/usr/bin/env python2
"""Native on-controller recorder; stdin is a leased command channel, not sensor transport."""
from __future__ import print_function
import argparse
import ctypes
import hashlib
import json
import math
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from local_paths import Path


class Timespec(ctypes.Structure):
    _fields_ = [('sec', ctypes.c_long), ('nsec', ctypes.c_long)]


_libc = ctypes.CDLL('librt.so.1', use_errno=True)
def monotonic():
    value = Timespec()
    if _libc.clock_gettime(1, ctypes.byref(value)):
        raise OSError(ctypes.get_errno(), 'clock_gettime')
    return value.sec + value.nsec / 1e9


def validate_run_id(value):
    if not re.match(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,99}$', value):
        raise ValueError('Invalid run id')
    return value


def pose(message):
    p = message.pose.pose.position; q = message.pose.pose.orientation
    result = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    if not all(not math.isnan(v) and not math.isinf(v) for v in result):
        raise ValueError('Non-finite odometry')
    return result


class Capture(object):
    def __init__(self, args):
        import rospy
        import rosgraph
        self.args = args; self.rospy = rospy
        self.root = Path(os.path.expanduser('~/MA_Raihan'))
        self.folder = self.root/'captures'/validate_run_id(args.run_id)
        self.folder.mkdir(parents=True, exist_ok=False)
        (self.root/'pids').mkdir(parents=True,exist_ok=True)
        (self.root/'pids'/('native_'+args.run_id)).write_text(str(os.getpid()))
        self.output_lock = threading.Lock(); self.stats_lock = threading.Lock()
        self.commands = []; self.last_heartbeat = monotonic(); self.stop_reason = None
        self.rows = {}; self.subscriptions = []; self.ready = False
        self.requested_finish = False; self.assessment = None; self.recorder = None
        self.nav_pose = None; self.nav_pose_time = 0.; self.nav_mode = None
        self.nav_started = None; self.nav_origin = None; self.target_reached = False
        self.motion = None; self.motion_value = (0., 0.); self.plan_time = 0.
        self.motion_running = False; self.mode_requested = None
        self.mode_response = None; self.failure = None
        self.events = (self.folder/'events.jsonl').open('a')
        suffix=hashlib.sha256(args.run_id.encode('ascii')).hexdigest()[:16]
        self.node_name = '/ma_local_'+args.role+'_'+suffix
        self.record_node = '/ma_record_'+args.role+'_'+suffix
        rospy.init_node(self.node_name[1:], disable_signals=True)
        self.master = rosgraph.Master(self.node_name)
        if args.role=='pi':
            # Factory cameras used this master before takeover. Remove only
            # confirmed-dead factory registrations, never active publishers.
            import rosnode
            import xmlrpclib
            nodes=set(rosnode.get_node_names())
            previous_timeout=socket.getdefaulttimeout();socket.setdefaulttimeout(.5)
            try:
                for node in ['/camera1','/camera3','/camera4','/camera5']:
                    if node not in nodes:continue
                    try:
                        uri=self.master.lookupNode(node)
                        response=xmlrpclib.ServerProxy(uri).getPid(self.node_name)
                        dead=response[0]!=1
                    except Exception:dead=True
                    if dead:rosnode.cleanup_master_blacklist(self.master,[node])
            finally:socket.setdefaulttimeout(previous_timeout)
        self.inventory = dict(self.master.getPublishedTopics(''))
        self.excluded = []
        if args.skip_lidar:self.excluded.append(r'/rslidar_.*')
        if args.skip_realsense:self.excluded.append(r'/camera/(color|depth|aligned_depth.*|infra.*|gyro|accel|extrinsics).*')
        self.inventory = dict((n,t) for n,t in self.inventory.items() if not self.is_excluded(n))
        cameras = {'13':['front','chin'], '14':['left','right'], '15':['rear_down']}
        self.required = []
        if args.role == 'pi':
            self.required = ['/ma_raihan/robot/'+n for n in ['imu','battery','joint_states_measured','motor_states','low_state']]
            if args.navigate:self.required.append('/ma_raihan/robot/odom')
            if args.include_lidar:self.required.append('/rslidar_points')
        else:
            self.required = ['/ma_raihan/cameras/'+n+'/image_rect/compressed'
                             for n in cameras[args.role] if n not in args.optional_camera]
        if args.include_realsense and args.role==args.realsense_controller:
            self.required.extend(['/camera/color/image_raw','/camera/depth/image_rect_raw',
                '/camera/color/camera_info','/camera/depth/camera_info','/camera/gyro/sample','/camera/accel/sample'])
        absent = set(self.required)-set(self.inventory)
        registration_deadline=monotonic()+args.startup_timeout
        while absent and monotonic()<registration_deadline:
            # Camera SDK calibration/opening can finish after start.sh returns.
            time.sleep(.2)
            self.inventory=dict((n,t) for n,t in self.master.getPublishedTopics('') if not self.is_excluded(n))
            absent=set(self.required)-set(self.inventory)
        if absent:raise RuntimeError('Required topics are absent: '+str(sorted(absent)))
        for topic in self.required:
            self.rows[topic] = dict(count=0, last=0., stamp=0., age_s=0.)
            self.subscriptions.append(rospy.Subscriber(topic, rospy.AnyMsg, self.callback(topic), queue_size=100))
        if args.navigate:
            if args.role!='pi':raise RuntimeError('Navigation is permitted only on the Pi')
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            self.Twist = Twist
            publishers = dict(self.master.getSystemState()[0]).get('/go1_controller/cmd_vel', [])
            if publishers:raise RuntimeError('Existing velocity publisher(s): '+str(publishers))
            self.motion = rospy.Publisher('/go1_controller/cmd_vel', Twist, queue_size=1)
            # One raw subscription per topic; decode locally for navigation.
            # rospy shares transports, so mixing AnyMsg and Odometry is unsafe.
            # Include the intentionally advertised command topic before record discovery.
            self.inventory['/go1_controller/cmd_vel'] = 'geometry_msgs/Twist'
        self.save_state('preparing')

    def is_excluded(self, topic):
        return any(re.match('^'+pattern+'$',topic) for pattern in self.excluded)

    def callback(self, topic):
        def receive(message):
            stamp = 0.
            if len(message._buff)>=12:
                seq, sec, nsec = struct.unpack_from('<III',message._buff)
                if 0<nsec<1000000000 or nsec==0:stamp=sec+nsec/1e9
            with self.stats_lock:
                row=self.rows[topic];row['count']+=1;row['last']=monotonic()
                row['stamp']=stamp;row['age_s']=time.time()-stamp if stamp else 1e9
            if self.args.navigate and topic=='/ma_raihan/robot/odom':
                try:
                    from nav_msgs.msg import Odometry
                    decoded=Odometry();decoded.deserialize(message._buff)
                    self.odom(decoded)
                except Exception as error:
                    self.stop_reason='Odometry decode failed: '+str(error)
        return receive

    def odom(self, message):
        try:
            frame=message.header.frame_id
            if self.nav_started is not None and frame!=getattr(self,'nav_frame',frame):self.stop_reason='Odometry frame changed'
            self.nav_frame=frame;self.nav_pose=pose(message);self.nav_pose_time=monotonic()
        except ValueError:self.stop_reason='Invalid odometry'

    def emit(self, event, **values):
        values.update(event=event, role=self.args.role, unix_ns=int(time.time()*1e9))
        with self.output_lock:
            line=json.dumps(values,allow_nan=False)
            self.events.write(line+'\n');self.events.flush()
            print(line);sys.stdout.flush()

    def save_state(self, status):
        data=dict(schema_version=1,role=self.args.role,run_id=self.args.run_id,status=status,
                  required_topics=self.required,initial_topics=self.inventory,
                  excluded_regexes=self.excluded,assessment_start_ns=self.assessment,
                  recording_stop_ns=getattr(self,'recording_stop_ns',None),
                  navigate=self.args.navigate,target_reached=self.target_reached,
                  requested_distance_m=self.args.distance,requested_speed_m_s=self.args.speed,
                  distance_reached_m=getattr(self,'distance',None),error=self.failure,
                  timestamp_semantics='ROS1 bag receipt time; original source headers preserved',
                  hardware_synchronized=False,sampling='none',finished_ns=int(time.time()*1e9) if status!='preparing' else None)
        data.update(node_name=self.node_name,recorder_node=self.record_node)
        if status!='preparing':
            try:data['final_topics']=dict((n,t) for n,t in self.master.getPublishedTopics('') if not self.is_excluded(n))
            except Exception as error:data['inventory_error']=str(error)
        temporary=self.folder/'run.json.tmp';temporary.write_text(json.dumps(data,indent=2))
        temporary.rename(self.folder/'run.json')

    def reader(self):
        try:
            while True:
                # Python 2 file iteration can wait for a large read-ahead buffer
                # on a pipe. readline delivers each heartbeat immediately.
                line=sys.stdin.readline()
                if not line:break
                message=json.loads(line);command=message.get('command')
                if command=='heartbeat':self.last_heartbeat=monotonic()
                elif command=='clock_probe':self.emit('clock',probe_id=message['probe_id'],remote_ns=int(time.time()*1e9))
                else:self.commands.append(command)
        except Exception as error:self.stop_reason='Command channel error: '+str(error)
        finally:
            if not self.requested_finish:self.stop_reason='SSH command channel closed'

    def guard(self):
        # Independent local deadman: never retain a velocity through a stalled plan or SSH failure.
        while self.motion_running:
            now=monotonic();speed,yaw=self.motion_value
            if now-self.last_heartbeat>1. or now-self.nav_pose_time>.3 or now-self.plan_time>.2 or self.stop_reason:
                speed,yaw=0.,0.
            message=self.Twist();message.linear.x=speed;message.angular.z=yaw
            try:self.motion.publish(message)
            except Exception:self.stop_reason='Velocity publisher failed'
            time.sleep(.05)

    def request_navigation(self):
        if not self.args.navigate:raise RuntimeError('Movement was not enabled for this session')
        self.mode_requested=monotonic();self.plan_time=monotonic()
        self.motion_running=True
        threading.Thread(target=self.guard).start()
        def call():
            try:
                from go1_legged_msgs.srv import SetMode
                self.rospy.wait_for_service('/set_mode',timeout=5.)
                response=self.rospy.ServiceProxy('/set_mode',SetMode)(mode=2,gait_type=1)
                self.mode_response=bool(response.success)
            except Exception as error:self.mode_response=str(error)
        worker=threading.Thread(target=call);worker.daemon=True;worker.start()
        self.emit('walking_mode_requested')

    def navigate_tick(self):
        if self.mode_requested is None or self.target_reached:return
        self.plan_time=monotonic()
        if self.mode_response is None:
            if monotonic()-self.mode_requested>6.:raise RuntimeError('Walking-mode service timeout')
            return
        if self.mode_response is not True:raise RuntimeError('Walking-mode request failed: '+str(self.mode_response))
        if monotonic()-self.nav_pose_time>.3:raise RuntimeError('Navigation odometry stale')
        if self.nav_started is None:
            if monotonic()-self.mode_requested<1.:return
            self.nav_origin=self.nav_pose;self.nav_started=monotonic()
            self.emit('motion_started',target_m=self.args.distance,speed_m_s=self.args.speed)
        if monotonic()-self.nav_started>max(10.,3*self.args.distance/self.args.speed+10):
            raise RuntimeError('Navigation timeout')
        x,y,heading=self.nav_pose;x0,y0,h0=self.nav_origin
        previous=getattr(self,'previous_nav_pose',None)
        if previous is not None and math.hypot(x-previous[0],y-previous[1])>.25:
            raise RuntimeError('Odometry jump/reset')
        self.previous_nav_pose=(x,y)
        dx=x-x0;dy=y-y0;self.distance=math.hypot(dx,dy)
        lateral=-dx*math.sin(h0)+dy*math.cos(h0)
        error=math.atan2(math.sin(heading-h0),math.cos(heading-h0))
        if abs(lateral)>.25 or abs(error)>.5:raise RuntimeError('Deviation from straight path')
        if self.distance>self.args.distance+.25:raise RuntimeError('Navigation overshoot')
        remaining=self.args.distance-self.distance
        if remaining<=.02:
            self.motion_value=(0.,0.);self.target_reached=True
            self.emit('target_reached',distance_m=self.distance);return
        speed=max(.08,self.args.speed*remaining/.4) if remaining<.4 else self.args.speed
        self.motion_value=(min(self.args.speed,speed),max(-.15,min(.15,-.8*error-.3*lateral)))

    def health(self):
        now=monotonic()
        with self.stats_lock:
            return [name for name,row in self.rows.items()
                    if row['count']<2 or now-row['last']>.6 or not -.05<=row['age_s']<=.5]

    def run(self):
        started=monotonic();last_status=0.;last_record_check=0.;record_attached=False
        reader=threading.Thread(target=self.reader);reader.daemon=True;reader.start()
        log=(self.folder/'record.log').open('w')
        # Invoke C++ directly: the older Pi Python rosbag wrapper can exit on
        # SIGINT while leaving its child recorder alive and its .active file open.
        binary='/opt/ros/melodic/lib/rosbag/record'
        if not os.access(binary,os.X_OK):raise RuntimeError('Native rosbag record binary is unavailable')
        command=[binary,'--all','--buffsize=256','-O',str(self.folder/'capture.bag'),
                 '__name:='+self.record_node[1:]]
        if self.args.role==self.args.realsense_controller and self.args.include_realsense:command+=['--lz4']
        if self.excluded:command+=['--exclude','^('+'|'.join(self.excluded)+')$']
        try:
            free=os.statvfs(str(self.folder))
            if free.f_bavail*free.f_frsize<1536*1024*1024:
                raise RuntimeError('Less than 1.5 GiB free; archive recordings before starting')
            self.recorder=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,preexec_fn=os.setsid)
            (self.root/'pids'/('native_record_'+self.args.run_id)).write_text(str(self.recorder.pid))
            self.emit('recording_started',path=str(self.folder))
            while not self.rospy.is_shutdown():
                now=monotonic()
                if self.stop_reason:raise RuntimeError(self.stop_reason)
                if now-self.last_heartbeat>2.:raise RuntimeError('Laptop heartbeat expired')
                if now-started>self.args.max_seconds:raise RuntimeError('Session hard time limit exceeded')
                if self.recorder.poll() is not None:raise RuntimeError('Native recorder exited unexpectedly')
                disk=os.statvfs(str(self.folder))
                if disk.f_bavail*disk.f_frsize<1280*1024*1024:raise RuntimeError('Less than 1.25 GiB free; stopping above rosbag disk cutoff')
                missing=self.health()
                if not self.ready:
                    if now-last_record_check>1.:
                        last_record_check=now
                        subscribers=dict(self.master.getSystemState()[1])
                        node=self.record_node
                        record_attached=all(node in subscribers.get(name,[]) for name in self.required)
                    if not missing and record_attached and now-started>2.:
                        self.ready=True;self.emit('ready',required_topics=self.required)
                    elif now-started>self.args.startup_timeout:
                        raise RuntimeError('Readiness timeout; missing/stale '+str(missing)+'; recorder_attached='+str(record_attached))
                elif missing:raise RuntimeError('Required streams stale: '+str(missing))
                while self.commands:
                    action=self.commands.pop(0)
                    if action=='finish':self.requested_finish=True;break
                    if action=='assessment_start':self.assessment=int(time.time()*1e9);self.emit('assessment_started')
                    if action=='navigate':
                        if not self.ready:raise RuntimeError('Navigation requested before readiness')
                        if self.mode_requested is None:self.request_navigation()
                if self.requested_finish:break
                self.navigate_tick()
                if now-last_status>=1.:
                    last_status=now
                    self.emit('status',ready=self.ready,counts=dict((n,r['count']) for n,r in self.rows.items()),
                              distance_m=getattr(self,'distance',0.),free_bytes=disk.f_bavail*disk.f_frsize)
                time.sleep(.05)
        except BaseException as error:
            self.failure=str(error) or type(error).__name__;self.emit('error',message=self.failure)
        finally:
            self.stop_reason=self.stop_reason or 'Session ending';self.motion_value=(0.,0.)
            if self.motion_running:
                time.sleep(.55);self.motion_running=False;time.sleep(.1)
            if self.motion is not None:self.motion.unregister()
            self.recording_stop_ns=int(time.time()*1e9)
            if self.recorder is not None and self.recorder.poll() is None:
                os.killpg(self.recorder.pid,signal.SIGINT)
                deadline=monotonic()+20.
                while self.recorder.poll() is None and monotonic()<deadline:time.sleep(.1)
                if self.recorder.poll() is None:
                    self.recorder.kill();self.failure=self.failure or 'Native recorder did not finalize'
                elif self.recorder.returncode not in (0,-signal.SIGINT):
                    self.failure=self.failure or 'Recorder returned '+str(self.recorder.returncode)
            log.close()
            if self.args.navigate and not self.target_reached:self.failure=self.failure or 'Navigation target was not reached'
            if not self.requested_finish:self.failure=self.failure or 'No coordinated finish command'
            self.save_state('aborted' if self.failure else 'complete')
            self.emit('finalized',status='aborted' if self.failure else 'complete',error=self.failure)
            self.events.close()
        return 2 if self.failure else 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True);parser.add_argument('--role',required=True,choices=['pi','13','14','15'])
    parser.add_argument('--max-seconds',type=float,required=True)
    parser.add_argument('--startup-timeout',type=float,default=30.)
    parser.add_argument('--navigate',action='store_true');parser.add_argument('--distance',type=float,default=1.)
    parser.add_argument('--speed',type=float,default=.2)
    for name in ['include-lidar','include-realsense','skip-lidar','skip-realsense']:
        parser.add_argument('--'+name,action='store_true')
    parser.add_argument('--realsense-controller',default='14',choices=['pi','13','14','15'])
    parser.add_argument('--optional-camera',action='append',default=[])
    args=parser.parse_args()
    if not 0<args.max_seconds<=4000 or not 0<args.startup_timeout<=120:parser.error('Invalid time limits')
    if args.navigate and not (0<args.distance<=100 and 0<args.speed<=.5):parser.error('Navigation distance (0,100] m; speed (0,0.5] m/s')
    capture=None
    try:
        capture=Capture(args)
        def stop(signum,frame):capture.stop_reason='Received signal '+str(signum)
        signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
        return capture.run()
    except BaseException as error:
        print(json.dumps(dict(event='error',role=args.role,message=str(error))));sys.stdout.flush()
        return 2


if __name__=='__main__':sys.exit(main())
