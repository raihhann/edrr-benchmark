#!/usr/bin/env python3
"""MA_Raihan capture: rectified stereo only, stamped motor telemetry, all active built-ins.

Stationary by default. Add --navigate explicitly to run the existing navigation controller.
Custom ROS 1 messages are preserved as JSON in ROS 2 String messages, retaining their
source headers and using those headers for bag timestamps. No old recorder is modified.
"""
import importlib.util
import sys
import threading
from pathlib import Path

base=Path(__file__).with_name('nav_and_collect_v3.py')
if not base.exists():base=Path('/home/fsti/ROS/Go1_Project/NavigationScripts/nav_and_collect_v3.py')
spec=importlib.util.spec_from_file_location('ma_v3_engine',base)
v3=importlib.util.module_from_spec(spec);spec.loader.exec_module(v3)


def select_ma_topics(topics, transport='auto', include_realsense=False, include_lidar=False):
    selected={}
    for name,kind in topics.items():
        if name.startswith('/ma_raihan/'):
            if kind in ('sensor_msgs/Image','sensor_msgs/CompressedImage'):
                if '/image_rect/compressed' not in name and '/depth/image_native' not in name:
                    continue
            selected[name]=kind
        elif name=='/go1_controller/cmd_vel':selected[name]=kind
        elif include_realsense and name.startswith(('/camera/color','/camera/depth','/camera/aligned_depth','/camera/infra','/camera/gyro','/camera/accel','/camera/extrinsics')):
            selected[name]=kind
        elif include_lidar and name.startswith(('/rslidar_points','/rslidar_packets')):
            selected[name]=kind
    # Only optional RealSense transport selection uses the legacy image filter.
    return legacy_select(selected,transport) if include_realsense else (selected,[])


legacy_select=v3.select_recording_topics

OriginalRecorder=v3.MultiSourceRecorder
class Recorder(OriginalRecorder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        v3._ma_recorder = self
        self.latest_navigation_odom = None
        self._writer_stop = threading.Event()
        self._writer_thread = None
        self._writer_error = None

    def _callback(self, topic_name):
        record = super()._callback(topic_name)
        def receive(data):
            if topic_name == '/ma_raihan/robot/odom':
                self.latest_navigation_odom = data
            record(data)
        return receive

    def open(self, discovered):
        super().open(discovered)
        self._writer_thread = threading.Thread(target=self._write_loop,
                                               name="ma-bag-writer", daemon=True)
        self._writer_thread.start()

    def _write_loop(self):
        try:
            while not self._writer_stop.is_set():
                super().drain()
                self._writer_stop.wait(.001)
        except Exception as error:
            self._writer_error = error

    def drain(self):
        if self._writer_error is not None:
            raise RuntimeError("Background recording failed: " + str(self._writer_error)) from self._writer_error
        if self._writer_thread is None:
            return super().drain()

    def close(self):
        self._writer_stop.set()
        if self._writer_thread is not None:
            self._writer_thread.join()
            self._writer_thread = None
        if self._writer_error is not None:
            self.status, self.error = 'aborted', str(self._writer_error)
        return super().close()

    def configure_requirements(self,allow_no_video=False):
        super().configure_requirements(allow_no_video)
        expected={'/ma_raihan/robot/low_state','/ma_raihan/robot/motor_states',
                  '/ma_raihan/robot/imu','/ma_raihan/robot/battery','/ma_raihan/robot/joint_states_measured'}
        if getattr(v3, '_ma_navigate', False):
            expected.add('/ma_raihan/robot/odom')
        selected=set(self.specs)
        absent=expected-selected
        if absent:raise RuntimeError('Missing required stamped robot streams: '+str(sorted(absent)))
        self.required_live_topics.update(expected)

    def check_health(self, require_ready=True):
        ready=super().check_health(require_ready)
        for topic in self.required_live_topics:
            if topic.startswith('/ma_raihan/robot/'):
                ages=self.topic_timing.get(topic,{}).get('age_ms',[])
                if ages and abs(ages[-1])>500:
                    # Subscription setup can deliver queued samples. Wait for a
                    # fresh stable interval before motion; keep runtime checks strict.
                    if not require_ready:
                        return False
                    raise RuntimeError('Robot packet delivery delay exceeds 500 ms: '+topic)
        return ready



class Navigator(v3.Go1Navigator):
    @property
    def latest_odom(self):
        recorder = getattr(v3, '_ma_recorder', None)
        return recorder.latest_navigation_odom if recorder is not None else None

    @latest_odom.setter
    def latest_odom(self, value):
        # The recorder owns the single subscription and its latest sample.
        pass

    def connect(self):
        import roslibpy
        if not self.ros.is_connected:
            raise RuntimeError('Navigation rosbridge is disconnected')
        if '/ma_raihan/robot/odom' not in v3._ma_recorder.specs:
            raise RuntimeError('Stamped navigation odometry missing; start Pi with --control')
        self.command = roslibpy.Topic(self.ros, '/go1_controller/cmd_vel', 'geometry_msgs/Twist')
        self.odom_sub = None

    def set_walking_mode(self):
        super().set_walking_mode()
        print('[INFO] Checking fresh telemetry after walking-mode request; no velocity sent yet.', flush=True)
        v3._ma_recorder.wait_until_ready(15.)


def main():
    args=sys.argv[1:]
    navigate='--navigate' in args
    v3._ma_navigate = navigate
    args=[a for a in args if a!='--navigate']
    rs='--include-realsense' in args;lidar='--include-lidar' in args
    args=[a for a in args if a not in ('--include-realsense','--include-lidar')]
    if not navigate and '--record-only' not in args:args.insert(0,'--record-only')
    v3.select_recording_topics=lambda topics,transport='auto':select_ma_topics(topics,transport,rs,lidar)
    v3.MultiSourceRecorder=Recorder
    v3.Go1Navigator=Navigator
    sys.argv=[sys.argv[0]]+args
    v3.main()

if __name__=='__main__':main()
