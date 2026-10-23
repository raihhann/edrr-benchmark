#!/usr/bin/env python3
"""MA_Raihan capture: rectified stereo only, stamped motor telemetry, all active built-ins.

Stationary by default. Add --navigate explicitly to run the existing navigation controller.
Custom ROS 1 messages are preserved as JSON in ROS 2 String messages, retaining their
source headers and using those headers for bag timestamps. No old recorder is modified.
"""
import importlib.util
import sys
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
    def configure_requirements(self,allow_no_video=False):
        super().configure_requirements(allow_no_video)
        expected={'/ma_raihan/robot/low_state','/ma_raihan/robot/motor_states',
                  '/ma_raihan/robot/imu','/ma_raihan/robot/battery','/ma_raihan/robot/joint_states_measured'}
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
                    raise RuntimeError('Robot packet delivery delay exceeds 500 ms: '+topic)
        return ready


def main():
    args=sys.argv[1:]
    navigate='--navigate' in args
    args=[a for a in args if a!='--navigate']
    rs='--include-realsense' in args;lidar='--include-lidar' in args
    args=[a for a in args if a not in ('--include-realsense','--include-lidar')]
    if not navigate and '--record-only' not in args:args.insert(0,'--record-only')
    v3.select_recording_topics=lambda topics,transport='auto':select_ma_topics(topics,transport,rs,lidar)
    v3.MultiSourceRecorder=Recorder
    sys.argv=[sys.argv[0]]+args
    v3.main()

if __name__=='__main__':main()
