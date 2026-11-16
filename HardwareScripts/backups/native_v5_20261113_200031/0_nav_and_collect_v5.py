#!/usr/bin/env python3
"""V4 navigation and recording, with explicit post-run quality decisions.

Navigation is opt-in: --navigate --distance 1 --speed 0.2.
Without --navigate this records stationary data only. Never automatically reruns.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

from run_quality import Thresholds, audit_bag, save_report


def load_v4():
    path=Path(__file__).with_name('nav_and_collect_v4.py')
    if not path.exists():
        path=Path('/home/fsti/ROS/Go1_Project/NavigationScripts/nav_and_collect_v4.py')
    spec=importlib.util.spec_from_file_location('v5_navigation_engine',path)
    engine=importlib.util.module_from_spec(spec);spec.loader.exec_module(engine)
    return engine


def cli(argv):
    parser=argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Live options forwarded to v4: --navigate --distance M --speed M/S; '
               '--record-only --duration S; --out DIR; --include-realsense; --include-lidar; '
               '--allow-no-video; --host IP; --sources IP,IP. Exit: 0 valid/warnings, 2 invalid, 3 unverified.')
    parser.add_argument('--audit-only',type=Path,metavar='BAG',help='Check an existing bag without contacting or moving the robot')
    parser.add_argument('--report-dir',type=Path,help='Override report destination (normally the bag itself)')
    parser.add_argument('--max-loss-percent',type=float,default=1.,help='Maximum missing ROS sequence percentage per stream (default 1)')
    parser.add_argument('--min-camera-fps',type=float,default=8.,help='Front camera minimum; other Go1 cameras require at least 15 FPS')
    parser.add_argument('--skip-payload-check',action='store_true',help='Timing-only analysis; verdict cannot be fully VALID')
    parser.add_argument('--navigation-timeout',type=float,help='Maximum navigation seconds; default max(10, 3*distance/speed+10)')
    parser.add_argument('--ma-only',action='store_true',help='Use the previous curated MA_Raihan selection instead of all discovered topics')
    opts,remaining=parser.parse_known_args(argv)
    if not math.isfinite(opts.max_loss_percent) or not 0<=opts.max_loss_percent<=100:
        parser.error('--max-loss-percent must be finite and in [0,100]')
    if not math.isfinite(opts.min_camera_fps) or opts.min_camera_fps<=0:
        parser.error('--min-camera-fps must be finite and positive')
    if opts.navigation_timeout is not None and (not math.isfinite(opts.navigation_timeout) or opts.navigation_timeout<=0):
        parser.error('--navigation-timeout must be finite and positive')
    if opts.audit_only and remaining:parser.error('Unknown audit options: '+' '.join(remaining))
    return opts,remaining


def main(argv=None):
    opts,args=cli(sys.argv[1:] if argv is None else argv)
    thresholds=Thresholds(max_loss_percent=opts.max_loss_percent,min_camera_fps=opts.min_camera_fps)
    if opts.audit_only:
        result=audit_bag(opts.audit_only,thresholds,decode=not opts.skip_payload_check)
        save_report(result,opts.report_dir or opts.audit_only)
        return exit_code(result)
    if '--navigate' in args and '--record-only' in args:
        raise SystemExit('Choose --navigate OR --record-only.')
    engine=load_v4()
    # Parse through the existing engine once to validate all live arguments before connecting.
    engine_args=[a for a in args if a not in ('--navigate','--include-realsense','--include-lidar')]
    old_argv=sys.argv
    sys.argv=[str(Path(__file__))]+engine_args
    try: parsed=engine.v3.parse_args()
    finally: sys.argv=old_argv
    navigate='--navigate' in args
    if navigate and (parsed.distance<=0 or parsed.speed<=0):
        raise SystemExit('Navigation requires positive --distance and --speed.')
    context={'schema_version':1,'navigate':navigate,'requested_distance_m':parsed.distance if navigate else None,
             'requested_speed_m_s':parsed.speed if navigate else None,'requested_duration_s':parsed.duration,
             'allow_no_video':parsed.allow_no_video,'include_realsense':'--include-realsense' in args,
             'include_lidar':'--include-lidar' in args,'target_reached':False,
             'started_unix_ns':time.time_ns(),'thresholds':vars(thresholds),
             'topic_selection':'ma_only' if opts.ma_only else 'all',
             'new_topics_at_finish':[],'topic_inventory':[]}
    navigation_timeout=(opts.navigation_timeout or max(10.,3*parsed.distance/parsed.speed+10.)) if navigate else None
    context['navigation_timeout_s']=navigation_timeout
    recorders=[]
    if not opts.ma_only:
        def select_all(topics,transport='auto',include_realsense=False,include_lidar=False):
            selected={}
            for name,kind in topics.items():
                if not include_realsense and name.startswith(('/camera/color','/camera/depth','/camera/aligned_depth','/camera/infra','/camera/gyro','/camera/accel','/camera/extrinsics')):
                    continue
                if not include_lidar and name.startswith(('/rslidar_points','/rslidar_packets')):
                    continue
                if '--skip-realsense' in args and name.startswith(('/camera/color','/camera/depth','/camera/infra','/camera/gyro','/camera/accel','/camera/extrinsics')):continue
                if '--skip-lidar' in args and name.startswith(('/rslidar_points','/rslidar_packets')):continue
                selected[name]=kind
            return selected,[]
        engine.select_ma_topics=select_all

    class Recorder(engine.Recorder):
        def __init__(self,*a,**kw):
            if not opts.ma_only:kw['skip_topics']=()
            super().__init__(*a,**kw);recorders.append(self)
            self.quality_context=dict(context)

        def open(self,*a,**kw):
            answer=super().open(*a,**kw)
            self.save_context()
            return answer

        def save_context(self):
            # rosbag2 creates its URI directory during open(); do not pre-create it.
            if not self.bag_path.is_dir():return
            p=self.bag_path/'run_context.json'
            temporary=p.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.quality_context,indent=2,allow_nan=False))
            temporary.replace(p)
            if self.quality_context['topic_inventory']:
                (self.bag_path/'topic_inventory.json').write_text(json.dumps(self.quality_context['topic_inventory'],indent=2))

        def discover(self):
            found=super().discover()
            self.quality_context['topic_inventory']=[
                dict(host=host,source_topic=name,ros1_type=kind,bag_topic=self.topic_map[(host,name)])
                for host,name,kind in found]
            return found

        def wait_until_ready(self,*a,**kw):
            answer=super().wait_until_ready(*a,**kw)
            self.quality_context['assessment_start_arrival_ns']=time.time_ns()
            self.save_context()
            return answer

        def close(self):
            try:
                # Detect topics that appeared after discovery rather than claiming all were captured.
                if not self.closed and not opts.ma_only and self.quality_context['topic_inventory']:
                    import roslibpy
                    selected={(v['host'],v['source_topic']) for v in self.quality_context['topic_inventory']}
                    for host,ros in self.ros_by_host.items():
                        try:
                            roster=roslibpy.Service(ros,'/rosapi/topics','rosapi/Topics').call(roslibpy.ServiceRequest({}),timeout=2)
                            names=select_all(dict(zip(roster['topics'],roster['types'])),
                                             include_realsense=context['include_realsense'],include_lidar=context['include_lidar'])[0]
                            self.quality_context['new_topics_at_finish'].extend(
                                dict(host=host,topic=name) for name in names if (host,name) not in selected)
                        except Exception as error:
                            self.quality_context.setdefault('inventory_errors',[]).append(host+': '+str(error))
                return super().close()
            finally:
                self.quality_context['finished_unix_ns']=time.time_ns()
                for item in self.quality_context['topic_inventory']:
                    item['messages']=self.topic_counts.get(item['bag_topic'],0)
                self.save_context()

    class Navigator(engine.Navigator):
        def __init__(self,*a,**kw):
            super().__init__(*a,**kw);self.origin=None;self.motion_started=None

        def pose(self):
            pose=super().pose()
            if not all(math.isfinite(v) for v in pose):raise RuntimeError('Invalid navigation pose; stopping.')
            if self.origin is None:
                self.origin=pose;self.motion_started=time.monotonic()
            if time.monotonic()-self.motion_started>navigation_timeout:
                raise RuntimeError('Navigation timeout exceeded; stopping before audit.')
            distance=math.hypot(pose[0]-self.origin[0],pose[1]-self.origin[1])
            ctx=recorders[-1].quality_context
            ctx['distance_reached_m']=distance
            ctx['target_reached']=distance>=parsed.distance or parsed.distance-distance<=.02
            return pose

    engine.Recorder=Recorder;engine.Navigator=Navigator
    failure=None
    sys.argv=[str(Path(__file__))]+args
    try:
        engine.main()
    except Exception as error:
        failure=error
        print('[ERROR] '+str(error),file=sys.stderr)
    finally:sys.argv=old_argv
    if not recorders:
        print('[QUALITY] INVALID — no bag was created; fix connection/configuration before collecting again.')
        return 2
    recorder=recorders[-1]
    result=audit_bag(recorder.bag_path,thresholds,recorder.quality_context,decode=not opts.skip_payload_check)
    if failure is not None:
        result['issues'].append(dict(level='fail',code='execution_error',message=str(failure),topic=None))
        from run_quality import finalize
        result=finalize(result)
    save_report(result,opts.report_dir or recorder.bag_path)
    return exit_code(result)


def exit_code(result):
    return 2 if result['verdict']=='INVALID' else 3 if result['verdict']=='UNVERIFIED' else 0


if __name__=='__main__':
    try:sys.exit(main())
    except (OSError,ValueError,KeyError) as error:
        print('[QUALITY] UNVERIFIED — audit could not finish: '+str(error),file=sys.stderr)
        sys.exit(3)
