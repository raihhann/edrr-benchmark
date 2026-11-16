"""Post-run checks for MA_Raihan bags. No robot or network actions."""
import bisect
from collections import defaultdict
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sqlite3
import statistics


ROBOT = '/ma_raihan/robot/'
CAMERAS = ('front', 'left', 'right', 'rear_down')
CORE = {ROBOT + s for s in ('imu', 'battery', 'joint_states_measured', 'motor_states', 'low_state')}


@dataclass
class Thresholds:
    max_loss_percent: float = 1.0
    robot_p95_ms: float = 200.0
    robot_max_ms: float = 500.0
    camera_p95_ms: float = 250.0
    camera_max_ms: float = 1000.0
    max_alignment_ms: float = 50.0
    min_camera_fps: float = 8.0
    min_window_s: float = 2.0


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    x = (len(v) - 1) * q
    lo = int(x)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (x - lo)


def camera_topic(name):
    return '/cameras/' in name and '/image_rect/compressed' in name


def profile(name, t):
    """Minimum rates reflect the verified reduced Pi profile, not sensor maxima."""
    if camera_topic(name):
        role = name.split('/cameras/', 1)[1].split('/')[0]
        return (t.min_camera_fps if role == 'front' else max(15., t.min_camera_fps), .5)
    rates = {'imu': 20., 'joint_states_measured': 20., 'battery': 20.,
             'motor_states': 6., 'odom': 10., 'low_state': 3., 'high_state': 3.,
             'raw_low_state': 3., 'raw_high_state': 3.}
    if name.startswith(ROBOT) and name[len(ROBOT):] in rates:
        return rates[name[len(ROBOT):]], .5
    if '/temperature/' in name:
        return .5, 2.5
    if '/depth/image_native/compressed' in name:
        return 3., 1.
    if name.endswith('/points_native'):
        return .5, 3.
    # Calibration, metadata and CameraInfo may be latched, not periodic.
    return None, None


def timing_metrics(rows, start, end):
    r = [d for d in rows if start <= d['arrival_ns'] <= end]
    out = {'messages_in_window': len(r), 'missing_sequences': 0, 'sequence_resets': 0,
           'duplicate_sequences': 0, 'timestamp_backwards': 0, 'timestamp_duplicates': 0}
    seqs = [d.get('seq') for d in r]
    for a, b in zip(seqs, seqs[1:]):
        if a is None or b is None:
            continue
        # Header.seq is uint32; an actual wrap is not a node reset.
        if a > 0xffff0000 and b < 0x10000:
            out['missing_sequences'] += max(0, (b - a) % 2**32 - 1)
        elif b > a:
            out['missing_sequences'] += b - a - 1
        elif b == a:
            out['duplicate_sequences'] += 1
        else:
            out['sequence_resets'] += 1
    seq_count = sum(s is not None for s in seqs)
    out['sequence_available'] = seq_count > 1
    out['loss_percent'] = (100. * out['missing_sequences'] /
                           (seq_count + out['missing_sequences'])) if seq_count > 1 else None
    stamps = [d['source_ns'] for d in r if d.get('source_ns')]
    out['missing_source_stamps'] = len(r) - len(stamps)
    out['timestamp_backwards'] = sum(b < a for a, b in zip(stamps, stamps[1:]))
    out['timestamp_duplicates'] = sum(b == a for a, b in zip(stamps, stamps[1:]))
    out['source_hz'] = ((len(stamps)-1)*1e9/(stamps[-1]-stamps[0])
                        if len(stamps)>1 and stamps[-1]>stamps[0] else None)
    arrivals = [d['arrival_ns'] for d in r]
    out['arrival_hz'] = ((len(arrivals)-1)*1e9/(arrivals[-1]-arrivals[0])
                         if len(arrivals)>1 and arrivals[-1]>arrivals[0] else None)
    # Include start/end boundaries so a stream dying near the end cannot pass.
    bounds = [start] + arrivals + [end]
    out['max_arrival_gap_s'] = max((b-a)/1e9 for a,b in zip(bounds,bounds[1:]))
    ages = [d['age_ms'] for d in r if d.get('age_ms') is not None]
    out.update(median_delay_ms=percentile(ages,.5), p95_delay_ms=percentile(ages,.95),
               max_delay_ms=max(ages) if ages else None,
               min_delay_ms=min(ages) if ages else None)
    return out


def inspect_payloads(connection, add, decode=True):
    result = {'checked': False, 'invalid_messages': 0, 'invalid_images': 0,
            'timestamp_mismatches': 0, 'command_messages': 0, 'last_command_zero': None,
            'last_command_timestamp_ns': None}
    if not decode:
        add('unverified', 'payload_check_skipped', 'Payload decoding was explicitly skipped.')
        return result, []
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import cv2
        import numpy as np
    except ImportError as error:
        add('unverified', 'payload_dependencies', 'Source ROS Jazzy setup to verify payloads: '+str(error))
        return result, []
    odom = []
    for tid, name, kind in connection.execute('SELECT id,name,type FROM topics'):
        try:
            cls = get_message(kind)
        except Exception as error:
            add('unverified', 'message_type', name+': '+str(error))
            continue
        motor_counts = set()
        for stamp, raw in connection.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY id', (tid,)):
            try:
                m = deserialize_message(raw, cls)
                source = None
                if hasattr(m, 'header'):
                    source = m.header.stamp.sec*10**9+m.header.stamp.nanosec
                elif kind == 'std_msgs/msg/String' and name.startswith(ROBOT):
                    data = json.loads(m.data)['message']
                    h = data['header']['stamp']; source = h['secs']*10**9+h['nsecs']
                    if name == ROBOT+'motor_states':
                        motor_counts.add(len(data['motor_state']))
                        for motor in data['motor_state']:
                            for field in ('q','dq','tauEst','temperature'):
                                if not isinstance(motor[field],(float,int)) or not math.isfinite(motor[field]):
                                    raise ValueError('Invalid motor '+field)
                if source and stamp != source:
                    result['timestamp_mismatches'] += 1
                if kind == 'sensor_msgs/msg/CompressedImage':
                    im = cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_UNCHANGED)
                    if im is None or im.size == 0:
                        result['invalid_images'] += 1
                if kind == 'sensor_msgs/msg/PointCloud2':
                    if len(m.data)!=m.row_step*m.height or m.row_step<m.width*m.point_step:
                        raise ValueError('Point cloud dimensions do not match data buffer')
                if kind == 'sensor_msgs/msg/Image':
                    if len(m.data)!=m.step*m.height or (m.width and not m.step):
                        raise ValueError('Image dimensions do not match data buffer')
                if name == ROBOT+'odom':
                    p=m.pose.pose.position
                    if not all(math.isfinite(v) for v in (p.x,p.y)):
                        raise ValueError('Non-finite odometry')
                    odom.append((stamp,p.x,p.y))
                if name == '/go1_controller/cmd_vel':
                    vals=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
                    result['command_messages'] += 1
                    result['last_command_zero'] = all(math.isfinite(v) and abs(v)<1e-9 for v in vals)
                    result['last_command_timestamp_ns'] = stamp
            except Exception as error:
                result['invalid_messages'] += 1
                if result['invalid_messages'] <= 3:
                    add('fail','payload_decode', name+': '+str(error))
        if motor_counts and motor_counts != {12}:
            add('fail', 'motor_count', 'Expected 12 motor entries, found '+str(sorted(motor_counts)))
    result['checked'] = True
    for key in ('invalid_messages','invalid_images','timestamp_mismatches'):
        if result[key]:
            add('fail',key,str(result[key])+' '+key.replace('_',' '))
    return result, odom


def audit_bag(bag, thresholds=None, context=None, decode=True):
    bag=Path(bag).resolve();t=thresholds or Thresholds();context=dict(context or {})
    issues=[]
    def add(level,code,message,topic=None):
        issues.append(dict(level=level,code=code,message=message,topic=topic))
    result={'schema_version':1,'bag':str(bag),'thresholds':asdict(t),'issues':issues,
            'topics':{},'loss_definition':'ROS Header.seq gaps at the recorder; not Ethernet packet loss or TCP retransmissions.',
            'synchronization':'Source timestamp consistency and nearest-IMU pairing; hardware synchronization is not proven.'}
    report_path=bag/'capture_report.json'
    if not report_path.exists():
        add('fail','missing_report','capture_report.json is missing; recording may not have finalized.')
        return finalize(result)
    try:
        report=json.loads(report_path.read_text())
    except (OSError,ValueError) as error:
        add('fail','unreadable_report','Cannot read capture report: '+str(error))
        return finalize(result)
    if not context and (bag/'run_context.json').exists():
        try:context=json.loads((bag/'run_context.json').read_text())
        except (OSError,ValueError) as error:
            add('fail','unreadable_context','Cannot verify navigation context: '+str(error))
    result['context']=context
    if context.get('new_topics_at_finish'):
        add('fail','late_topics','Topics appeared after initial discovery and were not recorded: '+str(context['new_topics_at_finish']))
    if context.get('inventory_errors'):
        add('unverified','inventory_check','Could not verify final topic inventory: '+str(context['inventory_errors']))
    result['capture_status']=report.get('status')
    result['messages_written']=report.get('messages_written',0)
    if report.get('status') not in ('complete','partial'):
        add('fail','capture_incomplete',str(report.get('status'))+': '+str(report.get('error')))
    for key in ('conversion_failures','local_queue_drops','unprocessed_queue_messages'):
        if report.get(key,0):add('fail',key,str(report[key])+' '+key.replace('_',' '))
    specs=report.get('topics',{})
    for name in report.get('topics_without_messages',[]):
        if name.rsplit('/',1)[-1] in ('rosout','rosout_agg','connected_clients','client_count'):
            result.setdefault('silent_event_topics',[]).append(name)
            continue
        # In record-only mode there is deliberately no command publisher.
        if name=='/go1_controller/cmd_vel' and not context.get('navigate',False):continue
        add('fail','empty_topic','No messages on '+name,name)
    required=set(CORE)
    if context.get('navigate'):required.add(ROBOT+'odom')
    if not context.get('allow_no_video',False):
        required.update('/ma_raihan/cameras/'+role+'/image_rect/compressed' for role in CAMERAS)
    for name in sorted(required-set(specs)):
        add('fail','required_topic_missing','Required topic was not selected: '+name,name)
    for flag,prefixes in [('include_realsense',('/camera/color','/camera/depth')),
                          ('include_lidar',('/rslidar_points','/rslidar_packets'))]:
        if context.get(flag) and not any(n.startswith(prefixes) for n in specs):
            add('fail','optional_sensor_missing',flag+' requested, but no matching topics were recorded.')
    rows=defaultdict(list)
    try:
        with (bag/'timing.jsonl').open() as f:
            for line in f:
                d=json.loads(line)
                if d.get('arrival_ns') is not None:rows[d['topic']].append(d)
    except (OSError,ValueError,KeyError) as error:
        add('fail','timing_log','Cannot verify timing log: '+str(error))
    live={n for n in specs if profile(n,t)[0] is not None}
    anchors=[n for n in live if n in CORE or camera_topic(n) or n==ROBOT+'odom']
    present=[rows[n] for n in anchors if rows[n]]
    if present:
        start=max(ds[0]['arrival_ns'] for ds in present)+int(1e9)
        start=max(start,context.get('assessment_start_arrival_ns',0))
        end=max(ds[-1]['arrival_ns'] for ds in present)
        result['assessment_window_s']=(end-start)/1e9
        if end-start<t.min_window_s*1e9:
            add('fail','short_window','Less than %.1f seconds of assessable live data.'%t.min_window_s)
        else:
            for name in sorted(live):
                minimum,max_gap=profile(name,t)
                m=timing_metrics(rows[name],start,end);result['topics'][name]=m
                if m['messages_in_window']<2:
                    add('fail','too_few_messages','Fewer than two samples in the assessment window.',name);continue
                if not m['sequence_available']:
                    add('warning','loss_unmeasurable','No usable Header.seq; frame/message loss cannot be counted exactly.',name)
                loss=m['loss_percent']
                if loss is not None and loss>t.max_loss_percent:
                    add('fail','sequence_loss','%.3f%% missing ROS sequence numbers exceeds %.3f%%.'%(loss,t.max_loss_percent),name)
                elif loss:
                    add('warning','sequence_loss','%.3f%% missing ROS sequence numbers.'%loss,name)
                if m['sequence_resets'] or m['duplicate_sequences']:
                    add('fail','sequence_discontinuity','Source sequence reset or duplicate detected.',name)
                if m['timestamp_backwards'] or m['timestamp_duplicates']:
                    add('fail','source_clock','Repeated or decreasing source timestamps.',name)
                if m['missing_source_stamps']:
                    add('fail','missing_stamp','Live samples have no source timestamps.',name)
                rate=m['source_hz']
                if rate is None or rate<minimum:
                    add('fail','low_rate','Source rate %s Hz is below %.1f Hz.'%(round(rate,2) if rate else rate,minimum),name)
                if m['max_arrival_gap_s']>max_gap:
                    add('fail','stream_gap','Longest receipt gap %.3f s exceeds %.3f s.'%(m['max_arrival_gap_s'],max_gap),name)
                image=name.startswith('/ma_raihan/cameras/') or '/depth/' in name
                max_ms=t.camera_max_ms if image else t.robot_max_ms
                p95_ms=t.camera_p95_ms if image else t.robot_p95_ms
                # Board and joint thermal topics are low-rate, but still stamped.
                if m['max_delay_ms'] is None:
                    add('fail','delay_unavailable','No timestamp-based delay measurement.',name)
                elif m['max_delay_ms']>max_ms or m['p95_delay_ms']>p95_ms:
                    add('fail','delivery_delay','Delay p95/max %.1f/%.1f ms exceeds %.1f/%.1f ms.'%(m['p95_delay_ms'],m['max_delay_ms'],p95_ms,max_ms),name)
                if m['min_delay_ms'] is not None and m['min_delay_ms'] < -50.:
                    add('fail','clock_offset','Source timestamp is more than 50 ms in the future.',name)
                if camera_topic(name) and rate is not None and rate<15:
                    add('warning','camera_performance','Camera meets current minimum but delivers less than 15 FPS.',name)
            imu=sorted(d['source_ns'] for d in rows[ROBOT+'imu'] if d.get('source_ns') and start<=d['arrival_ns']<=end)
            for name in sorted(n for n in live if camera_topic(n)):
                offsets=[]
                for d in rows[name]:
                    stamp=d.get('source_ns')
                    if not stamp or not imu or not (start<=d['arrival_ns']<=end and imu[0]<=stamp<=imu[-1]):continue
                    i=bisect.bisect_left(imu,stamp)
                    offsets.append(min(abs(stamp-v) for v in imu[max(0,i-1):i+1])/1e6)
                if offsets:
                    result['topics'][name]['nearest_imu_ms']={'p95':percentile(offsets,.95),'max':max(offsets)}
                    if max(offsets)>t.max_alignment_ms:
                        add('fail','camera_imu_alignment','Nearest IMU timestamp exceeds %.1f ms.'%t.max_alignment_ms,name)
                else:add('fail','camera_imu_overlap','No camera/IMU timestamp overlap.',name)
    else:add('fail','no_live_data','No assessable camera or robot data.')
    dbs=sorted(bag.glob('*.db3'));total=0;odom=[];payload=[]
    if not dbs:add('fail','missing_database','No SQLite bag database found.')
    for db in dbs:
        try:
            with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as connection:
                integrity=connection.execute('PRAGMA quick_check').fetchone()[0]
                if integrity!='ok':add('fail','database_integrity',str(integrity))
                total+=connection.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
                decoded,poses=inspect_payloads(connection,add,decode);payload.append(decoded);odom.extend(poses)
        except sqlite3.Error as error:add('fail','database_error',str(error))
    result['database_messages']=total;result['payload_checks']=payload
    if total!=report.get('messages_written'):
        add('fail','database_count','Database count differs from recorder count.')
    timing_count=sum(len(v) for v in rows.values())
    if timing_count!=total:add('fail','timing_count','Timing log count differs from database count.')
    if context.get('navigate'):
        if not context.get('target_reached',False):add('fail','target_not_reached','Navigation did not reach its requested target.')
        actual=context.get('distance_reached_m');target=context.get('requested_distance_m')
        if actual is not None and target is not None:
            tolerance=max(.05,.05*target)
            if abs(actual-target)>tolerance:add('fail','distance_error','Distance error exceeds %.3f m.'%tolerance)
        if payload and all(p.get('checked') for p in payload) and not any(p.get('command_messages',0) for p in payload):
            add('fail','no_commands','Navigation has no recorded velocity commands.')
    command_chunks=[p for p in payload if p.get('command_messages')]
    if command_chunks and not max(command_chunks,key=lambda p:p.get('last_command_timestamp_ns') or 0)['last_command_zero']:
        add('fail','nonzero_final_command','Last recorded velocity command is not zero.')
    if odom:
        odom.sort();a,b=odom[0],odom[-1]
        result['odometry_displacement_m']=math.hypot(b[1]-a[1],b[2]-a[2])
    if not context:
        add('warning','legacy_run','No v5 run context: requested distance and target completion cannot be confirmed.')
    return finalize(result)


def finalize(result):
    levels={v['level'] for v in result['issues']}
    result['verdict']=('INVALID' if 'fail' in levels else 'UNVERIFIED' if 'unverified' in levels
                       else 'VALID_WITH_WARNINGS' if 'warning' in levels else 'VALID')
    result['rerun_recommended']=result['verdict']=='INVALID'
    result['action']=('Fix the reported issues, then repeat collection.' if result['rerun_recommended'] else
                      'Complete verification before using this run.' if result['verdict']=='UNVERIFIED' else
                      'Keep this run; review warnings against your experiment requirements.' if levels else 'Keep this run.')
    return result


def save_report(result, directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    (directory/'quality_report.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    lines=['# Run quality report','', '**'+result['verdict']+'** — '+result['action'], '',
           '- Bag: `'+result['bag']+'`','- Recorded messages: '+str(result.get('database_messages',0)),
           '- Rerun recommended: '+str(result['rerun_recommended']), '',
           'ROS message sequence gaps are not Ethernet packet-loss measurements. TCP can retransmit lost network packets.',
           'Timestamp agreement does not prove hardware synchronization.', '', '## Findings','']
    lines += ['- **'+i['level'].upper()+'** '+(i['topic']+': ' if i.get('topic') else '')+i['message'] for i in result['issues']]
    if not result['issues']:lines.append('- All configured checks passed.')
    lines += ['', '## Rates and losses','', '| Topic | Hz | Missing ROS messages | Loss % | p95 delay ms | Max gap s |',
              '|---|---:|---:|---:|---:|---:|']
    def fmt(v):return 'unknown' if v is None else '%.3f'%v
    for name,v in result['topics'].items():
        lines.append('| '+name+' | '+fmt(v['source_hz'])+' | '+str(v['missing_sequences'])+' | '+fmt(v['loss_percent'])+' | '+fmt(v['p95_delay_ms'])+' | '+fmt(v['max_arrival_gap_s'])+' |')
    lines += ['', '## Thresholds','', '```json',json.dumps(result['thresholds'],indent=2),'```','']
    (directory/'quality_report.md').write_text('\n'.join(lines))
    print('\n[QUALITY] '+result['verdict']+' — '+result['action'])
    for issue in result['issues']:
        print('  '+issue['level'].upper()+': '+(issue.get('topic') or '')+' '+issue['message'])
    print('[QUALITY] Reports: '+str(directory/'quality_report.md')+' and quality_report.json')
