"""Quality decisions for a downloaded multi-controller native ROS1 dataset."""
import bisect
import hashlib
import json
from pathlib import Path


def assess(folder, *, min_camera_fps=15., max_loss_percent=1.):
    folder=Path(folder);context=json.loads((folder/'experiment.json').read_text())
    issues=[];summaries={};audits={}
    def add(level,message,role=None,topic=None):issues.append(dict(level=level,message=message,role=role,topic=topic))
    if context.get('include_realsense'):add('warning','RealSense driver/host timestamp alignment is assessed; hardware exposure synchronization and camera IMU calibration are not verified')
    if context.get('error'):add('fail',context['error'])
    for camera in context.get('optional_cameras',[]):add('warning','Operator explicitly made camera optional: '+camera)
    if not context.get('coordinated_finish'):add('fail','Run did not finish normally on every controller')
    for role in ['pi','13','14','15']:
        directory=folder/role
        try:
            report=json.loads((directory/'audit.json').read_text());audits[role]=report
            transfer=json.loads((directory/'download_verified.json').read_text())
            if not transfer.get('verified'):add('unverified','Download checksums not verified',role)
            for name,expected in transfer.get('files',{}).items():
                path=directory/name
                if Path(name).name!=name or path.is_symlink():raise ValueError('Unsafe manifest entry')
                with path.open('rb') as handle:digest=hashlib.file_digest(handle,'sha256').hexdigest()
                if digest!=expected['sha256'] or path.stat().st_size!=expected['bytes']:
                    add('fail','Downloaded file checksum mismatch: '+name,role)
        except (OSError,ValueError) as error:
            add('unverified','Controller audit/download missing: '+str(error),role);continue
        if not report.get('decoded_all'):add('fail','Native bag could not be decoded: '+str(report.get('error')),role);continue
        if report['state']['status']!='complete':add('fail','Controller run aborted: '+str(report['state'].get('error')),role)
        if not report['state'].get('assessment_start_ns'):add('fail','No stable assessment window',role)
        elif (report['state'].get('recording_stop_ns',0)-report['state']['assessment_start_ns'])/1e9<2:
            add('fail','Stable assessment window shorter than two seconds',role)
        if report['decode_failures']:add('fail','Payload/image decoding failures',role)
        if report['missing_sensor_topics']:add('fail','Advertised sensors without messages: '+str(report['missing_sensor_topics']),role)
        if report['new_topics']:add('fail','Topics appeared after initial discovery: '+str(report['new_topics']),role)
        if report['state'].get('inventory_error'):add('unverified','Final topic inventory failed',role)
        for topic in report['state']['required_topics']:
            if topic not in report['topics']:add('fail','Required topic missing from native bag',role,topic)
        if role=='pi' and report.get('motor_counts')!=[12]:add('fail','Expected all 12 measured motors',role)
        if role=='pi' and context['navigate']:
            state=report['state']
            if not state.get('target_reached'):add('fail','Navigation target not reached',role)
            if report.get('final_command_zero') is not True:add('fail','Final command is absent or nonzero',role)
            reached=state.get('distance_reached_m')
            if reached is None or abs(reached-context['distance_m'])>max(.05,.05*context['distance_m']):add('fail','Navigation distance outside tolerance',role)
        summaries[role]=dict(messages=report['message_count'],topics=len(report['topics']),duration_s=report['duration_s'])
        for topic,stats in report['topics'].items():
            # rosout_agg forwards headers from several original publishers;
            # its mixed counters are not a sensor-loss counter.
            if stats.get('type')=='rosgraph_msgs/Log':continue
            loss=stats['sequence_loss_percent']
            if loss is not None and loss>max_loss_percent:add('fail','ROS sequence loss exceeds threshold',role,topic)
            elif stats['sequence_missing']:add('warning','Some ROS sequence gaps',role,topic)
            if stats['sequence_resets'] or stats['duplicate_sequences'] or stats['stamp_backwards'] or stats['stamp_duplicates']:
                add('fail','Source sequence/timestamp reset, duplicate or reversal',role,topic)
            camera='/image_rect/compressed' in topic or topic=='/camera/color/image_raw'
            core=topic in report['state']['required_topics']
            legacy=topic in ['/motor_states','/joint_states','/battery_state','/go1_controller/state','/go1_controller/imu/data','/go1_controller/odom']
            if not (core or camera or legacy):continue
            minimum=40. if legacy else (min_camera_fps if camera else (6. if topic.endswith('/motor_states') else 3. if topic.endswith('/low_state') else 10. if topic in ['/camera/depth/image_rect_raw','/camera/depth/camera_info'] else 40. if topic in ['/camera/gyro/sample','/camera/accel/sample'] else 5. if topic=='/rslidar_points' else 10. if topic.endswith('/odom') else 20.))
            rate=stats['source_hz'] or stats['receipt_hz'] or 0.
            if rate<minimum:add('fail','Rate %.2f Hz below %.2f Hz minimum'%(rate,minimum),role,topic)
            elif camera and rate<20:add('warning','Camera below desired 20 FPS',role,topic)
            if stats['max_gap_ms']>600:add('fail','Required stream gap exceeds 600 ms',role,topic)
            if stats['p95_age_ms'] is None:add('warning' if legacy else 'unverified','No valid source timestamps; only recorder arrival can be assessed',role,topic)
            elif stats['p95_age_ms']>250 or stats['max_age_ms']>1000 or stats['min_age_ms'] < -50:
                add('fail','Source/recorder clock or delivery delay outside limits',role,topic)
    imu=audits.get('pi',{}).get('topics',{}).get('/ma_raihan/robot/imu',{}).get('source_stamps',[])
    imu=sorted(imu);alignment={}
    for role,report in audits.items():
        for topic,stats in report.get('topics',{}).items():
            if '/image_rect/compressed' not in topic and topic!='/camera/color/image_raw':continue
            offsets=[]
            for stamp in stats.get('source_stamps',[]):
                if not imu or stamp<imu[0] or stamp>imu[-1]:continue
                i=bisect.bisect_left(imu,stamp)
                offsets.append(min(abs(stamp-v) for v in imu[max(0,i-1):i+1])/1e6)
            alignment[topic]=dict(max_nearest_imu_ms=max(offsets) if offsets else None,paired_frames=len(offsets))
            if not offsets:add('fail','No camera/IMU timestamp overlap',role,topic)
            elif max(offsets)>50:add('fail','Camera/IMU nearest timestamp exceeds 50 ms',role,topic)
    clocks=context.get('clock_probes',{});pi=clocks.get('pi')
    clock_differences={}
    for role in ['13','14','15']:
        probe=clocks.get(role)
        if not pi or not probe:add('unverified','Clock probe missing',role);continue
        difference=probe['offset_ns']-pi['offset_ns'];uncertainty=(probe['rtt_ns']+pi['rtt_ns'])/2
        clock_differences[role]=dict(relative_to_pi_ms=difference/1e6,uncertainty_ms=uncertainty/1e6)
        if abs(difference)-uncertainty>50e6:add('fail','Controller clock differs from Pi by more than 50 ms',role)
        elif uncertainty>50e6:add('unverified','SSH clock estimate is too uncertain',role)
    levels={i['level'] for i in issues}
    verdict='INVALID' if 'fail' in levels else 'UNVERIFIED' if 'unverified' in levels else 'VALID_WITH_WARNINGS' if issues else 'VALID'
    stream_metrics={role+':'+topic:{k:value for k,value in stats.items() if k!='source_stamps'}
                    for role,report in audits.items() for topic,stats in report.get('topics',{}).items()}
    return dict(schema_version=1,verdict=verdict,rerun_recommended=verdict=='INVALID',issues=issues,streams=stream_metrics,
                controllers=summaries,camera_imu_alignment=alignment,clock_differences=clock_differences,
                thresholds=dict(min_camera_fps=min_camera_fps,max_ros_sequence_loss_percent=max_loss_percent,
                                max_nearest_imu_ms=50,source_age_p95_ms=250,source_age_max_ms=1000,max_required_gap_ms=600),
                limitation='ROS header gaps are not Ethernet packet loss. Pi packet receipt and SDK camera stamps do not establish shared hardware triggering. Native ROS1 bags retain original serialized messages and headers.')


def save(result,folder):
    folder=Path(folder)
    (folder/'quality_report.json').write_text(json.dumps(result,indent=2))
    lines=['# Native Go1 run quality','', '**'+result['verdict']+'**','',
           'Rerun recommended: '+str(result['rerun_recommended']),'',result['limitation'],'',
           '| Controller | Messages | Topics | Duration s |','|---|---:|---:|---:|']
    for role,data in result['controllers'].items():lines.append('| %s | %d | %d | %.2f |'%(role,data['messages'],data['topics'],data['duration_s']))
    lines+=['','## Findings','']
    for issue in result['issues']:lines.append('- **'+issue['level'].upper()+'** '+str(issue.get('role') or '')+' '+str(issue.get('topic') or '')+': '+issue['message'])
    if not result['issues']:lines.append('- All configured checks passed.')
    lines+=['','## Recorded stream rates and gaps','',
            '| Controller:topic | Messages | Hz | Missing sequences | p95 age ms | Max gap ms |',
            '|---|---:|---:|---:|---:|---:|']
    def number(value):return 'unavailable' if value is None else '%.2f'%value
    for name,data in result.get('streams',{}).items():
        loss=str(data['sequence_missing']) if data['header_messages']>1 else 'unavailable'
        lines.append('| %s | %d | %s | %s | %s | %s |'%(name,data['messages'],number(data['source_hz'] or data['receipt_hz']),loss,
                      number(data['p95_age_ms']),number(data['max_gap_ms'])))
    (folder/'quality_report.md').write_text('\n'.join(lines)+'\n')
