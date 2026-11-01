#!/usr/bin/env python3
"""Offline ROS 2 SQLite bag insights; Python standard library only, no robot/ROS needed."""
import argparse
import bisect
import html
import json
import math
import sqlite3
import statistics
import struct
from pathlib import Path

HEADER_TYPES = {
    'sensor_msgs/msg/' + name for name in
    ['Image', 'CompressedImage', 'CameraInfo', 'Imu', 'MagneticField',
     'BatteryState', 'JointState', 'LaserScan', 'PointCloud2', 'Range',
     'Temperature', 'RelativeHumidity', 'FluidPressure', 'NavSatFix']
} | {'nav_msgs/msg/Odometry', 'geometry_msgs/msg/TwistStamped',
     'diagnostic_msgs/msg/DiagnosticArray', 'turtlebot3_msgs/msg/SensorState'}


def stamp_from_cdr(prefix):
    """Read leading ROS 2 Header stamp only; explicitly accept standard CDR v1."""
    if len(prefix) < 12 or prefix[:2] not in (b'\x00\x00', b'\x00\x01'):
        return None
    sec, nano = struct.unpack(('>' if prefix[1] == 0 else '<') + 'iI', prefix[4:12])
    if sec < 0 or nano >= 10**9 or (sec == 0 and nano == 0):
        return None
    return sec * 10**9 + nano


def percentile(values, percent):
    if not values:
        return None
    values = sorted(values)
    i = (len(values)-1)*percent/100
    lo, hi = math.floor(i), math.ceil(i)
    return values[lo] + (values[hi]-values[lo])*(i-lo)


def metrics(times):
    if len(times) < 2:
        return {'hz': None, 'median_gap_ms': None, 'max_gap_ms': None,
                'gaps_over_1_5x_median': 0, 'backwards': 0, 'duplicates': 0}
    differences = [b-a for a,b in zip(times,times[1:])]
    positive = [d for d in differences if d > 0]
    median = statistics.median(positive) if positive else None
    span = times[-1]-times[0]
    return {'hz': (len(times)-1)*1e9/span if span > 0 and min(differences) >= 0 else None,
            'median_gap_ms': median/1e6 if median else None,
            'max_gap_ms': max(positive)/1e6 if positive else None,
            'gaps_over_1_5x_median': sum(d > 1.5*median for d in positive) if median else 0,
            'backwards': sum(d < 0 for d in differences),
            'duplicates': sum(d == 0 for d in differences)}


def inspect(bag):
    files = [bag] if bag.is_file() and bag.suffix == '.db3' else list(bag.glob('*.db3'))
    if not files:
        raise ValueError('No SQLite .db3 bag found')
    def first_time(path):
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            return db.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0] or 0
    topics, checks = {}, {}
    for path in sorted(files, key=first_time):
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            checks[path.name] = [r[0] for r in db.execute('PRAGMA quick_check')]
            local = {}
            for ident, name, kind in db.execute('SELECT id,name,type FROM topics'):
                item = topics.setdefault(name, {'type':kind, 'receipts':[], 'stamps':[], 'ages':[], 'bytes':0, 'unreadable_headers':0})
                if item['type'] != kind:
                    raise ValueError('Conflicting types for '+name)
                local[ident] = item
            # Fetch only the 12-byte timestamp prefix, never full camera payloads.
            for ident, receipt, size, prefix in db.execute('SELECT topic_id,timestamp,length(data),substr(data,1,12) FROM messages ORDER BY id'):
                item = local[ident]
                item['receipts'].append(receipt)
                item['bytes'] += size
                if item['type'] in HEADER_TYPES:
                    stamp = stamp_from_cdr(prefix)
                    if stamp is None:
                        item['unreadable_headers'] += 1
                    else:
                        item['stamps'].append(stamp)
                        item['ages'].append((receipt-stamp)/1e6)
    all_receipts = [v for item in topics.values() for v in item['receipts']]
    if not all_receipts:
        raise ValueError('Bag has no messages')
    start, end = min(all_receipts), max(all_receipts)
    duration = (end-start)/1e9
    report = {'bag':str(bag), 'duration_s':duration,
              'messages':sum(len(s['receipts']) for s in topics.values()),
              'topic_count':len(topics), 'database_size_mb':sum(p.stat().st_size for p in files)/1e6,
              'sqlite_quick_check':checks, 'topics':{}, 'camera_alignment':[], 'warnings':[]}
    for name,item in sorted(topics.items()):
        report['topics'][name] = {
            'type':item['type'], 'messages':len(item['receipts']),
            'payload_mb':item['bytes']/1e6,
            'receipt':metrics(item['receipts']), 'source':metrics(item['stamps']),
            'source_stamp_count':len(item['stamps']), 'unreadable_headers':item['unreadable_headers'],
            'first_receipt_offset_s':(min(item['receipts'])-start)/1e9 if item['receipts'] else None,
            'last_receipt_offset_s':(max(item['receipts'])-start)/1e9 if item['receipts'] else None,
            'receipt_minus_source_ms':{'median':percentile(item['ages'],50), 'p95':percentile(item['ages'],95),
                                       'min':min(item['ages']) if item['ages'] else None,
                                       'max':max(item['ages']) if item['ages'] else None}}
        if item['unreadable_headers']:
            report['warnings'].append(f'{name}: {item["unreadable_headers"]} zero/invalid/unsupported source headers')
        if report['topics'][name]['source']['backwards']:
            report['warnings'].append(name+': source timestamp moved backwards')
    cameras = [(n,s) for n,s in topics.items() if s['type'] in ('sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage')]
    references = [(n,s) for n,s in topics.items() if s['type'] in ('sensor_msgs/msg/Imu','nav_msgs/msg/Odometry','sensor_msgs/msg/LaserScan')]
    for camera,c in cameras:
        for reference,r in references:
            targets = sorted(r['stamps'])
            if not targets:
                continue
            errors = []
            for stamp in c['stamps']:
                if not targets[0] <= stamp <= targets[-1]:
                    continue
                i = bisect.bisect_left(targets, stamp)
                near = targets[max(0,i-1):min(len(targets),i+1)]
                errors.append(min(abs(stamp-t) for t in near)/1e6)
            report['camera_alignment'].append({'camera':camera,'reference':reference,'paired_frames':len(errors),
                'outside_reference_window':len(c['stamps'])-len(errors),
                'nearest_stamp_difference_ms':{'median':percentile(errors,50),'p95':percentile(errors,95),'max':max(errors) if errors else None}})
    folder = bag.parent if bag.is_file() else bag
    experiment_path = folder/'experiment.json'
    experiment = json.loads(experiment_path.read_text()) if experiment_path.exists() else {}
    report['experiment'] = {k:experiment.get(k) for k in ['mode','status','arguments','missing_required_in_bag']}
    report['navigation_events'] = [e for e in experiment.get('events',[]) if e.get('event') in ['motion_started','target_reached','error']]
    report['missing_required_topics'] = [n for n in experiment.get('required_topics',[]) if n not in topics or not topics[n]['receipts']]
    if report['missing_required_topics']:
        report['warnings'].append('Missing required streams: '+', '.join(report['missing_required_topics']))
    audit_path = folder/'audit.json'
    if audit_path.exists():
        audit = json.loads(audit_path.read_text())
        # Explicitly label this as a separate saved audit, not a new full decode.
        report['saved_native_audit'] = {k:audit.get(k) for k in ['total_messages','storage_and_decode_ok','decode_failures','last_driver_command','odom_displacement_m']}
        report['saved_native_audit']['message_count_matches'] = audit.get('total_messages') == report['messages']
        report['camera_calibration_from_saved_audit'] = {n:s.get('calibrated') for n,s in audit.get('topics',{}).items() if s.get('type') == 'sensor_msgs/msg/CameraInfo'}
        for name,calibrated in report['camera_calibration_from_saved_audit'].items():
            if calibrated is False:
                report['warnings'].append(name+': saved audit reports no camera intrinsic calibration')
    report['limitations'] = [
        'Source stamps and bag receipt times are different clocks/events; receipt-minus-source includes clock offset and pipeline delay.',
        'Nearest timestamps measure sampling proximity, not simultaneous physical acquisition or hardware synchronization.',
        'Large gaps are indicators only; without publisher counters they cannot prove dropped messages.',
        'Only known Header-first CDR v1 message types have source timing parsed. Unstamped/unsupported types use receipt timing.',
        'This report checks SQLite and timestamp prefixes; it does not decode every sensor payload. Saved native audit results are labelled separately.',
        'Navigation events and camera calibration are supplemental saved metadata. No ground-truth distance or calibration is inferred.']
    return report


def fmt(value):
    return '—' if value is None else f'{value:.3f}' if isinstance(value, float) else str(value)


def markdown(report):
    lines = ['# TurtleBot bag insights', '', f'Bag: `{report["bag"]}`', '',
        f'Duration: **{report["duration_s"]:.2f} s** | Messages: **{report["messages"]}** | Topics: **{report["topic_count"]}** | Database: **{report["database_size_mb"]:.1f} MB**', '',
        '## Stream rates and timing', '',
        '| Topic | Messages | Source Hz/FPS | Receipt Hz | Source max gap ms | Delay median / p95 ms |',
        '|---|---:|---:|---:|---:|---:|']
    for name,s in report['topics'].items():
        delay=s['receipt_minus_source_ms']
        lines.append(f'| {name} | {s["messages"]} | {fmt(s["source"]["hz"])} | {fmt(s["receipt"]["hz"])} | {fmt(s["source"]["max_gap_ms"])} | {fmt(delay["median"])} / {fmt(delay["p95"])} |')
    lines += ['', '## Camera timestamp proximity', '', '| Camera | Reference | Paired frames | Median / p95 / maximum ms |', '|---|---|---:|---:|']
    for item in report['camera_alignment']:
        d=item['nearest_stamp_difference_ms']
        lines.append(f'| {item["camera"]} | {item["reference"]} | {item["paired_frames"]} | {fmt(d["median"])} / {fmt(d["p95"])} / {fmt(d["max"])} |')
    lines += ['', '**These timestamp differences do not prove physical synchronization.**', '', '## Navigation and saved audit', '', '```json', json.dumps({k:report.get(k) for k in ['experiment','navigation_events','saved_native_audit','camera_calibration_from_saved_audit']},indent=2), '```', '', '## Warnings', '']
    lines += ['- '+w for w in report['warnings']] or ['No timestamp/required-stream warnings detected.']
    lines += ['', '## Interpretation limits', '']+['- '+x for x in report['limitations']]
    return '\n'.join(lines)+'\n'


def html_report(report):
    escape=lambda value: html.escape(str(value))
    def table(headers, rows):
        return '<div class="scroll"><table><thead><tr>'+''.join('<th>'+escape(h)+'</th>' for h in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+escape(c)+'</td>' for c in row)+'</tr>' for row in rows)+'</tbody></table></div>'
    rates=[(n,s['source']['hz']) for n,s in report['topics'].items() if s['source']['hz'] is not None]
    maxrate=max([r for _,r in rates],default=1.) or 1.
    bars=''.join(f'<div class="barrow"><span>{escape(name)}</span><div class="bar" style="width:{rate/maxrate*100:.2f}%">{rate:.2f} Hz</div></div>' for name,rate in rates)
    stream_rows=[]
    for name,s in report['topics'].items():
        delay=s['receipt_minus_source_ms']
        stream_rows.append([name,s['messages'],fmt(s['source']['hz']),fmt(s['receipt']['hz']),fmt(s['source']['max_gap_ms']),fmt(delay['median']),fmt(delay['p95'])])
    pairs=[]
    for item in report['camera_alignment']:
        d=item['nearest_stamp_difference_ms']
        pairs.append([item['camera'],item['reference'],item['paired_frames'],fmt(d['median']),fmt(d['p95']),fmt(d['max']),item['outside_reference_window']])
    warning='<ul>'+''.join('<li>'+escape(w)+'</li>' for w in report['warnings'])+'</ul>' if report['warnings'] else '<p>No required-stream or timestamp warnings detected.</p>'
    details=json.dumps({k:report.get(k) for k in ['experiment','navigation_events','saved_native_audit','camera_calibration_from_saved_audit']},indent=2)
    return ('<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>TurtleBot bag insights</title><style>'
        'body{font:15px system-ui;margin:30px auto;max-width:1200px;padding:0 20px;color:#182238;background:#f5f7fa}'
        'section{background:white;padding:22px;margin:20px 0;border-radius:10px}h1{margin-bottom:8px}.path{overflow-wrap:anywhere;color:#536378}'
        'pre{white-space:pre-wrap;overflow-wrap:anywhere;line-height:1.6}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:10px;text-align:left;border-bottom:1px solid #dbe2eb}th{background:#eef3fa}'
        '.barrow{display:grid;grid-template-columns:minmax(180px,360px) 1fr;align-items:center;margin:8px 0;gap:12px}.barrow span{overflow-wrap:anywhere}.bar{background:#2866a8;color:white;min-width:70px;padding:5px;box-sizing:border-box}'
        '</style></head><body><h1>TurtleBot bag insights</h1><p class="path">'+escape(report['bag'])+'</p>'+
        f'<p><b>{report["duration_s"]:.2f} s</b> · <b>{report["messages"]} messages</b> · <b>{report["topic_count"]} topics</b> · <b>{report["database_size_mb"]:.1f} MB</b></p>'+
        '<section><h2>Warnings</h2>'+warning+'</section><section><h2>Source stream rates</h2>'+bars+'</section>'+
        '<section><h2>Streams and delivery timing</h2><p>Delay is bag receipt minus source stamp; it includes clock offset and pipeline delay.</p>'+table(['Topic','Messages','Source Hz/FPS','Receipt Hz','Max source gap ms','Median delay ms','p95 delay ms'],stream_rows)+'</section>'+
        '<section><h2>Camera timestamp proximity</h2><p>Nearest source timestamps within overlapping coverage. These differences do not prove physical synchronization.</p>'+table(['Camera','Reference','Paired frames','Median ms','p95 ms','Max ms','Outside coverage'],pairs)+'</section>'+
        '<section><h2>Navigation and saved audit</h2><pre>'+escape(details)+'</pre></section>'+
        '<section><h2>Interpretation limits</h2><ul>'+''.join('<li>'+escape(x)+'</li>' for x in report['limitations'])+'</ul></section></body></html>')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path,help='Downloaded bag folder or .db3 file')
    args=parser.parse_args()
    bag=args.bag.expanduser().resolve()
    try:
        report=inspect(bag)
    except (ValueError, sqlite3.Error, OSError) as error:
        parser.exit(1,str(error)+'\n')
    output=(bag.parent if bag.is_file() else bag)/'Insights'
    output.mkdir(exist_ok=True)
    (output/'insights.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (output/'insights.md').write_text(markdown(report))
    (output/'insights.html').write_text(html_report(report))
    print(f'{report["messages"]} messages | {report["topic_count"]} topics | {report["duration_s"]:.2f} seconds')
    print('Warnings:',len(report['warnings']))
    print('Open report:',output/'insights.html')


if __name__=='__main__':
    main()
