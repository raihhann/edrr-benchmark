#!/usr/bin/env python3
"""Stationary compressed-camera/telemetry capture, with source-time diagnostics.

Never publishes velocity or changes a robot mode. NTP offset is measured against
the Pi; source header time is used for stamped messages, corrected receipt time
for messages without a header. Arrival and source times remain in timing.jsonl.
"""
import argparse
import array
import base64
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import queue
import socket
import statistics
import struct
import time

CORE = ('/go1_controller/', '/joint_states', '/motor_states', '/battery_state', '/tf')


def select_topics(topics):
    result = {}
    for name, kind in topics.items():
        if kind == 'sensor_msgs/CompressedImage' and name.endswith('/compressed'):
            result[name] = kind
        elif kind == 'sensor_msgs/CameraInfo' or name.startswith(CORE):
            if name != '/go1_controller/cmd_vel': result[name] = kind
    return result


def filter_navigation_topics(topics):
    """Keep sensors, but never send raw/duplicate color images over rosbridge."""
    selected = {}
    for name, kind in topics.items():
        if any(part in name for part in ('/theora','/compressedDepth','/parameter_descriptions','/parameter_updates')):
            continue
        if kind == 'sensor_msgs/Image':
            if '/depth/' in name or 'depth_' in name:
                selected[name] = kind
            elif topics.get(name+'/compressed') != 'sensor_msgs/CompressedImage':
                raise RuntimeError('Camera has no compressed stream: '+name)
            continue
        selected[name] = kind
    return selected


def source_stamp_ns(data):
    stamp = data.get('header', {}).get('stamp', {})
    return int(stamp.get('secs', 0)) * 10**9 + int(stamp.get('nsecs', 0))


def compressed_message(data):
    from sensor_msgs.msg import CompressedImage
    msg=CompressedImage(); source=source_stamp_ns(data)
    msg.header.stamp.sec=source//10**9; msg.header.stamp.nanosec=source%10**9
    msg.header.frame_id=data.get('header',{}).get('frame_id',''); msg.format=data['format']
    raw=base64.b64decode(data['data'],validate=True) if isinstance(data['data'],str) else bytes(data['data'])
    msg.data=array.array('B',raw)
    return msg


def ntp_offset(host, samples=3):
    results = []
    epoch = 2208988800
    for _ in range(samples):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2)
            packet = bytearray(48); packet[0] = 0x23
            start = time.time(); seconds = start + epoch
            struct.pack_into('!II', packet, 40, int(seconds), int((seconds % 1) * 2**32))
            sock.sendto(packet, (host, 123)); response, _ = sock.recvfrom(512)
            end = time.time()
            if len(response) < 48 or response[0] >> 6 == 3 or not 0 < response[1] < 16:
                # ntpd can rate-limit probes with a stratum-zero kiss-of-death packet.
                time.sleep(2.1)
                continue
            if response[24:32] != packet[40:48]: raise RuntimeError('NTP origin timestamp mismatch')
            def timestamp(offset):
                sec, frac = struct.unpack_from('!II', response, offset)
                return sec - epoch + frac / 2**32
            received, sent = timestamp(32), timestamp(40)
            results.append((end-start-(sent-received), ((received-start)+(sent-end))/2, response[1]))
        time.sleep(2.1)
    if not results: raise RuntimeError('No valid synchronized NTP reply from the Pi')
    delay, offset, stratum = min(results)
    return {'server': host, 'offset_s': offset, 'round_trip_s': delay, 'stratum': stratum,
            'note': 'Offset is Pi minus laptop; orphan/local reference is not verified absolute UTC.'}


def normalize(value):
    if isinstance(value, list): return [normalize(v) for v in value]
    if not isinstance(value, dict): return value
    if set(value) == {'secs', 'nsecs'}: return {'sec': value['secs'], 'nanosec': value['nsecs']}
    return {k.lower() if k in ('D','K','R','P') else k: normalize(v)
            for k,v in value.items() if k != 'seq'}


def restore_nulls(values, message):
    from rosidl_runtime_py.convert import get_message_slot_types
    from rosidl_runtime_py.import_message import import_message_from_namespaced_type
    from rosidl_parser.definition import AbstractNestedType, BasicType, NamespacedType
    def restore(value, slot):
        if isinstance(slot, BasicType):
            if value is None and slot.typename in ('float','double','float32','float64','long double'):
                return float('nan')
            return value
        if isinstance(slot, AbstractNestedType): return [restore(v,slot.value_type) for v in value]
        if isinstance(slot, NamespacedType): return restore_nulls(value,import_message_from_namespaced_type(slot)())
        return value
    slots=get_message_slot_types(message)
    return {key:restore(value,slots[key]) for key,value in values.items()}


def main(default_hosts=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', action='append', help='Repeat for multiple sources; default: Pi and all Jetsons')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--output', type=Path, default=Path('/home/fsti/ROS/Go1_Project/Bags/go1_bags'))
    parser.add_argument('--bag-name')
    parser.add_argument('--clock-source', default='192.168.123.161')
    parser.add_argument('--measure-only', action='store_true')
    parser.add_argument('--queue-size', type=int, default=1000)
    args = parser.parse_args()
    if not 0 < args.duration <= 3600: parser.error('duration must be in (0,3600] seconds')
    if args.queue_size <= 0: parser.error('queue size must be positive')
    if args.bag_name and (Path(args.bag_name).name != args.bag_name or args.bag_name in ('.','..')):
        parser.error('bag-name must be a single folder name')
    import roslibpy
    import rclpy
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.set_message import set_message_fields
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    clock = ntp_offset(args.clock_source)
    print('Clock reference:', json.dumps(clock), flush=True)
    hosts = args.host or default_hosts or ['192.168.123.161', '192.168.123.13', '192.168.123.14', '192.168.123.15']
    name = args.bag_name or datetime.now().strftime('%Y%m%d_%H%M%S_sync_capture')
    folder = args.output.expanduser().resolve()/name
    if folder.exists(): raise FileExistsError(folder)
    folder.parent.mkdir(parents=True, exist_ok=True)
    q = queue.Queue(maxsize=args.queue_size)
    connections, subs, specs, stats = [], [], {}, {}
    drops, errors = Counter(), Counter()
    writer = None; timeline = None
    status = 'complete'
    rclpy.init(args=[])
    try:
        discovered = []
        for host in hosts:
            ros = roslibpy.Ros(host=host, port=9090); connections.append(ros); ros.run(timeout=8)
            result = roslibpy.Service(ros,'/rosapi/topics','rosapi/Topics').call(roslibpy.ServiceRequest({}),timeout=5)
            for topic, kind in select_topics(dict(zip(result['topics'],result['types']))).items():
                discovered.append((host,ros,topic,kind))
        if not any(k=='sensor_msgs/CompressedImage' for _,_,_,k in discovered):
            raise RuntimeError('No compressed image topics available')
        if not args.measure_only:
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(folder),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        else: folder.mkdir()
        timeline = (folder/'timing.jsonl').open('w')
        names=Counter(topic for _,_,topic,_ in discovered)
        for index,(host,ros,topic,kind) in enumerate(discovered,1):
            output=topic if names[topic]==1 else '/source_'+host.replace('.','_')+topic
            try: cls=get_message(kind.replace('/','/msg/',1)); native=True
            except (ImportError,AttributeError,ValueError): cls=String; native=False
            specs[output]=(host,topic,kind,cls,native)
            stats[output]={'source_host':host,'ros1_topic':topic,'ros1_type':kind,'arrival':[],'source':[],'seq':[],'age_ms':[],'bytes':0}
            if writer:
                writer.create_topic(rosbag2_py.TopicMetadata(id=index,name=output,type=kind.replace('/','/msg/',1) if native else 'std_msgs/msg/String',serialization_format='cdr'))
            def callback(data, output=output):
                try: q.put_nowait((output,data,time.time_ns(),time.monotonic_ns()))
                except queue.Full: drops[output]+=1
            sub=roslibpy.Topic(ros,topic,kind,throttle_rate=0,queue_length=10,reconnect_on_close=False)
            sub.subscribe(callback); subs.append(sub)
            print('Selected',host,topic,flush=True)
        deadline=time.monotonic()+args.duration
        while time.monotonic()<deadline:
            if not all(ros.is_connected for ros in connections): raise RuntimeError('Rosbridge disconnected')
            try: topic,data,arrival,mono=q.get(timeout=.1)
            except queue.Empty: continue
            host,original,kind,cls,native=specs[topic]; stat=stats[topic]
            source=source_stamp_ns(data)
            age_ms=((arrival+clock['offset_s']*1e9-source)/1e6) if source else None
            if kind=='sensor_msgs/CompressedImage' and source and abs(age_ms)>1000:
                raise RuntimeError('Camera timestamp differs from common clock by %.1f ms: %s. Wait for time sync or investigate backlog.' % (age_ms,topic))
            stat['arrival'].append(arrival); stat['source'].append(source)
            seq=data.get('header',{}).get('seq'); stat['seq'].append(seq)
            if age_ms is not None: stat['age_ms'].append(age_ms)
            bag_stamp=source or round(arrival+clock['offset_s']*1e9)
            try:
                if kind=='sensor_msgs/CompressedImage':
                    msg=compressed_message(data); stat['bytes']+=len(msg.data)
                elif native:
                    msg=cls(); set_message_fields(msg,restore_nulls(normalize(data),msg))
                else:
                    msg=String(data=json.dumps({'ros1_type':kind,'message':data,'receipt_time_ns':arrival},separators=(',',':')))
                if writer: writer.write(topic,serialize_message(msg),bag_stamp)
                timeline.write(json.dumps({'topic':topic,'source_ns':source,'arrival_ns':arrival,'arrival_monotonic_ns':mono,
                    'bag_timestamp_ns':bag_stamp,'seq':seq,'age_ms':age_ms})+'\n')
            except Exception as error:
                errors[topic]+=1
                raise RuntimeError('Recording conversion failed for '+topic+': '+str(error)) from error
    except KeyboardInterrupt: status='interrupted'
    except Exception:
        status='aborted'; raise
    finally:
        for sub in subs: sub.unsubscribe()
        if writer: writer.close()
        if timeline: timeline.close()
        if folder.exists():
            summary={}
            def rate(values): return (len(values)-1)*1e9/(values[-1]-values[0]) if len(values)>1 and values[-1]>values[0] else None
            for topic,s in stats.items():
                seq=[v for v in s['seq'] if v is not None]
                skipped=sum(max(0,b-a-1) for a,b in zip(seq,seq[1:]) if b>=a)
                ages=sorted(s['age_ms'])
                positive=[v for v in s['source'] if v>0]
                summary[topic]={k:s[k] for k in ('source_host','ros1_topic','ros1_type')}
                summary[topic].update({'received':len(s['arrival']),'arrival_fps':rate(s['arrival']),'source_fps':rate(positive),
                    'source_sequence_gaps':skipped,'local_queue_drops':drops[topic],'conversion_errors':errors[topic],
                    'age_median_ms':statistics.median(ages) if ages else None,'age_p95_ms':ages[int(.95*(len(ages)-1))] if ages else None,
                    'age_max_ms':max(ages) if ages else None,'image_bytes':s['bytes']})
            if status=='complete' and (any(not s['arrival'] for s in stats.values()) or q.qsize() or sum(drops.values())):
                status='partial'
            report={'status':status,'clock_reference':clock,'bag_time':'source header time, otherwise Pi-corrected laptop receipt time',
                    'hardware_synchronization':False,'measure_only':args.measure_only,'topics':summary,'unprocessed_queue_messages':q.qsize()}
            (folder/'sync_report.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report,indent=2),flush=True)
            print('Output:',folder,flush=True)
        # roslibpy shares one Twisted reactor; only its first manager owns the thread.
        for ros in connections:
            if ros.is_connected: ros.close()
        if connections: connections[0].terminate()
        rclpy.shutdown()


if __name__=='__main__': main()
