#!/usr/bin/env python3
"""Lossless, time-ordered merge of the four downloaded native ROS1 bags."""
import argparse
from collections import Counter
from contextlib import ExitStack
import hashlib
import heapq
import json
from pathlib import Path
import shutil
import struct
import uuid

ROLES=('pi','13','14','15')


def digest_file(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle,'sha256').hexdigest()


def signature(connection):
    return (connection.topic,connection.msgtype,connection.msgdef.data,connection.digest,
            connection.ext.callerid,connection.ext.latching)


def update_digest(digest,timestamp,data):
    digest.update(struct.pack('<QQ',timestamp,len(data)));digest.update(data)


def merge(folder):
    from rosbags.rosbag1 import Reader,Writer
    folder=Path(folder).resolve()
    sources={role:folder/role/'capture.bag' for role in ROLES}
    for path in sources.values():
        if not path.is_file():raise RuntimeError('Missing source bag: '+str(path))
    source_hashes={role:digest_file(path) for role,path in sources.items()}
    output=folder/'combined.bag';report_path=folder/'merge_report.json'
    if output.exists():
        if report_path.is_file():
            report=json.loads(report_path.read_text())
            if (report.get('verified') and report.get('source_sha256')==source_hashes
                    and report.get('output_sha256')==digest_file(output)):
                return report
        raise RuntimeError('Existing combined.bag is unverified or sources changed; preserved without overwriting')
    if shutil.disk_usage(folder).free<sum(p.stat().st_size for p in sources.values())*1.2+64*1024**2:
        raise RuntimeError('Insufficient free disk space for a separate verified merged bag')
    temporary=folder/('.combined_'+uuid.uuid4().hex+'.bag')
    expected={};counts=Counter();topic_counts=Counter();mapping={};connections={}
    try:
        with ExitStack() as stack:
            readers=[stack.enter_context(Reader(sources[role])) for role in ROLES]
            writer=stack.enter_context(Writer(temporary))
            for index,reader in enumerate(readers):
                for connection in reader.connections:
                    key=signature(connection)
                    if key not in connections:
                        out=writer.add_connection(connection.topic,connection.msgtype,
                            msgdef=connection.msgdef.data,md5sum=connection.digest,
                            callerid=connection.ext.callerid,latching=connection.ext.latching)
                        connections[key]=out;expected[key]=hashlib.sha256()
                    mapping[index,connection.id]=connections[key]
            def rows(index,reader):
                for connection,timestamp,data in reader.messages():
                    yield timestamp,index,connection,data
            for timestamp,index,connection,data in heapq.merge(*(rows(i,r) for i,r in enumerate(readers)),key=lambda row:(row[0],row[1])):
                out=mapping[index,connection.id]
                writer.write(out,timestamp,data)
                update_digest(expected[signature(connection)],timestamp,data)
                counts[ROLES[index]]+=1;topic_counts[connection.topic]+=1
        observed={key:hashlib.sha256() for key in expected};observed_topics=Counter();observed_count=0;previous=-1
        with Reader(temporary) as reader:
            for connection,timestamp,data in reader.messages():
                if timestamp<previous:raise RuntimeError('Merged bag timestamps out of order')
                previous=timestamp;observed_count+=1;observed_topics[connection.topic]+=1
                update_digest(observed[signature(connection)],timestamp,data)
        if observed_count!=sum(counts.values()) or observed_topics!=topic_counts:
            raise RuntimeError('Merged message counts do not match inputs')
        if any(expected[key].digest()!=observed[key].digest() for key in expected):
            raise RuntimeError('Merged timestamps or serialized message bytes differ from sources')
        report=dict(schema_version=1,verified=True,output='combined.bag',
            message_count=observed_count,source_counts=dict(counts),topic_count=len(topic_counts),
            topic_counts=dict(topic_counts),source_sha256=source_hashes,output_sha256=digest_file(temporary),
            semantics='Original ROS1 serialized bytes, source headers, bag receipt timestamps and connection metadata retained. No filtering, resampling, deduplication or clock rewriting.',
            source_bags_retained=True)
        temporary.rename(output)
        staged_report=folder/('.merge_report_'+uuid.uuid4().hex+'.json')
        staged_report.write_text(json.dumps(report,indent=2));staged_report.replace(report_path)
        return report
    except BaseException:
        # Partial outputs are diagnostic evidence and never replace a valid bag.
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path,nargs='?');parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    if args.check:
        from rosbags.rosbag1 import Reader,Writer
        return
    if args.dataset is None:parser.error('Provide the dataset folder')
    report=merge(args.dataset)
    print('[MERGED] '+str(args.dataset.resolve()/'combined.bag'))
    print('[MERGED] Verified %d messages across %d topics'%(report['message_count'],report['topic_count']))

if __name__=='__main__':main()
