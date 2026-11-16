#!/usr/bin/env python3
"""Convert the verified all-controller ROS1 merge into one ROS2 SQLite bag."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import uuid


def file_hash(path):
    with path.open('rb') as handle:return hashlib.file_digest(handle,'sha256').hexdigest()


def accumulate(digests,key,stamp,data):
    digest=digests.setdefault(key,hashlib.sha256())
    digest.update(struct.pack('<QQ',stamp,len(data)));digest.update(data)


def convert_verified(folder):
    from rosbags.convert import convert
    from rosbags.convert.converter import generate_message_converter,STATIC_MSGTYPE_RENAMES
    from rosbags.highlevel import AnyReader
    folder=Path(folder).resolve();source=folder/'combined.bag'
    from merge_native_bags import merge
    merged=merge(folder)
    destination=folder/'combined_ros2';receipt=folder/'db3_report.json'
    if destination.exists():
        if receipt.is_file():
            report=json.loads(receipt.read_text())
            if (report.get('verified') and report.get('source_sha256')==file_hash(source)
                and all((destination/name).is_file() and file_hash(destination/name)==digest
                        for name,digest in report['output_sha256'].items())):
                return report
        raise RuntimeError('Existing ROS2 output cannot be verified; preserved without overwrite')
    if shutil.disk_usage(folder).free<source.stat().st_size*1.3+64*1024**2:
        raise RuntimeError('Insufficient laptop space for verified ROS2 conversion')
    staging=folder/('.ros2_convert_'+uuid.uuid4().hex);staging.mkdir()
    temporary=staging/'combined_ros2'
    convert([source],temporary,'sqlite3',9,None,'file',None,None,[],[],[],[])
    databases=list(temporary.glob('*.db3'))
    if len(databases)!=1:raise RuntimeError('Expected exactly one combined SQLite database')
    with sqlite3.connect('file:'+str(databases[0])+'?mode=ro',uri=True) as database:
        if database.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise RuntimeError('SQLite integrity check failed')
    expected={};actual={};counts=Counter();output_counts=Counter();cache={}
    with AnyReader([source]) as src,AnyReader([temporary]) as dst:
        converters={}
        for connection in src.connections:
            target=STATIC_MSGTYPE_RENAMES.get(connection.msgtype,connection.msgtype)
            converters[connection.id]=(target,generate_message_converter(src.typestore,dst.typestore,
                connection.msgtype,target,cache,src_is2=False,dst_is2=True))
        for connection,stamp,raw in src.messages():
            target,converter=converters[connection.id];cdr=converter(raw)
            key=(connection.topic,target);accumulate(expected,key,stamp,cdr);counts[key]+=1
        previous=-1
        for connection,stamp,cdr in dst.messages():
            if stamp<previous:raise RuntimeError('ROS2 output timestamps out of order')
            previous=stamp
            dst.deserialize(cdr,connection.msgtype)
            key=(connection.topic,connection.msgtype);accumulate(actual,key,stamp,cdr);output_counts[key]+=1
    if counts!=output_counts or any(expected[key].digest()!=actual[key].digest() for key in expected):
        raise RuntimeError('ROS2 conversion count/payload/timestamp verification failed')
    total=sum(counts.values())
    if total!=merged['message_count']:raise RuntimeError('ROS2 count does not match verified native merge')
    output_hashes={path.name:file_hash(path) for path in temporary.iterdir() if path.is_file()}
    report=dict(schema_version=1,verified=True,source_sha256=file_hash(source),output_sha256=output_hashes,
        database=str(Path('combined_ros2')/databases[0].name),message_count=total,topic_count=len({k[0] for k in counts}),
        semantics='ROS2 CDR conversion; all messages, source stamp values and bag receipt timestamps retained. Standard ROS2 Header has no ROS1 seq field; original seq values remain in the retained local ROS1 bags and sequence audit. Custom message definitions embedded; installed ROS2 custom interfaces may be needed for playback consumers.')
    temporary.rename(destination)
    staged=folder/('.db3_report_'+uuid.uuid4().hex+'.json');staged.write_text(json.dumps(report,indent=2));staged.replace(receipt)
    staging.rmdir()
    return report


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('dataset',type=Path)
    args=parser.parse_args();report=convert_verified(args.dataset)
    print('[DB3] '+str(args.dataset.resolve()/report['database']))
    print('[DB3] Verified %d messages, %d topics; SQLite integrity and converted payloads/timestamps passed'%(report['message_count'],report['topic_count']))

if __name__=='__main__':main()
