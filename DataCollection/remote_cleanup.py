#!/usr/bin/env python2
"""Remove only one finalized, checksum-matched recorded file; keep all metadata."""
from __future__ import print_function
import hashlib
import json
import os
import re
import sys


def cleanup(run_id,expected):
    if not re.match(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,99}$',run_id):raise ValueError('Invalid run id')
    if not re.match(r'^[0-9a-f]{64}$',expected):raise ValueError('Invalid digest')
    root=os.path.expanduser('~/MA_Raihan/captures')
    folder=os.path.join(root,run_id);bag=os.path.join(folder,'capture.bag')
    if os.path.islink(root) or os.path.islink(folder) or os.path.realpath(folder)!=os.path.join(os.path.realpath(root),run_id):
        raise ValueError('Unsafe capture path')
    if os.path.exists(bag+'.active'):raise ValueError('Recorder has an active bag; refusing cleanup')
    with open(os.path.join(folder,'run.json')) as handle:state=json.load(handle)
    if state.get('run_id')!=run_id or not state.get('finished_ns'):raise ValueError('Run not finalized')
    pidroot=os.path.expanduser('~/MA_Raihan/pids')
    for prefix in ['native_','native_record_']:
        pidpath=os.path.join(pidroot,prefix+run_id)
        if os.path.isfile(pidpath):
            with open(pidpath) as handle:pid=handle.read().strip()
            if pid.isdigit() and os.path.exists('/proc/'+pid+'/cmdline'):
                with open('/proc/'+pid+'/cmdline','rb') as handle:command=handle.read()
                if run_id.encode('ascii') in command:raise ValueError('Run process remains active')
    if not os.path.exists(bag):return dict(status='already_absent',run_id=run_id)
    if os.path.islink(bag) or not os.path.isfile(bag):raise ValueError('Unsafe recorded file')
    before=os.stat(bag);digest=hashlib.sha256()
    with open(bag,'rb') as handle:
        for chunk in iter(lambda:handle.read(1024*1024),b''):digest.update(chunk)
    if digest.hexdigest()!=expected:raise ValueError('Remote bag checksum mismatch; retained')
    after=os.stat(bag)
    if (before.st_ino,before.st_size,before.st_mtime)!=(after.st_ino,after.st_size,after.st_mtime):
        raise ValueError('Recorded file changed; retained')
    os.unlink(bag)
    return dict(status='deleted_verified_capture',run_id=run_id,bytes=before.st_size,sha256=expected)

if __name__=='__main__':print(json.dumps(cleanup(sys.argv[1],sys.argv[2])))
