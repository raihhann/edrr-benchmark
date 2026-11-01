#!/usr/bin/env python3
"""SSH front end. ROS runs on the robot; laptop ROS distribution is independent."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import uuid

PROJECT = Path(__file__).resolve().parent.parent
SSH_OPTIONS = ['-o', 'ControlMaster=auto', '-o', 'ControlPersist=600',
               '-o', 'ControlPath='+str(PROJECT/'.runtime/ssh_%C'), '-o', 'ServerAliveInterval=2',
               '-o', 'ServerAliveCountMax=3']


def name_ok(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('Bag name: 1–80 letters, digits, underscore or hyphen; no paths')
    return name


def connection(host=None):
    (PROJECT/'.runtime').mkdir(mode=0o700, exist_ok=True)
    cfg = json.loads((PROJECT/'Config/connection.json').read_text())
    if host:
        cfg['host'] = host
    for field in ['host', 'user']:
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+', cfg[field]):
            raise ValueError('Invalid SSH '+field)
    if cfg['robot_directory'] != 'MA_Raihan':
        raise ValueError('Robot directory must be MA_Raihan')
    return cfg, cfg['user']+'@'+cfg['host']


def remote(target, words, tty=False, **kwargs):
    return subprocess.run(['ssh', *( ['-tt'] if tty else []), *SSH_OPTIONS, target,
                           shlex.join(words)], **kwargs)


def fetch(name, host=None):
    name_ok(name)
    cfg, target = connection(host)
    bags = PROJECT/'Bags'
    bags.mkdir(exist_ok=True)
    dest = bags/name
    if dest.exists():
        raise RuntimeError('Local bag exists; refusing overwrite: '+str(dest))
    incoming = bags/('.incoming_'+name+'_'+uuid.uuid4().hex[:6])
    incoming.mkdir()
    source = target+':'+cfg['robot_directory']+'/Bags/'+name
    subprocess.run(['scp', *SSH_OPTIONS, '-r', source, str(incoming)], check=True)
    staged = incoming/name
    manifest = remote(target, ['python3', 'MA_Raihan/Scripts/manifest.py', name],
                      capture_output=True, text=True, check=True)
    expected = json.loads(manifest.stdout)
    for rel, digest in expected.items():
        path = staged/rel
        if not path.resolve().is_relative_to(staged.resolve()):
            raise RuntimeError('Invalid remote manifest path')
        h = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                h.update(chunk)
        if h.hexdigest() != digest:
            raise RuntimeError('Checksum mismatch: '+rel+'; partial download retained at '+str(staged))
    if not (staged/'metadata.yaml').is_file():
        raise RuntimeError('Bag metadata missing; download retained at '+str(staged))
    staged.rename(dest)
    (dest/'download_verified.json').write_text(json.dumps(expected, indent=2))
    incoming.rmdir()  # Empty temporary transfer directory only.
    print('Verified laptop bag:', dest)
    return dest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--record-only', action='store_true')
    mode.add_argument('--navigate', action='store_true', help='Explicitly enable actual movement')
    p.add_argument('--duration', type=float, default=20.)
    p.add_argument('--distance', type=float, default=1.)
    p.add_argument('--speed', type=float, default=.1)
    p.add_argument('--bag-name')
    p.add_argument('--host')
    p.add_argument('--require-topic', action='append', default=[])
    p.add_argument('--startup-timeout', type=float, default=20.)
    p.add_argument('--audit', action='store_true', help='Decode and validate the finalized robot bag before download')
    p.add_argument('--no-fetch', action='store_true', help='Keep final bag on robot; fetch later')
    args = p.parse_args(argv)
    cfg, target = connection(args.host)
    name = name_ok(args.bag_name or datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8])
    if (PROJECT/'Bags'/name).exists():
        p.error('Local bag already exists; choose another name')
    words = ['bash', 'MA_Raihan/run.sh', 'navigate' if args.navigate else 'record',
             '--duration', str(args.duration), '--distance', str(args.distance),
             '--speed', str(args.speed), '--bag-name', name,
             '--startup-timeout', str(args.startup_timeout)]
    for topic in args.require_topic:
        words += ['--require-topic', topic]
    print('Robot:', target, '| Mode:', 'MOVEMENT' if args.navigate else 'stationary recording', flush=True)
    print('Robot bag: ~/MA_Raihan/Bags/'+name, flush=True)
    try:
        result = remote(target, words, tty=True)
        code = result.returncode
    except KeyboardInterrupt:
        print('\nSSH interrupted. The robot-side runner handles hangup and sends stop commands; guard expires stale commands.')
        code = 130
    if args.audit and code == 0:
        audit_result = remote(target, ['bash', 'MA_Raihan/audit.sh', name], tty=True)
        code = audit_result.returncode or code
    if not args.no_fetch:
        # Aborted runs may still be useful; keep their explicit experiment status.
        try:
            fetch(name, args.host)
        except Exception as error:
            print('Fetch not complete:', error, file=sys.stderr)
            print('Recover later with: python3 DataCollection/fetch_bag.py '+name, file=sys.stderr)
            code = code or 1
    return code


if __name__ == '__main__':
    sys.exit(main())
