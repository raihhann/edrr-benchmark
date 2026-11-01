#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'DataCollection'))
from client import connection, remote
if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Start MA_Raihan drivers; no navigation commands')
    p.add_argument('--host')
    p.add_argument('--lidar', action='store_true')
    p.add_argument('--no-camera', action='store_true')
    a = p.parse_args()
    _, target = connection(a.host)
    sys.exit(remote(target, ['bash', 'MA_Raihan/start.sh',
        *(['--lidar'] if a.lidar else []), *(['--no-camera'] if a.no_camera else [])], tty=True).returncode)
