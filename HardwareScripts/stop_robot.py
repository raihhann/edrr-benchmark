#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'DataCollection'))
from client import connection, remote
if __name__ == '__main__':
    _, target = connection()
    sys.exit(remote(target, ['bash', 'MA_Raihan/stop.sh'], tty=True).returncode)
