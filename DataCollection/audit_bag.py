#!/usr/bin/env python3
"""Run the full native-message audit where the Humble interfaces are installed."""
import argparse
from pathlib import Path
import subprocess
from client import connection, remote, name_ok
if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('name')
    p.add_argument('--host')
    a = p.parse_args(); name_ok(a.name)
    _, target = connection(a.host)
    # Fixed helper avoids shell/environment ambiguity and preserves the robot setup.
    raise SystemExit(remote(target, ['bash', 'MA_Raihan/audit.sh', a.name], tty=True).returncode)
