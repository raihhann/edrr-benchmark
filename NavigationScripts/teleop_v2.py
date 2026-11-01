#!/usr/bin/env python3
"""Keyboard driving without recording, with configurable bounded speeds."""
import argparse
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'DataCollection'))
from client import connection, remote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=.08, help='Forward/backward speed in m/s (0, 0.26]')
    parser.add_argument('--turn-speed', type=float, default=.35, help='Turning speed in rad/s (0, 0.8]')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0 < args.speed <= .26:
        parser.error('--speed must be greater than 0 and at most 0.26 m/s')
    if not math.isfinite(args.turn_speed) or not 0 < args.turn_speed <= .8:
        parser.error('--turn-speed must be greater than 0 and at most 0.8 rad/s')
    _, target = connection()
    return remote(target, ['bash', 'MA_Raihan/teleop.sh', '--speed', str(args.speed), '--turn-speed', str(args.turn_speed)], tty=True).returncode


if __name__ == '__main__':
    raise SystemExit(main())
