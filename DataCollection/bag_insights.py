#!/usr/bin/env python3
"""Print offline TurtleBot SQLite bag insights in the terminal; creates no files."""
import argparse
import json
import sqlite3
from pathlib import Path
from bag_insights_html import inspect, fmt


def print_report(report):
    print('Bag:', report['bag'])
    print(f'{report["duration_s"]:.2f} seconds | {report["messages"]} messages | {report["topic_count"]} topics | {report["database_size_mb"]:.1f} MB')
    integrity = all(v == ['ok'] for v in report['sqlite_quick_check'].values())
    print('SQLite check:', 'OK' if integrity else report['sqlite_quick_check'])
    print('\nSTREAMS (rates in Hz/FPS; gap and delays in milliseconds)')
    width=max(5,max(map(len,report['topics']),default=5))
    print(f'{"Topic":<{width}} {"Count":>7} {"Source Hz":>10} {"Receipt Hz":>11} {"Max gap":>10} {"Delay med":>10} {"Delay p95":>10}')
    print('-'*(width+70))
    for name,item in report['topics'].items():
        delay=item['receipt_minus_source_ms']
        print(f'{name:<{width}} {item["messages"]:>7} {fmt(item["source"]["hz"]):>10} {fmt(item["receipt"]["hz"]):>11} {fmt(item["source"]["max_gap_ms"]):>10} {fmt(delay["median"]):>10} {fmt(delay["p95"]):>10}')
    print('\nCAMERA TIMESTAMP PROXIMITY (nearest reference stamp, milliseconds)')
    for item in report['camera_alignment']:
        d=item['nearest_stamp_difference_ms']
        print(f'  {item["camera"]} -> {item["reference"]}:')
        print(f'    median {fmt(d["median"])}, p95 {fmt(d["p95"])}, max {fmt(d["max"])}; paired {item["paired_frames"]}, outside coverage {item["outside_reference_window"]}')
    if not report['camera_alignment']:
        print('  No supported camera/reference timestamp pairs.')
    print('\nNAVIGATION (saved experiment metadata)')
    print('  Mode:', report['experiment'].get('mode'), '| Status:', report['experiment'].get('status'))
    for event in report['navigation_events']:
        print(' ',json.dumps(event))
    audit=report.get('saved_native_audit')
    if audit:
        print('\nSAVED NATIVE AUDIT')
        print('  Storage/decode OK:',audit.get('storage_and_decode_ok'), '| Message count matches:',audit.get('message_count_matches'))
        print('  Last driver command:',audit.get('last_driver_command'), '| Odom displacement:',audit.get('odom_displacement_m'),'m')
    print('\nWARNINGS')
    for warning in report['warnings']:
        print('  -',warning)
    if not report['warnings']:
        print('  No required-stream or timestamp warnings detected.')
    print('\nINTERPRETATION LIMITS')
    for limit in report['limitations']:
        print('  -',limit)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path,help='Downloaded bag folder or .db3 file')
    args=parser.parse_args()
    try:
        report=inspect(args.bag.expanduser().resolve())
    except (ValueError, sqlite3.Error, OSError) as error:
        parser.exit(1,str(error)+'\n')
    print_report(report)


if __name__=='__main__':
    main()
