#!/usr/bin/env python3
"""Compressed-only stationary vision capture; see sync_report.json and timing.jsonl."""
from sync_capture import main

if __name__ == '__main__':
    main(default_hosts=['192.168.123.14'])
