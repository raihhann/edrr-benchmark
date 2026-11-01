#!/usr/bin/env python3
import argparse
from client import fetch
if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Download and SHA-256 verify a finalized robot bag')
    p.add_argument('name')
    p.add_argument('--host')
    a = p.parse_args()
    fetch(a.name, a.host)
