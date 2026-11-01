#!/usr/bin/env python3
import sys
from client import main
if __name__ == '__main__':
    sys.exit(main(['--record-only', *sys.argv[1:]]))
