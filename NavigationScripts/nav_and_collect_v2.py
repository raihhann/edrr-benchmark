#!/usr/bin/env python3
"""Default: stationary recording. Actual navigation requires --navigate."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'DataCollection'))
from client import main
if __name__ == '__main__':
    sys.exit(main())
