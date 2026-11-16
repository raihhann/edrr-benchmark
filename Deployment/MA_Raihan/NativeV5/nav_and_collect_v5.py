#!/usr/bin/env python3
"""Default: stationary native robot recording, then verified automatic download."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'DataCollection'))
from go1_local_client import main
if __name__=='__main__':sys.exit(main())
