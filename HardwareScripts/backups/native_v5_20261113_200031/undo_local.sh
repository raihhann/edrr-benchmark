#!/bin/bash
set -e
python3 - "$(dirname -- "$0")/local_changes.json" <<'PY'
import json,shutil,sys
for item in json.load(open(sys.argv[1])):shutil.copy2(item['backup'],item['path'])
print('Restored the laptop entry points and command guide. Native datasets and unused new helper files retained.')
PY
