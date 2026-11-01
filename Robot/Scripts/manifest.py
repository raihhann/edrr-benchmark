#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
import sys
from control import valid_name
root = Path(__file__).resolve().parent.parent/'Bags'/valid_name(sys.argv[1])
if not (root/'metadata.yaml').exists():
    raise SystemExit('Bag is missing or not finalized')
values = {}
for path in sorted(root.rglob('*')):
    if path.is_file():
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                digest.update(chunk)
        values[str(path.relative_to(root))] = digest.hexdigest()
print(json.dumps(values))
