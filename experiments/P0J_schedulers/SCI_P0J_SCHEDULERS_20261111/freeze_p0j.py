#!/usr/bin/env python3
"""Write P0J_FREEZE_HASHES.json: SHA256 of every file of this package (except the hash file itself).
Run once, before the package is sent. run_p0j.py refuses to run if any file changed afterwards."""
import datetime, hashlib, json
from pathlib import Path
HERE = Path(__file__).resolve().parent
files = sorted(str(p.relative_to(HERE)) for p in HERE.rglob('*')
               if p.is_file() and p.name != 'P0J_FREEZE_HASHES.json' and '__pycache__' not in p.parts)
h = {f: hashlib.sha256((HERE / f).read_bytes()).hexdigest() for f in files}
now = datetime.datetime.now(datetime.timezone.utc).isoformat()
(HERE / 'P0J_FREEZE_HASHES.json').write_text(json.dumps({'frozen_utc': now, 'sha256': h}, indent=1) + '\n')
print('P0J_FROZEN', now, len(h), 'files')
