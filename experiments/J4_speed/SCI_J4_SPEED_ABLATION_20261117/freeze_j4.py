#!/usr/bin/env python3
"""Writes J4_FREEZE_HASHES.json (SHA256 of every package file except itself and freeze_j4.py)."""
import datetime,hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
files=sorted(str(p.relative_to(H)) for p in H.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name not in ('J4_FREEZE_HASHES.json','freeze_j4.py'))
h={f:hashlib.sha256((H/f).read_bytes()).hexdigest() for f in files}
(H/'J4_FREEZE_HASHES.json').write_text(json.dumps({'frozen_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'sha256':h},indent=1)+'\n')
print('J4_FROZEN',len(h),'files')
