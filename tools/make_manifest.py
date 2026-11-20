#!/usr/bin/env python3
"""Write MANIFEST.sha256 for all files in the repo (same format as sha256sum).

usage: python3 tools/make_manifest.py [--check]
"""
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'MANIFEST.sha256'
SKIP = {'.git', '__pycache__', 'build', 'install', 'log'}


def files():
    for p in sorted(ROOT.rglob('*')):
        rel = p.relative_to(ROOT)
        if p.is_file() and p != OUT and not SKIP.intersection(rel.parts) and p.suffix != '.pyc':
            yield rel


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == '__main__':
    if '--check' in sys.argv:
        listed = {}
        for line in OUT.read_text().splitlines():
            h, f = line.split('  ', 1)
            listed[f] = h
        bad = [f for f, h in listed.items() if not (ROOT / f).is_file() or sha256(ROOT / f) != h]
        new = [str(f) for f in files() if str(f) not in listed]
        for f in bad:
            print('changed or missing:', f)
        for f in new:
            print('not in manifest:', f)
        print(f'{len(listed)} files in manifest, {len(bad)} changed or missing, {len(new)} new')
        sys.exit(1 if bad or new else 0)
    lines = [f'{sha256(ROOT / f)}  {f.as_posix()}' for f in files()]
    OUT.write_text('\n'.join(lines) + '\n')
    print(f'{OUT.name}: {len(lines)} files')
