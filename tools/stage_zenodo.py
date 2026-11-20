#!/usr/bin/env python3
"""Copy the archives for Zenodo into one folder and write SHA256SUMS.txt.

The ZIPs are copied as they are (nothing inside is changed).

usage: python3 tools/stage_zenodo.py OUT_DIR FILE [FILE ...]
"""
import hashlib
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for f in map(Path, sys.argv[2:]):
        if not f.is_file():
            sys.exit(f'not a file: {f}')
        dst = out / f.name
        if dst.exists() and dst.read_bytes() != f.read_bytes():
            sys.exit(f'another file with the same name is already in {out}: {f.name}')
        shutil.copy2(f, dst)
    readme = ROOT / 'docs' / 'ZENODO_README.md'
    if readme.is_file():
        shutil.copy2(readme, out / 'README.md')
    lines = [f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}'
             for p in sorted(out.iterdir()) if p.is_file() and p.name != 'SHA256SUMS.txt']
    (out / 'SHA256SUMS.txt').write_text('\n'.join(lines) + '\n')
    print(f'{len(lines)} files in {out}')
