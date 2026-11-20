#!/usr/bin/env python3
"""Check the experiment packages against their *_FREEZE_HASHES.json files.

Packages copied into another package (under vendor/) only contain the files that package uses,
so missing files are fine there.

usage: python3 tools/verify_freezes.py [ROOT]
"""
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    n_fail = 0
    freeze_files = sorted(ROOT.joinpath('experiments').rglob('*_FREEZE_HASHES.json'))
    for fz in freeze_files:
        hashes = json.loads(fz.read_text())['sha256']
        pkg = fz.parent
        vendored = 'vendor' in pkg.relative_to(ROOT).parts
        changed = [f for f, h in hashes.items() if (pkg / f).is_file() and sha256(pkg / f) != h]
        missing = [f for f in hashes if not (pkg / f).is_file()]
        ok = not changed and (vendored or not missing)
        n_fail += not ok
        extra = ''
        if vendored and missing:
            extra = f', {len(hashes) - len(missing)} of {len(hashes)} vendored'
        print(f'{"OK  " if ok else "FAIL"} {fz.relative_to(ROOT)}  ({len(hashes)} files{extra}, sha256 {sha256(fz)[:12]})')
        for f in changed:
            print('       changed:', f)
        if not vendored:
            for f in missing:
                print('       missing:', f)
    print(f'{len(freeze_files)} freeze files checked, {n_fail} failed')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
