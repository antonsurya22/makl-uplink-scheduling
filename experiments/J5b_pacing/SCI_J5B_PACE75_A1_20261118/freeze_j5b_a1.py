#!/usr/bin/env python3
"""J5b A1 package freeze. Usage: freeze_j5b_a1.py         -> writes J5B_A1_FREEZE_HASHES.json (once, before the counted runs)
                            freeze_j5b_a1.py --check -> exit 0 only if every frozen file is present and unchanged
Frozen: every file of the package except J5B_A1_FREEZE_HASHES.json itself and Python byte-code caches."""
import hashlib, json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
OUT = HERE / 'J5B_A1_FREEZE_HASHES.json'


def files():
    return sorted(p for p in HERE.rglob('*') if p.is_file() and p != OUT and '__pycache__' not in p.parts
                  and p.suffix != '.pyc')


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


if __name__ == '__main__':
    if '--check' in sys.argv:
        z = json.loads(OUT.read_text())['sha256']; bad = []
        for f, h in z.items():
            p = HERE / f
            if not p.is_file() or sha(p) != h:
                bad.append(f)
        extra = [str(p.relative_to(HERE)) for p in files() if str(p.relative_to(HERE)) not in z and p.suffix in ('.py', '.sh', '.json')]
        for f in bad:
            print('J5B_A1_FREEZE_MISMATCH', f)
        for f in extra:
            print('J5B_A1_FREEZE_EXTRA_FILE', f)
        print('J5B_A1_FREEZE_CHECK', 'OK' if not bad and not extra else 'FAIL', len(z), 'files', 'freeze file', sha(OUT)[:12])
        sys.exit(0 if not bad and not extra else 1)
    if OUT.exists():
        sys.exit('J5B_A1_FREEZE_EXISTS: frozen files are never re-frozen; changes go in as a declared amendment')
    z = {str(p.relative_to(HERE)): sha(p) for p in files()}
    OUT.write_text(json.dumps({'package': 'SCI_J5B_PACE75_A1_20261118', 'n_files': len(z), 'sha256': z}, indent=1, sort_keys=True) + '\n')
    print('J5B_A1_FROZEN', len(z), 'files', sha(OUT))
