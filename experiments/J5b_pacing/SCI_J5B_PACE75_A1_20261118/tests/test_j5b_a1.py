#!/usr/bin/env python3
"""J5b amendment A1 tests (no ROS, no network). Usage: test_j5b_a1.py
 1. every file of the original J5b freeze (J5B_FREEZE_HASHES.json, c14886b7...) is present and unchanged
 2. BSSID reader: connected -> BSSID; one failed read then connected -> BSSID (2 attempts); 'Not connected.' -> none at
    once (a real disconnect stays invalid); 3 failed reads -> none (invalid, as in J5); each call <= 3 tries
 3. the frozen J5 validity rule with the A1 reader: a retried read is valid, a disconnect or 3 failures are not"""
import hashlib, json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import j5b_common as B
from j5b_common import C
import j5_sysops as S
import j5b_sysops_a1 as A
ok = True


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
z = json.loads((HERE / 'J5B_FREEZE_HASHES.json').read_text())['sha256']
check('original J5b files unchanged (freeze file c14886b7)', all((HERE / f).is_file() and sha(HERE / f) == h for f, h in z.items())
      and sha(HERE / 'J5B_FREEZE_HASHES.json').startswith('c14886b7'))

CONN = (0, 'Connected to 5c:e9:31:85:b4:1b (on wlan0)\n\tSSID: Sweet_Dumpling\n\tfreq: 2452\n\tsignal: -27 dBm\n\ttx bitrate: 72.2 MBit/s\n')
NOTC = (0, 'Not connected.\n')
FAIL = (99, "TimeoutExpired(['/usr/sbin/iw', 'dev', 'wlan0', 'link'], 2)")
calls = []


def run(seq):
    it = iter(seq); calls.clear()

    def fake(cmd, timeout=20):
        calls.append(timeout); return next(it)
    S.sh = fake
    A.SysOpsA1.GAP_S = 0
    return A.SysOpsA1().link_info()


r1 = run([CONN]); r2 = run([FAIL, CONN]); n2 = len(calls); r3 = run([NOTC, CONN]); n3 = len(calls)
r4 = run([FAIL, FAIL, FAIL, CONN]); n4 = len(calls); r5 = run([(0, ''), (0, 'garbage'), CONN]); n5 = len(calls)
check('reader: connected; retried read; not connected at once; 3 failures; empty outputs retried; timeout 2 s',
      r1['bssid'] == C.FROZEN_BSSID and r1['attempts'] == 1 and r1['freq'] == 2452
      and r2['bssid'] == C.FROZEN_BSSID and r2['attempts'] == 2 and n2 == 2
      and r3['bssid'] is None and r3['a1'] == 'not_connected' and n3 == 1
      and r4['bssid'] is None and r4['a1'] == 'read_failed' and n4 == 3
      and r5['bssid'] == C.FROZEN_BSSID and r5['attempts'] == 3 and set(calls) == {2}, (r2, r3, r4, r5))
# the frozen J5 rule (j5_robot_core.replay): invalid if any sample's bssid != the frozen BSSID
rule = lambda samples: not any(s.get('bssid') != C.FROZEN_BSSID for s in samples)
check('frozen validity rule with the A1 reader: retried read valid; disconnect and 3 failures invalid',
      rule([r1, r2, r5]) and not rule([r1, r3]) and not rule([r1, r4]))
import py_compile, tempfile
t = Path(tempfile.mkdtemp()); cc = True
for f in ('j5b_sysops_a1.py', 'j5b_robot_a1.py', 'freeze_j5b_a1.py', 'tests/test_j5b_a1.py'):
    try:
        py_compile.compile(str(HERE / f), doraise=True, cfile=str(t / (Path(f).name + 'c')))
    except Exception as ex:
        cc = False; print(ex)
check('A1 scripts compile; the A1 entry point uses SysOpsA1', cc and 'SysOpsA1()' in (HERE / 'j5b_robot_a1.py').read_text())
print('J5B_A1_TESTS', 'PASS' if ok else 'FAIL', flush=True)
sys.exit(0 if ok else 1)
