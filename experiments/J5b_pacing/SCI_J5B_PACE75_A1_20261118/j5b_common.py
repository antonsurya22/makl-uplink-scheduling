"""J5b: the frozen J5 package (vendor/SCI_J5_NETSTACK_20261118, unchanged) with ONE scheduler constant changed:
the pace of mode NP is 0.75 c instead of 0.9 c (in the scheduler's own byte count). Everything else - schedulers, link
model, replay loop, DDS transport, tc settings, checks - is the J5 code. This module also sets the J5b run matrix:
  counted:  Q1, modes A and NP (pace 0.75 c), 4 arms x 5 traces = 40 replays in 10 blocks
  demo:     Q2, mode NP (pace 0.75 c), 4 arms x 5 traces = 20 replays in 5 blocks (exploratory; after the counted ones)
Import this module before any J5 module that reads C.PACE or C.matrix."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
J5DIR = HERE / 'vendor' / 'SCI_J5_NETSTACK_20261118'
sys.path.insert(0, str(J5DIR))
import j5_common as C          # noqa: E402  (frozen J5)

PACE_J5B = 0.75
C.PACE = PACE_J5B              # the one changed constant (J5: 0.9)
PACKAGE = 'SCI_J5B_PACE75_20261118'
J5_RESULTS_REL = 'ros2_ws/results/SCI_J5_NETSTACK_20261118/J5_MAIN'   # under $HOME on both hosts
PRED_LINK_S = {'Q1': round(8 * (3000 + 3170) / 180_000 * (1 / PACE_J5B - 1), 4)}   # 0.0914 s


def matrix():
    """Counted Q1 blocks (rep x {A, NP}, mode order alternating per rep, arms rotated), then the Q2 demo blocks."""
    rows = []; b = 0
    for rep in C.REPS:
        ms = ['A', 'NP'] if rep % 2 == 1 else ['NP', 'A']
        for mode in ms:
            a = list(C.ARMS); r = b % 4; a = a[r:] + a[:r]; b += 1
            for arm in a:
                rows.append({'rid': f'J5B_R{rep}_Q1_{"NP75" if mode == "NP" else "A"}_{arm}', 'rep': rep, 'cell': 'Q1',
                             'mode': mode, 'arm': arm, 'block': b, 'part': 'counted'})
    for rep in C.REPS:
        a = list(C.ARMS); r = b % 4; a = a[r:] + a[:r]; b += 1
        for arm in a:
            rows.append({'rid': f'J5B_R{rep}_Q2_NP75_{arm}', 'rep': rep, 'cell': 'Q2', 'mode': 'NP', 'arm': arm,
                         'block': b, 'part': 'demo'})
    assert len(rows) == 60
    return rows


C.matrix = matrix              # the frozen J5 runner calls C.matrix(); pilots stay the J5 pilots (C.pilots)
