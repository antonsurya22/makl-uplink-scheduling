#!/usr/bin/env python3
"""Frozen J5b predictions from the frozen J5 replay core with a virtual clock (no network): per trace, the scan age at
the link (p95 of the peak ages in W), useful frames F (delivery in W within 1.0 s of capture) and the open-loop
violation, for every arm in Q1 (A, NP at 0.75 c) and Q2 (A, NP at 0.75 c). J5 showed that the scheduler on the Pi
reproduces this replay exactly (80 of 80 replays), so these are the values the real stack must reach if the network
adds no queue. Usage: make_predictions.py OUT.json"""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5b_common as B
from j5b_common import C
import j5_analysis as AN


def predict():
    out = {}
    for cell in ('Q1', 'Q2'):
        for mode in ('A', 'NP'):
            for arm in C.ARMS:
                rows = []
                for rep in C.REPS:
                    tr = C.load_trace(cell, rep)
                    R = C.Replayer(tr, C.make_link(arm), mode=mode, cell=cell, clock=C.VirtualClock(), tc_rate=lambda c: None).run()
                    T = R.T
                    sc = [(fin, cap) for (_, s, cap, _, fin, _, _) in R.sent if s == 'scan']
                    cam = [(fin, cap) for (_, s, cap, _, fin, _, _) in R.sent if s == 'camera']
                    F = sum(1 for t, c in cam if T <= t < T + int(C.DUR_S * 1e6) and t - c <= 1_000_000) / C.DUR_S
                    v = C.JR.violation({'recv': [('scan', t, c, None, None) for t, c in sc], 'T': T}, tr['grid'])['viol_s']
                    rows.append({'rep': rep, 'alink_p95_s': round(AN.pct(AN.peaks(sc, T), .95), 4), 'F_fps': round(F, 4),
                                 'viol_s': v})
                out[f'{cell}|{"NP75" if mode == "NP" else "A"}|{arm}'] = rows
    return {'pace': C.PACE, 'source': 'frozen J5 replay core, virtual clock, delay 0', 'cells': out}


if __name__ == '__main__':
    Path(sys.argv[1]).write_text(json.dumps(predict(), sort_keys=True, indent=0) + '\n')
    print('J5B_PREDICTIONS written', sys.argv[1])
