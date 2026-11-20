"""Distance switch vs. MAKL in the J0 replay (exploratory).

Arms: B3, B7, MAKL, the distance switch dswitch(d_th) (scan first if d < d_th, otherwise the older head first)
and MAKL with a fixed budget. Used to choose the J3 thresholds.

usage: dswitch_replay.py TRACES_DIR OUT_DIR
"""
import csv, json, statistics, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j0_explore_v2 as J0
J, R, H = J0.J, J0.R, J0.H
SCAN, CAM = J.SCAN, J.CAM

class DSwitch(J.JQdiscLink):
    """Depth-1 queues, probes first; d < d_th: scan first (B7); else older head first (keep-last, as B3)."""
    d_th = 1.0
    def _choose(self):
        if self._head(J.PROBE) is not None:
            self.decisions['probe'] += 1
            return self._take(J.PROBE)
        hs, hm = self._head(SCAN), self._head(CAM)
        if self.state['d'] < self.d_th:
            order = [SCAN, CAM]
        elif hs is not None and hm is not None:
            cap = lambda m: self.capture_us.get(m, self.records[m].created)
            order = [SCAN, CAM] if cap(hs) <= cap(hm) else [CAM, SCAN]
        else:
            order = [SCAN, CAM]
        for s in order + self._others():
            if self._head(s) is not None:
                self.decisions[s] += 1
                return self._take(s)
        return None

def make(arm):
    kw = dict(capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY)
    if arm.startswith('DSW'):
        L = J.make_jlink('B7', **kw)
        L.__class__ = type('DSW', (DSwitch,), {'d_th': float(arm[4:])})
        return L
    if arm.startswith('MAKLfix'):
        return J.make_jlink('MAKL', params={'beta_override': float(arm[8:])}, **kw)
    return J.make_jlink(arm, **kw)

ARMS = ['B3', 'B7', 'MAKL'] + [f'DSW_{x}' for x in (0.6, 0.8, 1.0, 1.2, 1.5, 2.0)] + [f'MAKLfix_{x}' for x in (0.4, 0.6, 0.8, 1.0, 1.5)]
CELLS = [('Q1', 0.18, 25), ('Q2', 0.13, 25), ('Q1', 0.18, 8), ('Q2', 0.13, 8), ('Q1', 0.13, 25), ('Q1', 0.21, 25)]

def one(run, arm, cap, dur):
    L = make(arm)
    states = R.state_series(run['grid'], 'raw360')
    res = R.replay(run['tr'], L, capacity_mbps=cap, duration_s=dur, offers='live', states=states,
                   horizon_after_s=max(5.0, 25.0 - dur + 2.0))
    v = R.violation(res, run['grid'])
    T = res['T']; a, b = T, T + 25_000_000
    cam = [(now, st) for s, now, st, _, _ in res['recv'] if s == 'camera' and a <= now < b]
    F = sum(1 for now, st in cam if now - st <= 1_000_000) / 25.0
    return dict(trace=run['tid'], profile=run['profile'], arm=arm, cap=cap, dur=dur, viol=v['viol_s'], a95=v['a_nav_p95_s'], F=round(F, 3))

def main():
    tr, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    e5 = J0.load_runs(tr, 'E5', lambda tid, d: d.get('level') == 'C0' and abs(float(d.get('speed') or 0) - 0.5) < 1e-9
                      and d.get('arm') in ('F-N1', 'F-N2') and d.get('status') == 'SUCCEEDED')
    e7 = J0.load_runs(tr, 'E7', lambda tid, d: d.get('level') == 'C0' and abs(float(d.get('speed') or 0) - 0.5) < 1e-9
                      and d.get('status') == 'SUCCEEDED')
    rows = []
    for run in e5:
        for prof, cap, dur in CELLS:
            if run['profile'] != prof:
                continue
            for arm in ARMS:
                rows.append(one(run, arm, cap, dur))
    for run in e7:                                    # E6 traces, main Q2 cell only
        for arm in ARMS:
            r = one(run, arm, 0.13, 25); r['profile'] = 'Q2e6'; rows.append(r)
    with open(out / 'ABL_ROWS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    summ = {}
    for prof, cap, dur in CELLS + [('Q2e6', 0.13, 25)]:
        key = f'{prof}_{cap}_{dur}s'; summ[key] = {}
        for arm in ARMS:
            rr = [r for r in rows if r['profile'] == prof and r['cap'] == cap and r['dur'] == dur and r['arm'] == arm]
            if rr:
                summ[key][arm] = {k: round(statistics.mean(r[k] for r in rr), 3) for k in ('viol', 'F', 'a95')} | {'n': len(rr)}
    (out / 'ABL_SUMMARY.json').write_text(json.dumps(summ, indent=1))
    for key, c in summ.items():
        print('CELL', key, 'n', c['MAKL']['n'])
        for arm, x in c.items():
            print(f"   {arm:12s} viol={x['viol']:6.2f}  F={x['F']:5.2f}  a95={x['a95']:5.2f}")

if __name__ == '__main__':
    main()
