#!/usr/bin/env python3
"""J5b pre-specified analysis (J5B_PROTOCOL_FREEZE.md section 7). Usage: j5b_analysis.py RESULTS_DIR OUT_DIR [J5_CELLS.csv]
Per-replay measures are those of the frozen J5 analysis (j5_analysis.replay_measures). Counted: Q1 blocks (modes A and
NP at 0.75 c), pairs = traces rep 1-5 in the same session. Demo: Q2 NP blocks (exploratory, no test).
Outputs: J5B_REPLAYS.csv, J5B_CELLS.csv, J5B_TESTS.csv, J5B_VERDICTS.json, J5B_PLATFORM.json, J5B_PILOTS.json."""
import csv, json, math, statistics as st, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5b_common as B
from j5b_common import C
import j5_analysis as AN

J_F = 0.4; J_VIOL = 0.8; BACKLOG_B = 3000; NET_P99_MS = 50.0


def num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def main():
    base = Path(sys.argv[1]); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    j5c = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    S = json.loads((base / 'STATE_SENDER.json').read_text())
    part = {r['rid']: r['part'] for r in C.matrix()}
    use = []
    for b, Bk in sorted(S.get('blocks', {}).items(), key=lambda kv: int(kv[0])):
        if Bk.get('status') == 'valid':
            use += [a for a in Bk['attempts'] if a.get('valid')][-1]['replays']
    R = []
    for rid in use:
        m = AN.replay_measures(base, rid)
        m['mode'] = 'NP75' if m['mode'] == 'NP' else m['mode']
        m['part'] = part.get(rid[:-3] if rid.endswith('_a2') else rid, '?')
        R.append(m)
    keys = []
    for r in R:
        keys += [k for k in r if k not in keys]
    with open(out / 'J5B_REPLAYS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    cells = []
    for cell, mode in (('Q1', 'A'), ('Q1', 'NP75'), ('Q2', 'NP75')):
        for arm in C.ARMS:
            rr = [r for r in R if r['cell'] == cell and r['mode'] == mode and r['arm'] == arm]
            if not rr:
                continue
            x = {'cell': cell, 'mode': mode, 'arm': arm, 'part': rr[0]['part'], 'n': len(rr)}
            for k in keys:
                z = [r[k] for r in rr if num(r.get(k))]
                if z and k not in ('rep',):
                    x[k] = round(st.mean(z), 4); x[k + '_sd'] = round(st.stdev(z), 4) if len(z) > 1 else None
            cells.append(x)
    ck = []
    for x in cells:
        ck += [k for k in x if k not in ck]
    with open(out / 'J5B_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)

    by = lambda mode, arm, k, cell='Q1': {r['rep']: r[k] for r in R if r['cell'] == cell and r['mode'] == mode
                                          and r['arm'] == arm and num(r.get(k))}
    T = {}
    z = list(by('A', 'B7', 'anav_p95_s').values())
    J_age = round(max(0.1, (max(z) - min(z)) if len(z) >= 2 else 0.0), 4)
    # K1 kernel queue short, every arm: median backlog in W < 3,000 B in >= 4 of 5 replays
    k1 = True
    for arm in C.ARMS:
        med = by('NP75', arm, 'backlog_med_B'); sh = by('NP75', arm, 'backlog_share_gt_3000B')
        n_ok = sum(1 for v in med.values() if v < BACKLOG_B)
        T[f'K1_{arm}_median_backlog_lt_3000B'] = {'n': len(med), 'n_ok': n_ok, 'median_B': med,
                                                   'share_gt_3000B_mean': round(st.mean(sh.values()), 4) if sh else None,
                                                   'expected_share_lt': 0.10, 'holds': len(med) >= 4 and n_ok >= 4}
        k1 &= T[f'K1_{arm}_median_backlog_lt_3000B']['holds']
    # K2 network delay small for MAKL
    v = by('NP75', 'MAKL', 'nav_minus_link_p99_ms')
    T['K2_MAKL_net_p99_le_50ms'] = {'per_rep_ms': v, 'median_ms': round(st.median(v.values()), 3) if v else None,
                                    'holds': bool(v) and st.median(v.values()) <= NET_P99_MS}
    k2 = T['K2_MAKL_net_p99_le_50ms']['holds']
    # predictions: frozen offline replay of the same traces (predictions/J5B_PREDICTIONS.json)
    P = json.loads((B.HERE / 'predictions' / 'J5B_PREDICTIONS.json').read_text())['cells']
    pr = lambda cell, mode, arm, k: {r['rep']: r[k] for r in P[f'{cell}|{mode}|{arm}']}
    # R1 primary: MAKL p95 a_nav, NP75 - A (paired) <= predicted (offline) NP75 - A + J_age   (>= 4 pairs)
    r1 = None
    for arm in ('MAKL', 'B7', 'B3', 'B0'):
        a, b = by('NP75', arm, 'anav_p95_s'), by('A', arm, 'anav_p95_s')
        reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
        pa, pb = pr('Q1', 'NP75', arm, 'alink_p95_s'), pr('Q1', 'A', arm, 'alink_p95_s')
        pd = st.mean(pa[k] - pb[k] for k in C.REPS)
        t = {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d], 'mean_diff': round(st.mean(d), 4) if d else None,
             'predicted_mean_diff': round(pd, 4), 'J_age': J_age, 'limit_s': round(pd + J_age, 4),
             'holds': len(reps) >= 4 and st.mean(d) <= pd + J_age}
        T[f'R1_{arm}_NP75_minus_A_anav' + ('' if arm == 'MAKL' else '_reported')] = t
        if arm == 'MAKL':
            r1 = t['holds']
    # K3 fidelity: in NP75 every arm's measured p95 a_nav is within J_age of the predicted link age (mean over traces)
    k3 = True
    for arm in C.ARMS:
        m = by('NP75', arm, 'anav_p95_s'); pa = pr('Q1', 'NP75', arm, 'alink_p95_s')
        d = [m[k] - pa[k] for k in sorted(m)]
        h = len(d) >= 4 and st.mean(d) <= J_age
        T[f'K3_{arm}_NP75_anav_minus_predicted'] = {'diffs': [round(x, 4) for x in d], 'mean': round(st.mean(d), 4) if d else None,
                                                    'J_age': J_age, 'holds': h}
        k3 &= h
    # R2: MAKL violation NP75 not clearly greater than A
    t = AN.clearly_greater(by('NP75', 'MAKL', 'viol_s'), by('A', 'MAKL', 'viol_s'), J_VIOL)
    T['R2_MAKL_viol_NP75_gt_A'] = t
    r2 = t['evaluable'] and not t['clearly_greater']
    # reported: F and violation, measured against the prediction, every arm and mode (Q1) and the Q2 demo
    for cell, mode in (('Q1', 'A'), ('Q1', 'NP75'), ('Q2', 'NP75')):
        for arm in C.ARMS:
            f = by(mode, arm, 'F_fps', cell); vv = by(mode, arm, 'viol_s', cell); an = by(mode, arm, 'anav_p95_s', cell)
            pf, pv, pa = pr(cell, mode, arm, 'F_fps'), pr(cell, mode, arm, 'viol_s'), pr(cell, mode, arm, 'alink_p95_s')
            mean = lambda x: round(st.mean(x.values()), 4) if x else None
            T[f'M_{cell}_{mode}_{arm}_measured_vs_predicted'] = {
                'anav_p95': mean(an), 'pred_alink_p95': mean(pa), 'F': mean(f), 'pred_F': mean(pf),
                'viol': mean(vv), 'pred_viol': mean(pv)}
    T['F_MAKL_minus_B7_NP75_reported'] = AN.clearly_greater(by('NP75', 'MAKL', 'F_fps'), by('NP75', 'B7', 'F_fps'), J_F)
    # reported: mode A reproduces J5 mode A (cell means)
    if j5c and j5c.is_file():
        J5 = {(r['cell'], r['mode'], r['arm']): r for r in csv.DictReader(open(j5c))}
        for arm in C.ARMS:
            x = next((c for c in cells if c['cell'] == 'Q1' and c['mode'] == 'A' and c['arm'] == arm), None)
            y = J5.get(('Q1', 'A', arm)); y90 = J5.get(('Q1', 'NP', arm))
            z = next((c for c in cells if c['cell'] == 'Q1' and c['mode'] == 'NP75' and c['arm'] == arm), None)
            if x and y:
                T[f'R0_{arm}_A_vs_J5'] = {'anav_p95_j5b': x.get('anav_p95_s'), 'anav_p95_j5': float(y['anav_p95_s']),
                                          'F_j5b': x.get('F_fps'), 'F_j5': float(y['F_fps']),
                                          'within_0.1s_0.4fps': abs(x.get('anav_p95_s', 99) - float(y['anav_p95_s'])) <= 0.1
                                          and abs(x.get('F_fps', 99) - float(y['F_fps'])) <= 0.4}
            if z and y90:
                T[f'X_{arm}_NP90_J5_vs_NP75_exploratory'] = {'anav_p95_np90': float(y90['anav_p95_s']), 'anav_p95_np75': z.get('anav_p95_s'),
                                                            'F_np90': float(y90['F_fps']), 'F_np75': z.get('F_fps'),
                                                            'viol_np90': float(y90['viol_s']), 'viol_np75': z.get('viol_s')}
    V = {'n_counted': sum(1 for r in R if r['part'] == 'counted'), 'n_demo': sum(1 for r in R if r['part'] == 'demo'),
         'blocks': {b: x.get('status') for b, x in S.get('blocks', {}).items()}, 'pace': C.PACE, 'J_age': J_age,
         'K1_kernel_queue_short': k1, 'K2_net_delay_small': k2, 'K3_follows_link_model': k3,
         'R1_age_restored': r1, 'R2_violation_not_worse': r2,
         'verdict': 'CONTROL_RESTORED' if (r1 and r2) else 'CONTROL_NOT_RESTORED'}
    plat = {}
    for mode in ('A', 'NP75'):
        rr = [r for r in R if r['cell'] == 'Q1' and r['mode'] == mode]
        plat[mode] = {k: AN.pct([r[k] for r in rr if num(r.get(k))], .5) for k in
                      ('ow_p50_ms', 'ow_p99_ms', 'nav_minus_link_p50_ms', 'nav_minus_link_p99_ms', 'pub_late_p99_ms')}
    mk = [r for r in R if r['arm'] == 'MAKL']
    plat['makl_decision_thread_p50_ms'] = AN.pct([r.get('makl_dec_thread_p50_ms') for r in mk if r.get('makl_dec_thread_p50_ms') is not None], .5)
    plat['makl_decision_thread_p99_ms_max'] = max((r.get('makl_dec_thread_p99_ms') or 0) for r in mk) if mk else None
    plat['note'] = 'Q1 only; medians over replays of the per-replay statistics'
    (out / 'J5B_PLATFORM.json').write_text(json.dumps(plat, indent=1) + '\n')
    (out / 'J5B_VERDICTS.json').write_text(json.dumps(V, indent=1) + '\n')
    with open(out / 'J5B_TESTS.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['test', 'result_json'])
        for k, v in T.items():
            w.writerow([k, json.dumps(v)])
    if S.get('pilots'):
        (out / 'J5B_PILOTS.json').write_text(json.dumps({k: v for k, v in S['pilots'].items()
                                                         if k in ('checks', 'decision_time', 'iperf', 'passed', 'background')},
                                                        indent=1, default=str) + '\n')
    print('J5B_VERDICTS', json.dumps({k: v for k, v in V.items() if k != 'blocks'}))
    print('J5B_PLATFORM', json.dumps(plat))


if __name__ == '__main__':
    main()
