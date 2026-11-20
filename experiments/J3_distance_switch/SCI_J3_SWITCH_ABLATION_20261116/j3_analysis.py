#!/usr/bin/env python3
"""J3 pre-specified analysis (J3_PROTOCOL_FREEZE.md sections 8-10). Usage: j3_analysis.py STATE_JSON OUT_DIR
Per-run rows come from the frozen J1 v2 analysis (j1_analysis.per_run). Paired rule of J1 (j1_analysis.clearly_greater):
A is CLEARLY GREATER than B if at least 4 of the 5 pairs exist, every paired difference A - B > 0 and the mean
difference > J. Thresholds are FIXED from J1 (no C0 cell in J3): J_viol = 0.8 s (Q1 cells) / 0.7 s (Q2 cell);
J_F = 0.4 fps (Q1) / 0.2 fps (Q2). Outputs: J3_RUNS.csv, J3_CELLS.csv, J3_TESTS.csv, J3_VERDICTS.json (frozen verdicts),
J3_EXPLORATORY.json and FIG_J3_PARETO.png (exploratory)."""
import csv, json, math, statistics as st, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
for p in (str(HERE), str(J1)):
    if p not in sys.path:
        sys.path.insert(0, p)
import j1_analysis as A1

ARMS = ('B7', 'MAKL', 'DSW_0.8', 'DSW_1.0', 'DSW_1.2')
CELLS = ('LB-Q1', 'LC-Q1', 'LC-Q2')
VIOL = A1.VIOL; F = A1.F
J_VIOL = {'LB-Q1': 0.8, 'LC-Q1': 0.8, 'LC-Q2': 0.7}
J_F = {'LB-Q1': 0.4, 'LC-Q1': 0.4, 'LC-Q2': 0.2}
J1_REF = {  # J1 v2 frozen results, 25 s cells, n = 5: (viol mean, viol SD, F mean, F SD)
    'LC-Q2': {'B7': (1.62, 0.1924, 0.408, 0.0335), 'MAKL': (1.96, 0.532, 0.88, 0.0283)},
    'LB-Q1': {'B7': (1.3, 0.1581, 2.28, 0.04), 'MAKL': (1.42, 0.3114, 3.216, 0.0727)}}
cg = A1.clearly_greater


def main():
    S = json.loads(Path(sys.argv[1]).read_text()); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
    R = []
    for t, r in sorted(S['done'].items(), key=lambda kv: kv[1].get('order', 0)):
        row = A1.per_run(t, r)
        d = r.get('j3_dsw') or {}
        row.update({'dsw_' + k: v for k, v in d.items() if not isinstance(v, (list, dict))})
        R.append(row)
    keys = sorted({k for r in R for k in r}, key=lambda k: (k not in ('trial_id', 'config', 'cell', 'rep', 'status'), k))
    with open(OUT / 'J3_RUNS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    vals = lambda c, a, m: A1.by_rep(R, c, a, m)
    cells = []; CM = {}
    for c in CELLS:
        for a in ARMS:
            rr = [r for r in R if r['cell'] == c and r['config'] == a]
            if not rr:
                continue
            x = {'cell': c, 'config': a, 'n': len(rr), 'succeeded': sum(r['status'] == 'SUCCEEDED' for r in rr),
                 'robot_moving_contacts': sum(r['robot_moving_contact'] for r in rr)}
            for m in (VIOL, F, 'V', 'a_nav_p95_s', 'a_nav_p95_clipped_s', 'P1_s', 'W_min_clearance_m', 'makl_cam_share_W',
                      'dsw_scan_first_share_W'):
                v = [r.get(m) for r in rr if isinstance(r.get(m), (int, float)) and math.isfinite(r.get(m))]
                if v:
                    x[m] = round(st.mean(v), 4); x[m + '_sd'] = round(st.stdev(v), 4) if len(v) > 1 else None
                    x[m + '_range'] = round(max(v) - min(v), 4)
                    lo, hi = A1.boot_ci(v); x[m + '_ci_lo'] = round(lo, 4) if lo is not None else None
                    x[m + '_ci_hi'] = round(hi, 4) if hi is not None else None
            cells.append(x); CM[(c, a)] = x
    ck = []
    for x in cells:
        for k in x:
            if k not in ck:
                ck.append(k)
    with open(OUT / 'J3_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)
    T = {}
    # H1: new capacity cell, MAKL vs B7
    T['H1a_LC-Q1_MAKL_gt_B7_F'] = cg(vals('LC-Q1', 'MAKL', F), vals('LC-Q1', 'B7', F), J_F['LC-Q1'])
    T['H1b_LC-Q1_MAKL_gt_B7_viol'] = cg(vals('LC-Q1', 'MAKL', VIOL), vals('LC-Q1', 'B7', VIOL), J_VIOL['LC-Q1'])
    H1 = T['H1a_LC-Q1_MAKL_gt_B7_F']['clearly_greater'] and T['H1b_LC-Q1_MAKL_gt_B7_viol']['evaluable'] \
        and not T['H1b_LC-Q1_MAKL_gt_B7_viol']['clearly_greater']
    # H2: mistuned switches lose somewhere
    T['H2a_LC-Q1_DSW0.8_gt_MAKL_viol'] = cg(vals('LC-Q1', 'DSW_0.8', VIOL), vals('LC-Q1', 'MAKL', VIOL), J_VIOL['LC-Q1'])
    H2a = T['H2a_LC-Q1_DSW0.8_gt_MAKL_viol']['clearly_greater']
    h2b = {}
    for c in ('LB-Q1', 'LC-Q1'):
        t = T[f'H2b_{c}_MAKL_gt_DSW1.2_F'] = cg(vals(c, 'MAKL', F), vals(c, 'DSW_1.2', F), J_F[c])
        h2b[c] = t['clearly_greater']
    H2b = any(h2b.values())
    # H3: tuned switch DSW_1.0 is not clearly better than MAKL on either metric in any cell
    h3 = {}
    for c in CELLS:
        tv = T[f'H3_{c}_MAKL_gt_DSW1.0_viol'] = cg(vals(c, 'MAKL', VIOL), vals(c, 'DSW_1.0', VIOL), J_VIOL[c])
        tf = T[f'H3_{c}_DSW1.0_gt_MAKL_F'] = cg(vals(c, 'DSW_1.0', F), vals(c, 'MAKL', F), J_F[c])
        h3[c] = tv['evaluable'] and tf['evaluable'] and not tv['clearly_greater'] and not tf['clearly_greater']
    H3 = all(h3.values()) and len(h3) == 3
    rm = sum(r['robot_moving_contact'] for r in R)
    Rv = {}
    for c in ('LB-Q1', 'LC-Q2'):
        for a in ('B7', 'MAKL'):
            x = CM.get((c, a), {}); vm, vs, fm, fs = J1_REF[c][a]
            tv = max(J_VIOL[c], 2 * vs); tf = max(J_F[c], 2 * fs)
            Rv[f'{c}|{a}'] = {'viol_J3': x.get(VIOL), 'viol_J1': vm, 'tol_viol': round(tv, 3), 'F_J3': x.get(F), 'F_J1': fm,
                              'tol_F': round(tf, 3), 'ok': x.get(VIOL) is not None and abs(x[VIOL] - vm) <= tv and abs(x[F] - fm) <= tf}
    V = {'H1_LC-Q1_MAKL_more_frames_than_B7_and_violation_not_clearly_larger': H1,
         'H2a_DSW0.8_violation_clearly_larger_than_MAKL_at_LC-Q1': H2a,
         'H2b_MAKL_frames_clearly_larger_than_DSW1.2_in_a_Q1_cell': H2b, 'H2b_by_cell': h2b,
         'H3_tuned_DSW1.0_not_clearly_better_than_MAKL_on_either_metric_all_cells': H3, 'H3_by_cell': h3,
         'H4_no_robot_moving_contact': rm == 0, 'robot_moving_contacts': rm,
         'R_B7_MAKL_reproduce_J1': all(v['ok'] for v in Rv.values()), 'R_detail': Rv}
    if not H3:
        V['outcome'] = 'TUNED_SWITCH_BETTER'
    elif H2a or H2b:
        V['outcome'] = 'SUPPORTED'
    else:
        V['outcome'] = 'THRESHOLD_INSENSITIVE'
    V['n_runs'] = len(R); V['status_counts'] = {s: sum(r['status'] == s for r in R) for s in sorted({r['status'] for r in R})}
    (OUT / 'J3_VERDICTS.json').write_text(json.dumps(V, indent=1, default=str) + '\n')
    with open(OUT / 'J3_TESTS.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['test', 'result_json'])
        for k, v in T.items():
            w.writerow([k, json.dumps(v)])
    # ---------------- exploratory ----------------
    X = {}
    for c in CELLS:
        mk = CM.get((c, 'MAKL'), {})
        dom = [a for a in ARMS if a != 'MAKL' and CM.get((c, a)) and mk and CM[(c, a)][VIOL] <= mk[VIOL] and CM[(c, a)][F] >= mk[F]
               and (CM[(c, a)][VIOL] < mk[VIOL] or CM[(c, a)][F] > mk[F])]
        X[c] = {'MAKL_dominated_by_means': dom,
                'MAKL_minus_DSW1.0': {m: round(mk.get(m, float('nan')) - CM.get((c, 'DSW_1.0'), {}).get(m, float('nan')), 4) for m in (VIOL, F)} if mk else None}
    (OUT / 'J3_EXPLORATORY.json').write_text(json.dumps(X, indent=1) + '\n')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
        for i, c in enumerate(CELLS):
            ax = axs[i]
            for a in ARMS:
                x = CM.get((c, a))
                if x and x.get(VIOL) is not None:
                    ax.plot(x[VIOL], x[F], 'o', color='C3' if a == 'MAKL' else ('C0' if a == 'B7' else '0.4'))
                    ax.annotate(a, (x[VIOL], x[F]), textcoords='offset points', xytext=(3, 3), fontsize=8)
            ax.set_title(c); ax.set_xlabel('violation (s)'); ax.set_ylabel('camera F (fps)')
        fig.tight_layout(); fig.savefig(OUT / 'FIG_J3_PARETO.png', dpi=150)
    except Exception as ex:
        (OUT / 'FIG_ERROR.txt').write_text(repr(ex) + '\n')
    print('J3_VERDICTS', json.dumps({k: v for k, v in V.items() if not k.endswith('_by_cell') and k != 'R_detail'}))
    print('J3_OUTCOME', V['outcome'])


if __name__ == '__main__':
    main()
