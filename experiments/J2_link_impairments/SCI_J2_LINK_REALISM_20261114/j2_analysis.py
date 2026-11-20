#!/usr/bin/env python3
"""J2 pre-specified analysis (J2_PROTOCOL_FREEZE.md sections 7-9). Usage: j2_analysis.py STATE_JSON OUT_DIR
Per-run rows come from the frozen J1 v2 analysis (j1_analysis.per_run). Outputs: J2_RUNS.csv, J2_CELLS.csv,
J2_TESTS.csv, J2_VERDICTS.json (frozen verdicts first), J2_RANKS.csv and FIG_J2_PARETO.png (exploratory).
Thresholds are FIXED from J1 (no C0 cell in J2): J_viol = 0.7 s (Q2) / 0.8 s (Q1) (= the J1 non-inferiority margins
and superiority J), J_F = 0.2 fps (Q2) / 0.4 fps (Q1) (J1). Paired rule with 3 pairs (all 3 present):
A CLEARLY GREATER than B if all 3 paired differences > 0 and the mean difference > J."""
import csv, json, math, statistics as st, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
for p in (str(HERE), str(J1)):
    if p not in sys.path:
        sys.path.insert(0, p)
import j1_analysis as A1

ARMS = ('B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
CONDS = ('none', 'jit10', 'jit50', 'loss1', 'loss5', 'ge', 'capsq')
IMPAIRED = CONDS[1:]
PROFILES = ('LC-Q2', 'LB-Q1')
VIOL = A1.VIOL; F = A1.F
J_VIOL = {'LC-Q2': 0.7, 'LB-Q1': 0.8}
J_F = {'LC-Q2': 0.2, 'LB-Q1': 0.4}
J1_REF = {  # J1 v2 (frozen results), 25 s cells, n = 5: (viol mean, viol SD, F mean, F SD)
    'LC-Q2': {'B2': (1.28, 0.2775, 0.016, 0.0219), 'B3': (2.4, 0.324, 1.56, 0.0283), 'B4': (3.78, 0.9203, 1.568, 0.0522),
              'B5': (2.5, 0.6964, 1.584, 0.0537), 'B7': (1.62, 0.1924, 0.408, 0.0335), 'MAKL': (1.96, 0.532, 0.88, 0.0283)},
    'LB-Q1': {'B2': (1.18, 0.3347, 0.216, 0.0219), 'B3': (2.18, 1.2071, 2.968, 0.5264), 'B4': (2.36, 0.4278, 2.712, 0.0593),
              'B5': (1.84, 0.6189, 3.568, 0.5963), 'B7': (1.3, 0.1581, 2.28, 0.04), 'MAKL': (1.42, 0.3114, 3.216, 0.0727)}}


def cg3(a, b, j):
    reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
    r = {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d], 'mean_diff': round(st.mean(d), 4) if d else None, 'J': j,
         'evaluable': len(reps) == 3}
    r['clearly_greater'] = bool(r['evaluable'] and all(x > 0 for x in d) and st.mean(d) > j)
    return r


def kendall_tau(x, y):
    n = len(x); c = d = 0
    for i in range(n):
        for k in range(i + 1, n):
            s = (x[i] - x[k]) * (y[i] - y[k])
            c += s > 0; d += s < 0
    return (c - d) / (n * (n - 1) / 2)


def main():
    S = json.loads(Path(sys.argv[1]).read_text()); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
    R = []
    for t, r in sorted(S['done'].items(), key=lambda kv: kv[1].get('order', 0)):
        row = A1.per_run(t, r)
        row['condition'] = r.get('j2_condition'); row['profile_cell'] = r['j1_cell'].split('|')[0]
        R.append(row)
    keys = sorted({k for r in R for k in r}, key=lambda k: (k not in ('trial_id', 'config', 'cell', 'condition', 'rep', 'status'), k))
    with open(OUT / 'J2_RUNS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    def vals(pc, cond, a, m):
        return {r['rep']: r[m] for r in R if r['profile_cell'] == pc and r['condition'] == cond and r['config'] == a
                and isinstance(r.get(m), (int, float)) and math.isfinite(r[m])}
    cells = []; CM = {}
    for pc in PROFILES:
        for cond in CONDS:
            for a in ARMS:
                rr = [r for r in R if r['profile_cell'] == pc and r['condition'] == cond and r['config'] == a]
                if not rr:
                    continue
                x = {'profile_cell': pc, 'condition': cond, 'config': a, 'n': len(rr), 'succeeded': sum(r['status'] == 'SUCCEEDED' for r in rr),
                     'robot_moving_contacts': sum(r['robot_moving_contact'] for r in rr)}
                for m in (VIOL, F, 'V', 'a_nav_p95_s', 'a_nav_p95_clipped_s', 'P1_s', 'W_min_clearance_m', 'L_scan_channel_loss',
                          'L_camera_channel_loss', 'j2_loss_rate_packets', 'j2_mean_abs_jitter_ms', 'j2_cap_changes', 'makl_cam_share_W'):
                    v = [r.get(m) for r in rr if isinstance(r.get(m), (int, float)) and math.isfinite(r.get(m))]
                    if v:
                        x[m] = round(st.mean(v), 4); x[m + '_sd'] = round(st.stdev(v), 4) if len(v) > 1 else None
                        x[m + '_range'] = round(max(v) - min(v), 4)          # run-to-run stability (descriptive)
                cells.append(x); CM[(pc, cond, a)] = x
    ck = []
    for x in cells:
        for k in x:
            if k not in ck:
                ck.append(k)
    with open(OUT / 'J2_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)
    T = {}; h1 = {}; h2 = {}; h3 = {}
    for pc in PROFILES:
        for cond in IMPAIRED:
            key = f'{pc}|{cond}'
            t1 = T[f'H1_{key}_MAKL_gt_B7_viol'] = cg3(vals(pc, cond, 'MAKL', VIOL), vals(pc, cond, 'B7', VIOL), J_VIOL[pc])
            h1[key] = t1['evaluable'] and not t1['clearly_greater']
            t2 = T[f'H2_{key}_MAKL_gt_B7_F'] = cg3(vals(pc, cond, 'MAKL', F), vals(pc, cond, 'B7', F), J_F[pc])
            h2[key] = t2['clearly_greater']
            mk = CM.get((pc, cond, 'MAKL'), {}); dom = []
            for a in ARMS:
                x = CM.get((pc, cond, a), {})
                if a != 'MAKL' and x.get(VIOL) is not None and mk.get(VIOL) is not None and \
                        x[VIOL] < mk[VIOL] - J_VIOL[pc] and x[F] > mk[F] + J_F[pc]:
                    dom.append(a)
            h3[key] = bool(mk) and not dom
            T[f'H3_{key}_MAKL_clearly_dominated_by'] = {'arms': dom, 'evaluable': bool(mk)}
    rm = sum(r['robot_moving_contact'] for r in R)
    Rv = {}
    for pc in PROFILES:
        for a in ARMS:
            x = CM.get((pc, 'none', a), {}); vm, vs, fm, fs = J1_REF[pc][a]
            tv = max(J_VIOL[pc], 2 * vs); tf = max(J_F[pc], 2 * fs)
            Rv[f'{pc}|{a}'] = {'viol_J2': x.get(VIOL), 'viol_J1': vm, 'tol_viol': round(tv, 3), 'F_J2': x.get(F), 'F_J1': fm, 'tol_F': round(tf, 3),
                               'ok': x.get(VIOL) is not None and abs(x[VIOL] - vm) <= tv and abs(x[F] - fm) <= tf}
    V = {'H1_noninferior_violation_vs_B7_all_impaired_cells': all(h1.values()) and len(h1) == 12, 'H1_by_cell': h1,
         'H2_camera_above_B7_all_impaired_cells': all(h2.values()) and len(h2) == 12, 'H2_by_cell': h2,
         'H2_cells_passed': sum(h2.values()),
         'H3_MAKL_not_clearly_dominated_all_impaired_cells': all(h3.values()) and len(h3) == 12, 'H3_by_cell': h3,
         'H4_no_robot_moving_contact': rm == 0, 'robot_moving_contacts': rm,
         'R_none_reproduces_J1': all(v['ok'] for v in Rv.values()), 'R_detail': Rv}
    if V['H1_noninferior_violation_vs_B7_all_impaired_cells'] and V['H2_camera_above_B7_all_impaired_cells'] \
            and V['H3_MAKL_not_clearly_dominated_all_impaired_cells']:
        V['outcome'] = 'ROBUST'
    elif V['H1_noninferior_violation_vs_B7_all_impaired_cells'] and V['H3_MAKL_not_clearly_dominated_all_impaired_cells']:
        V['outcome'] = 'PARTIALLY_ROBUST'
    else:
        V['outcome'] = 'NOT_ROBUST'
    V['n_runs'] = len(R); V['status_counts'] = {s: sum(r['status'] == s for r in R) for s in sorted({r['status'] for r in R})}
    (OUT / 'J2_VERDICTS.json').write_text(json.dumps(V, indent=1, default=str) + '\n')
    with open(OUT / 'J2_TESTS.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['test', 'result_json'])
        for k, v in T.items():
            w.writerow([k, json.dumps(v)])
    # exploratory: Kendall tau of arm ranks vs 'none' (by mean violation and by mean F)
    ranks = []
    for pc in PROFILES:
        base_v = [CM.get((pc, 'none', a), {}).get(VIOL) for a in ARMS]; base_f = [CM.get((pc, 'none', a), {}).get(F) for a in ARMS]
        for cond in IMPAIRED:
            cv = [CM.get((pc, cond, a), {}).get(VIOL) for a in ARMS]; cf = [CM.get((pc, cond, a), {}).get(F) for a in ARMS]
            if None in base_v + cv + base_f + cf:
                continue
            ranks.append({'profile_cell': pc, 'condition': cond, 'tau_violation': round(kendall_tau(base_v, cv), 3),
                          'tau_camera_F': round(kendall_tau(base_f, cf), 3)})
    if ranks:
        with open(OUT / 'J2_RANKS.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(ranks[0])); w.writeheader(); w.writerows(ranks)
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 7, figsize=(28, 8))
        for i, pc in enumerate(PROFILES):
            for k, cond in enumerate(CONDS):
                ax = axs[i][k]
                for a in ARMS:
                    x = CM.get((pc, cond, a))
                    if x and x.get(VIOL) is not None:
                        ax.plot(x[VIOL], x[F], 'o', color='C3' if a == 'MAKL' else ('C0' if a == 'B7' else '0.4'))
                        ax.annotate(a, (x[VIOL], x[F]), textcoords='offset points', xytext=(3, 3), fontsize=7)
                ax.set_title(f'{pc} {cond}', fontsize=9); ax.set_xlabel('violation (s)', fontsize=8); ax.set_ylabel('camera F (fps)', fontsize=8)
        fig.tight_layout(); fig.savefig(OUT / 'FIG_J2_PARETO.png', dpi=150)
    except Exception as ex:
        (OUT / 'FIG_ERROR.txt').write_text(repr(ex) + '\n')
    print('J2_VERDICTS', json.dumps({k: v for k, v in V.items() if not k.endswith('_by_cell') and k != 'R_detail'}))
    print('J2_OUTCOME', V['outcome'])


if __name__ == '__main__':
    main()
