#!/usr/bin/env python3
"""J1 pre-specified analysis (J1_PROTOCOL_FREEZE.md sections 7-9). Usage: j1_analysis.py STATE_JSON OUT_DIR
Outputs: J1_RUNS.csv, J1_CELLS.csv (mean, SD, 95 % bootstrap CI per arm x cell), J1_PAIRED.csv (all frozen paired tests),
J1_VERDICTS.json (frozen verdicts first), J1_CHANGES_VS_B0_B7.csv (exploratory), FIG_J1_PARETO.png (exploratory).
Paired 'clear difference' rule (frozen): pairs = same start shift (rep); A is CLEARLY GREATER than B if >= 4 pairs exist,
every paired difference A - B > 0 and the mean difference > J. J (per metric and camera profile) = max(floor, range of the
metric over all C0 runs of that profile). Floors: violation 0.5 s (superiority), camera F 0.2 fps, P1 1.0 s,
clearance 0.02 m. Non-inferiority margin for MAKL vs B7 violation: max(0.3 s, range). No p-values."""
import csv, json, math, random, statistics as st, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
E5 = WS / 'SCI_E5_PED_SPEED_20261101'
for p in (str(HERE), str(E5), str(E3B), str(E3B / 'analysis')):
    if p not in sys.path:
        sys.path.insert(0, p)
import j1_metrics as M

ARMS = ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
CELLS = ('C0-Q2', 'C0-Q1', 'LC-Q2-25s', 'LC-Q2-8s', 'LB-Q1-25s', 'LB-Q1-8s')
CONG = ('LC-Q2-25s', 'LC-Q2-8s', 'LB-Q1-25s', 'LB-Q1-8s')
C0_OF = {'LC-Q2-25s': 'C0-Q2', 'LC-Q2-8s': 'C0-Q2', 'LB-Q1-25s': 'C0-Q1', 'LB-Q1-8s': 'C0-Q1'}
CAP = {'LC-Q2-25s': 130_000, 'LC-Q2-8s': 130_000, 'LB-Q1-25s': 180_000, 'LB-Q1-8s': 180_000}
VIOL = 'W_viol_anav_unom_b0.2_s'; CLR = 'W_min_clearance_m'; F = 'F_fps'; P1 = 'P1_s'
FLOOR = {VIOL: 0.5, F: 0.2, P1: 1.0, CLR: 0.02}
NI_MARGIN_MIN = 0.3
E6_LC_050 = {'B0': 4.671, 'B1': 2.032, 'B2': 0.807, 'B3': 0.764}       # article E6 (E7 package) a_nav p95, LC, 0.50 m/s
R_TOL = 0.5
METRICS = ('a_nav_p95_s', 'a_nav_p95_clipped_s', VIOL, 'W_viol_anav_uact_b0.2_s', 'W_viol_anav_unom_b0.5_s',
           'W_viol_anav_unom_b1.0_s', F, 'V', 'S', P1, CLR, 'P3_m', 'L_scan_replaced', 'L_scan_dropped_codel',
           'L_camera_dropped_slo', 'L_camera_expired', 'L_camera_replaced', 'makl_cam_share_W',
           'makl_contested_cam_share_W', 'makl_beta_med_W_s', 'makl_cpu_med_us', 'makl_cpu_p99_us',
           'makl_cputhread_med_us', 'makl_cputhread_p99_us')
BOOT_N = 10_000; BOOT_SEED = 20261112


def mean(xs):
    xs = [x for x in xs if x is not None]
    return st.mean(xs) if xs else None


def per_run(tid, r):
    td = Path(r['evidence_dir'])
    row = {'trial_id': tid, 'config': r.get('j1_config'), 'cell': r.get('j1_cell'), 'rep': r.get('j1_rep'),
           'delta_s': r.get('j1_delta_s'), 'block': r.get('block'), 'order': r.get('order'), 'status': r.get('status'),
           'contacts': r.get('contacts'), 'emergency': bool(r.get('emergency_event')) or 'EMERGENCY' in str(r.get('error') or ''),
           'bridge_config_line': r.get('bridge_config_line')}
    try:
        import e3b_analysis as A
        b = A.per_run(tid, r, td); row.update({k: b.get(k) for k in ('P1_s', 'P2_stop_s', 'P3_m', 'S', 'F_fps', 'V', 'a_nav_p95_s')})
    except Exception as ex:
        row['e3b_metrics_error'] = repr(ex)
    e = r.get('e5') if isinstance(r.get('e5'), dict) and 'error' not in r.get('e5') else None
    if e is None:
        try:
            e = M.run_measures(td, r)
        except Exception as ex:
            e = {'measures_error': repr(ex)}
    row.update({k: v for k, v in e.items() if not isinstance(v, (list, dict))})
    row['robot_moving_contact'] = row.get('contact_class') == 'robot_moving'
    return row


def by_rep(R, cell, arm, m):
    return {r['rep']: r[m] for r in R if r['cell'] == cell and r['config'] == arm and isinstance(r.get(m), (int, float))
            and math.isfinite(r[m])}


def c0_range(R, cell, m):
    xs = [r[m] for r in R if r['cell'] == C0_OF[cell] and isinstance(r.get(m), (int, float)) and math.isfinite(r[m])]
    return (max(xs) - min(xs)) if xs else None


def J(R, cell, m):
    rg = c0_range(R, cell, m)
    return max(FLOOR[m], rg if rg is not None else 0.0)


def clearly_greater(a, b, j):
    """a, b: {rep: value}. Paired rule."""
    reps = sorted(set(a) & set(b))
    d = [a[k] - b[k] for k in reps]
    res = {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d], 'mean_diff': round(st.mean(d), 4) if d else None, 'J': round(j, 4)}
    res['evaluable'] = len(reps) >= 4
    res['clearly_greater'] = bool(res['evaluable'] and all(x > 0 for x in d) and st.mean(d) > j)
    return res


def boot_ci(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    if len(xs) < 2:
        return None, None
    rnd = random.Random(BOOT_SEED)
    ms = sorted(st.mean(rnd.choice(xs) for _ in xs) for _ in range(BOOT_N))
    return ms[int(0.025 * BOOT_N)], ms[int(0.975 * BOOT_N) - 1]


def a_min(c_bps, lm_payload, ls_payload=2944):
    w = lambda b: b + 28 * ((b + 1471) // 1472)
    return 0.2 + 8 * (w(lm_payload) + w(ls_payload)) / c_bps + 0.02


def main():
    S = json.loads(Path(sys.argv[1]).read_text()); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
    R = [per_run(t, r) for t, r in sorted(S['done'].items(), key=lambda kv: kv[1].get('order', 0))]
    keys = sorted({k for r in R for k in r}, key=lambda k: (k not in ('trial_id', 'config', 'cell', 'rep', 'status'), k))
    with open(OUT / 'J1_RUNS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    # ---------------- cells ----------------
    cells = []
    for c in CELLS:
        for a in ARMS:
            rr = [r for r in R if r['cell'] == c and r['config'] == a]
            if not rr:
                continue
            x = {'cell': c, 'config': a, 'n': len(rr), 'succeeded': sum(r['status'] == 'SUCCEEDED' for r in rr),
                 'robot_moving_contacts': sum(r['robot_moving_contact'] for r in rr), 'emergency_runs': sum(r['emergency'] for r in rr)}
            for m in METRICS:
                v = [r.get(m) for r in rr if isinstance(r.get(m), (int, float)) and math.isfinite(r.get(m))]
                if not v:
                    continue
                x[m] = round(st.mean(v), 4); x[m + '_sd'] = round(st.stdev(v), 4) if len(v) > 1 else None
                if m in ('a_nav_p95_s', 'a_nav_p95_clipped_s', VIOL, F, 'V', P1, CLR):
                    lo, hi = boot_ci(v); x[m + '_ci_lo'] = None if lo is None else round(lo, 4); x[m + '_ci_hi'] = None if hi is None else round(hi, 4)
            cells.append(x)
    ck = []
    for x in cells:
        for k in x:
            if k not in ck:
                ck.append(k)
    with open(OUT / 'J1_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)
    cm = {(x['cell'], x['config']): x for x in cells}
    # ---------------- frozen paired tests ----------------
    T = {}
    def test(name, cell, a, b, m, j):
        T[name] = dict(cell=cell, a=a, b=b, metric=m, **clearly_greater(by_rep(R, cell, a, m), by_rep(R, cell, b, m), j))
        return T[name]
    h1 = {}
    for c in CONG:
        rg = c0_range(R, c, VIOL); margin = max(NI_MARGIN_MIN, rg if rg is not None else 0.0)
        t = test(f'H1_{c}_MAKL_gt_B7_viol', c, 'MAKL', 'B7', VIOL, margin)
        h1[c] = t['evaluable'] and not t['clearly_greater']
    h1b = {c: test(f'H1b_{c}_B3_gt_MAKL_viol', c, 'B3', 'MAKL', VIOL, J(R, c, VIOL))['clearly_greater'] for c in ('LC-Q2-25s', 'LB-Q1-25s')}
    h2a = {c: test(f'H2a_{c}_MAKL_gt_B7_F', c, 'MAKL', 'B7', F, J(R, c, F))['clearly_greater'] for c in ('LB-Q1-25s', 'LB-Q1-8s')}
    h2b = {c: test(f'H2b_{c}_MAKL_gt_B7_F', c, 'MAKL', 'B7', F, J(R, c, F))['clearly_greater'] for c in ('LC-Q2-25s', 'LC-Q2-8s')}
    h3 = {}
    for c in ('LC-Q2-25s', 'LB-Q1-25s'):
        a4 = test(f'H3_{c}_B4_gt_MAKL_viol', c, 'B4', 'MAKL', VIOL, J(R, c, VIOL))['clearly_greater']
        a5 = test(f'H3_{c}_B5_gt_MAKL_viol', c, 'B5', 'MAKL', VIOL, J(R, c, VIOL))['clearly_greater']
        h3[c] = a4 and a5
    h4c = {}
    for c in CONG:
        ok = True
        for m in (P1, CLR):
            a = by_rep(R, c, 'MAKL', m); b = by_rep(R, C0_OF[c], 'MAKL', m); j = J(R, c, m)
            t1 = T[f'H4_{c}_MAKL_vs_C0_{m}_gt'] = dict(cell=c, a='MAKL@' + c, b='MAKL@' + C0_OF[c], metric=m, **clearly_greater(a, b, j))
            t2 = T[f'H4_{c}_MAKL_vs_C0_{m}_lt'] = dict(cell=c, a='MAKL@' + C0_OF[c], b='MAKL@' + c, metric=m, **clearly_greater(b, a, j))
            ok &= t1['evaluable'] and not t1['clearly_greater'] and not t2['clearly_greater']
        h4c[c] = ok
    makl_rm = sum(1 for r in R if r['config'] == 'MAKL' and r['robot_moving_contact'])
    with open(OUT / 'J1_PAIRED.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['test', 'cell', 'a', 'b', 'metric', 'n_pairs', 'evaluable', 'mean_diff', 'J', 'clearly_greater', 'diffs'])
        w.writeheader()
        for k, v in T.items():
            w.writerow(dict(test=k, **{x: v.get(x) for x in ('cell', 'a', 'b', 'metric', 'n_pairs', 'evaluable', 'mean_diff', 'J', 'clearly_greater', 'diffs')}))
    # ---------------- anchor R and bound check P ----------------
    rr = {a: (cm.get(('LC-Q2-25s', a)) or {}).get('a_nav_p95_s') for a in E6_LC_050}
    Rv = {a: {'J1': rr[a], 'E6': e, 'within_0.5s': rr[a] is not None and abs(rr[a] - e) <= R_TOL} for a, e in E6_LC_050.items()}
    P = {}
    for c in ('LC-Q2-25s', 'LB-Q1-25s'):
        lm = mean([r.get('L_camera_payload_mean_B') for r in R if r['cell'] == c])
        b7 = (cm.get((c, 'B7')) or {}).get('a_nav_p95_clipped_s'); b70 = (cm.get((C0_OF[c], 'B7')) or {}).get('a_nav_p95_clipped_s')
        if lm and b7 is not None and b70 is not None:
            off = b70 - a_min(3_000_000, lm); bound = a_min(CAP[c], lm) + off
            P[c] = {'B7_clipped_p95_s': b7, 'a_min_s': round(a_min(CAP[c], lm), 4), 'pipeline_offset_from_C0_s': round(off, 4),
                    'bound_s': round(bound, 4), 'holds': b7 <= bound, 'camera_payload_mean_B': round(lm, 1)}
    # ---------------- verdicts ----------------
    V = {'H1_noninferior_violation_vs_B7_all_congested_cells': all(h1.values()) and len(h1) == 4, 'H1_cells': h1,
         'H1b_violation_below_B3_25s_cells': all(h1b.values()), 'H1b_cells': h1b,
         'H2a_camera_above_B7_Q1_cells': all(h2a.values()), 'H2a_cells': h2a,
         'H2b_camera_above_B7_Q2_cells': all(h2b.values()), 'H2b_cells': h2b,
         'H3_violation_below_B4_and_B5_25s_cells': all(h3.values()), 'H3_cells': h3,
         'H4a_no_robot_moving_contact_MAKL': makl_rm == 0, 'H4a_MAKL_robot_moving_contacts': makl_rm,
         'H4b_P1_and_clearance_not_different_from_C0': all(h4c.values()), 'H4b_cells': h4c,
         'R_anchor_B0_B3_reproduce_E6': all(v['within_0.5s'] for v in Rv.values()), 'R_detail': Rv,
         'P_check_B7_bound': P}
    if V['H1_noninferior_violation_vs_B7_all_congested_cells'] and V['H2a_camera_above_B7_Q1_cells']:
        V['outcome'] = 'MAKL_SUPPORTED'
    elif V['H1_noninferior_violation_vs_B7_all_congested_cells'] or V['H2a_camera_above_B7_Q1_cells']:
        V['outcome'] = 'MAKL_PARTIALLY_SUPPORTED'
    else:
        V['outcome'] = 'MAKL_NOT_SUPPORTED'
    V['n_runs'] = len(R); V['status_counts'] = {s: sum(r['status'] == s for r in R) for s in sorted({r['status'] for r in R})}
    V['robot_moving_contacts_all'] = sum(r['robot_moving_contact'] for r in R)
    (OUT / 'J1_VERDICTS.json').write_text(json.dumps(V, indent=1, default=str) + '\n')
    # ---------------- exploratory: % changes ----------------
    ex = []
    for c in CONG:
        for a in ARMS:
            x = cm.get((c, a))
            if not x:
                continue
            row = {'cell': c, 'config': a}
            for ref in ('B0', 'B7'):
                y = cm.get((c, ref)) or {}
                for m in ('a_nav_p95_s', VIOL, F):
                    if x.get(m) is not None and y.get(m):
                        row[f'{m}_pct_vs_{ref}'] = round(100 * (x[m] - y[m]) / y[m], 1)
            ex.append(row)
    if ex:
        fk = []
        for r in ex:
            for k in r:
                if k not in fk:
                    fk.append(k)
        with open(OUT / 'J1_CHANGES_VS_B0_B7.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fk); w.writeheader(); w.writerows(ex)
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 4, figsize=(16, 4))
        for ax, c in zip(axs, CONG):
            for a in ARMS:
                x = cm.get((c, a))
                if x and x.get(VIOL) is not None and x.get(F) is not None:
                    ax.errorbar(x[VIOL], x[F], xerr=x.get(VIOL + '_sd') or 0, yerr=x.get(F + '_sd') or 0, fmt='o',
                                color='C3' if a == 'MAKL' else ('C0' if a == 'B7' else '0.4'), capsize=2)
                    ax.annotate(a, (x[VIOL], x[F]), textcoords='offset points', xytext=(4, 3), fontsize=8)
            ax.set_title(c); ax.set_xlabel('Age-speed violation (s)'); ax.set_ylabel('Useful camera frames (fps)')
        fig.tight_layout(); fig.savefig(OUT / 'FIG_J1_PARETO.png', dpi=200)
    except Exception as ex2:
        (OUT / 'FIG_ERROR.txt').write_text(repr(ex2) + '\n')
    print('J1_VERDICTS', json.dumps({k: v for k, v in V.items() if not k.endswith('_cells') and k not in ('R_detail', 'P_check_B7_bound')}))
    print('J1_OUTCOME', V['outcome'])


if __name__ == '__main__':
    main()
