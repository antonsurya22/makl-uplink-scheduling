#!/usr/bin/env python3
"""J4 pre-specified analysis (J4_PROTOCOL_FREEZE.md sections 6-8). Usage: j4_analysis.py STATE_JSON OUT_DIR
Per-run rows: the frozen J1 v2 analysis (j1_analysis.per_run) on the stored measures (j4_metrics adds the J4 items).
Cells are 'V010|LB-Q1' etc. (speed level | congestion cell); pairs are the start shifts (rep 1-5).
Rules (protocol 7):
  J-rule (J1): A clearly greater than B if >= 4 of 5 pairs exist, every paired difference A - B > 0 and the mean > J.
       J per speed and metric = max(J1 Q1 value, range of the 5 B7 C0 runs at that speed); J1 Q1 values: violation 0.8 s
       (total and phase A), useful frames 0.4 fps (F_run).
  sign rule: >= 4 pairs and every paired difference has the predicted sign (reported with mean and bootstrap interval;
       never called 'clearly').
  mechanism rule (d_sw): >= 4 pairs, every paired difference has the predicted sign and the mean > 0.10 m.
Outputs: J4_RUNS.csv, J4_CELLS.csv, J4_TESTS.csv, J4_VERDICTS.json, J4_EXPLORATORY.json, FIG_J4.png (exploratory)."""
import csv, json, math, random, statistics as st, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
for p in (str(HERE), str(J1)):
    if p not in sys.path:
        sys.path.insert(0, p)
import j1_analysis as A1

ARMS = ('B7', 'MAKL', 'MAKL_cap', 'DSW_1.0')
VIOL = A1.VIOL; FW = A1.F
VA = 'viol_A'; FR = 'F_run'; DSW = 'd_sw_m'
J1_VALUE = {VIOL: 0.8, VA: 0.8, FR: 0.4}
MECH_J = 0.10
J3_REF = {'B7': (1.24, 0.5079, 2.272, 0.0522), 'DSW_1.0': (1.38, 0.228, 3.088, 0.3549), 'MAKL': (1.24, 0.2302, 3.184, 0.1043)}
cg = A1.clearly_greater


def vtag(v):
    return 'V%03d' % int(round(v * 100))


def cellname(v, c='LB-Q1'):
    return f'{vtag(v)}|{c}'


def boot_mean_ci(d, n=10000, seed=20261117):
    d = [x for x in d if x is not None and math.isfinite(x)]
    if len(d) < 2:
        return None, None
    r = random.Random(seed); ms = sorted(st.mean(r.choice(d) for _ in d) for _ in range(n))
    return round(ms[int(0.025 * n)], 4), round(ms[int(0.975 * n) - 1], 4)


def sign_rule(a, b):
    reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
    lo, hi = boot_mean_ci(d)
    return {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d], 'mean_diff': round(st.mean(d), 4) if d else None,
            'ci_lo': lo, 'ci_hi': hi, 'evaluable': len(reps) >= 4, 'all_predicted_sign': bool(len(reps) >= 4 and all(x > 0 for x in d))}


def both(a, b, j):
    t = cg(a, b, j); s = sign_rule(a, b)
    t.update({'sign_rule': s['all_predicted_sign'], 'ci_lo': s['ci_lo'], 'ci_hi': s['ci_hi']})
    return t


def main():
    S = json.loads(Path(sys.argv[1]).read_text()); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
    levels = [float(x) for x in S.get('levels') or (0.10, 0.15, 0.22)]
    lo, mid, top = levels[0], levels[1], levels[-1]
    R = []
    for t, r in sorted(S['done'].items(), key=lambda kv: kv[1].get('order', 0)):
        row = A1.per_run(t, r)
        row['v_level'] = r.get('j4_v')
        d = r.get('j3_dsw') or {}
        row.update({'dsw_' + k: v for k, v in d.items() if not isinstance(v, (list, dict))})
        R.append(row)
    keys = sorted({k for r in R for k in r}, key=lambda k: (k not in ('trial_id', 'config', 'cell', 'rep', 'status'), k))
    with open(OUT / 'J4_RUNS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    vals = lambda c, a, m: A1.by_rep(R, c, a, m)
    MET = (VIOL, VA, 'viol_B', FW, FR, 'F_A', 'F_B', DSW, 'd_first_scan_m', 'dsw_n_qualifying', 'agree_share_W', 'v_hat_med_W',
           'a_nav_p95_clipped_s', 'P1_s', 'W_min_clearance_m', 'shield_time_s', 'wall_safety_ticks', 't_ca_after_trigger_s',
           't_run_after_trigger_s', 'makl_cam_share_W', 'dsw_scan_first_share_W')
    cells = []; CM = {}
    for v in levels:
        for c, arms in (('LB-Q1', ARMS), ('C0-Q1', ('B7',))):
            for a in arms:
                rr = [r for r in R if r['cell'] == cellname(v, c) and r['config'] == a]
                if not rr:
                    continue
                x = {'v_level': v, 'cell': cellname(v, c), 'config': a, 'n': len(rr),
                     'succeeded': sum(r['status'] == 'SUCCEEDED' for r in rr),
                     'robot_moving_contacts': sum(r['robot_moving_contact'] for r in rr)}
                for m in MET:
                    z = [r.get(m) for r in rr if isinstance(r.get(m), (int, float)) and math.isfinite(r.get(m))]
                    if z:
                        x[m] = round(st.mean(z), 4); x[m + '_sd'] = round(st.stdev(z), 4) if len(z) > 1 else None
                        x[m + '_range'] = round(max(z) - min(z), 4); x[m + '_n'] = len(z)
                cells.append(x); CM[(v, c, a)] = x
    ck = []
    for x in cells:
        for k in x:
            if k not in ck:
                ck.append(k)
    with open(OUT / 'J4_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)
    # ---- J per speed ----
    Jv = {}
    for v in levels:
        Jv[v] = {}
        for m in (VIOL, VA, FR):
            z = [r[m] for r in R if r['cell'] == cellname(v, 'C0-Q1') and r['config'] == 'B7' and isinstance(r.get(m), (int, float))
                 and math.isfinite(r[m])]
            Jv[v][m] = round(max(J1_VALUE[m], (max(z) - min(z)) if len(z) >= 2 else 0.0), 4)
    T = {}
    C = lambda v: cellname(v)
    # ---- mechanism ----
    T['M1_MAKL_dsw_top_gt_lo'] = cg(vals(C(top), 'MAKL', DSW), vals(C(lo), 'MAKL', DSW), MECH_J)
    M1 = T['M1_MAKL_dsw_top_gt_lo']['clearly_greater']
    a, b = vals(C(top), 'MAKL_cap', DSW), vals(C(lo), 'MAKL_cap', DSW)
    reps = sorted(set(a) & set(b)); dd = [a[k] - b[k] for k in reps]
    T['M2_FixedV_dsw_top_minus_lo'] = {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in dd],
                                       'mean_diff': round(st.mean(dd), 4) if dd else None, 'limit': MECH_J}
    M2 = len(reps) >= 4 and abs(st.mean(dd)) <= MECH_J
    T['M_extra_MAKL_dsw_top_minus_FixedV_dsw_top'] = sign_rule(vals(C(top), 'MAKL', DSW), vals(C(top), 'MAKL_cap', DSW))
    T['M_extra_FixedV_dsw_lo_minus_MAKL_dsw_lo'] = sign_rule(vals(C(lo), 'MAKL_cap', DSW), vals(C(lo), 'MAKL', DSW))
    # ---- S0: speed implementation (a) and agreement at the middle level (b) ----
    mk = [r for r in R if r['config'] in ('MAKL', 'MAKL_cap') and r['cell'].endswith('LB-Q1') and r.get('impl_n')]
    S0a = bool(mk) and all(r.get('impl_ok') is True and (r.get('sanity_share_W') or 0) >= 0.99 for r in mk)
    ag = [r['agree_share_W'] for r in R if r['config'] in ('MAKL', 'MAKL_cap') and r['cell'] == C(mid)
          and isinstance(r.get('agree_share_W'), (int, float))]
    S0b_mean = round(st.mean(ag), 4) if ag else None
    S0b = S0b_mean is not None and S0b_mean >= 0.95
    # ---- outcome tests ----
    T['S1_top_FixedV_gt_MAKL_violA'] = both(vals(C(top), 'MAKL_cap', VA), vals(C(top), 'MAKL', VA), Jv[top][VA])
    T['S2_lo_MAKL_gt_FixedV_Frun'] = both(vals(C(lo), 'MAKL', FR), vals(C(lo), 'MAKL_cap', FR), Jv[lo][FR])
    T['D1_top_DSW_gt_MAKL_violA'] = both(vals(C(top), 'DSW_1.0', VA), vals(C(top), 'MAKL', VA), Jv[top][VA])
    T['D2_lo_MAKL_gt_DSW_Frun'] = both(vals(C(lo), 'MAKL', FR), vals(C(lo), 'DSW_1.0', FR), Jv[lo][FR])
    S1, S2 = T['S1_top_FixedV_gt_MAKL_violA'], T['S2_lo_MAKL_gt_FixedV_Frun']
    D1, D2 = T['D1_top_DSW_gt_MAKL_violA'], T['D2_lo_MAKL_gt_DSW_Frun']
    h3 = {}; h4 = {}
    for v in levels:
        t = T[f'H3_{vtag(v)}_MAKL_gt_B7_violW'] = cg(vals(C(v), 'MAKL', VIOL), vals(C(v), 'B7', VIOL), Jv[v][VIOL])
        h3[vtag(v)] = t['evaluable'] and not t['clearly_greater']
        t = T[f'H4_{vtag(v)}_MAKL_gt_B7_Frun'] = cg(vals(C(v), 'MAKL', FR), vals(C(v), 'B7', FR), Jv[v][FR])
        h4[vtag(v)] = t['clearly_greater']
    H3 = all(h3.values()) and len(h3) == len(levels)
    H4 = all(h4.values()) and len(h4) == len(levels)
    rm = sum(r['robot_moving_contact'] for r in R)
    Rv = {}
    for a in ('B7', 'DSW_1.0'):
        x = CM.get((mid, 'LB-Q1', a), {}); vm, vs, fm, fs = J3_REF[a]
        tv = max(0.8, 2 * vs); tf = max(0.4, 2 * fs)
        Rv[a] = {'viol': x.get(VIOL), 'viol_J3': vm, 'tol_viol': round(tv, 3), 'F_W': x.get(FW), 'F_J3': fm, 'tol_F': round(tf, 3),
                 'ok': x.get(VIOL) is not None and x.get(FW) is not None and abs(x[VIOL] - vm) <= tv and abs(x[FW] - fm) <= tf}
    xm = CM.get((mid, 'LB-Q1', 'MAKL'), {})
    Rmakl = {'viol': xm.get(VIOL), 'viol_J3': J3_REF['MAKL'][0], 'F_W': xm.get(FW), 'F_J3': J3_REF['MAKL'][2],
             'note': 'reported only: at the middle level the DWB cap is 0.15 instead of 0.18 m/s, so MAKL speed input is lower than in J3'}
    V = {'levels': levels, 'top_level_reason': S.get('top_level_reason'), 'J_per_speed': {vtag(v): Jv[v] for v in levels},
         'M1_MAKL_switching_clearance_rises_with_speed': M1,
         'M2_FixedV_switching_clearance_flat': M2,
         'S0a_speed_implementation': S0a,
         'S0b_agreement_at_middle_level_ge_0.95': S0b, 'S0b_mean_agreement': S0b_mean,
         'S1_J': S1['clearly_greater'], 'S1_sign': S1['sign_rule'], 'S2_J': S2['clearly_greater'], 'S2_sign': S2['sign_rule'],
         'D1_J': D1['clearly_greater'], 'D1_sign': D1['sign_rule'], 'D2_J': D2['clearly_greater'], 'D2_sign': D2['sign_rule'],
         'H3_MAKL_violation_not_clearly_above_B7_all_speeds': H3, 'H3_by_speed': h3,
         'H4_MAKL_frames_clearly_above_B7_all_speeds': H4, 'H4_by_speed': h4,
         'H5_no_robot_moving_contact': rm == 0, 'robot_moving_contacts': rm,
         'R_B7_DSW_reproduce_J3_at_middle_level': all(v['ok'] for v in Rv.values()), 'R_detail': Rv, 'R_MAKL_reported': Rmakl}
    if not S0a:
        V['speed_term'] = 'INVALID_SPEED_IMPLEMENTATION'
    elif M1 and M2 and (S1['clearly_greater'] or S2['clearly_greater']):
        V['speed_term'] = 'SPEED_TERM_SUPPORTED'
    elif M1 and M2 and (S1['sign_rule'] or S2['sign_rule']):
        V['speed_term'] = 'SPEED_TERM_MECHANISM_ONLY'
    else:
        V['speed_term'] = 'SPEED_TERM_NOT_SHOWN'
    if H3 and (D1['clearly_greater'] or D2['clearly_greater']):
        V['distance_switch'] = 'DSW_ADVANTAGE'
    elif H3 and (D1['sign_rule'] or D2['sign_rule']):
        V['distance_switch'] = 'DSW_SIGN_ONLY'
    else:
        V['distance_switch'] = 'DSW_NOT_SHOWN'
    V['MAKL_MARGIN_TRADEOFF_DEGRADED'] = not H3
    V['phase_of_frame_differences'] = {}
    for name, (va, vb, aa, ab) in {'S2': (lo, lo, 'MAKL', 'MAKL_cap'), 'D2': (lo, lo, 'MAKL', 'DSW_1.0'),
                                   'S_top': (top, top, 'MAKL', 'MAKL_cap')}.items():
        out = {}
        for m in ('F_A', 'F_B', FR):
            s = sign_rule(vals(C(va), aa, m), vals(C(vb), ab, m)); out[m] = {'mean_diff': s['mean_diff'], 'n': s['n_pairs']}
        V['phase_of_frame_differences'][name] = out
    V['safety_record'] = {f'{vtag(v)}|{a}': {k: CM.get((v, 'LB-Q1', a), {}).get(k) for k in
                                              ('robot_moving_contacts', 'shield_time_s', 'wall_safety_ticks', 'W_min_clearance_m')}
                          for v in levels for a in ARMS}
    V['n_runs'] = len(R); V['status_counts'] = {s: sum(r['status'] == s for r in R) for s in sorted({r['status'] for r in R})}
    (OUT / 'J4_VERDICTS.json').write_text(json.dumps(V, indent=1, default=str) + '\n')
    with open(OUT / 'J4_TESTS.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['test', 'result_json'])
        for k, v in T.items():
            w.writerow([k, json.dumps(v)])
    # ---------------- exploratory ----------------
    X = {'d_sw_mean_by_arm_and_speed': {a: {vtag(v): CM.get((v, 'LB-Q1', a), {}).get(DSW) for v in levels}
                                        for a in ('MAKL', 'MAKL_cap', 'DSW_1.0')},
         'v_hat_median_MAKL': {vtag(v): CM.get((v, 'LB-Q1', 'MAKL'), {}).get('v_hat_med_W') for v in levels},
         'agreement_mean': {f'{vtag(v)}|{a}': CM.get((v, 'LB-Q1', a), {}).get('agree_share_W') for v in levels
                            for a in ('MAKL', 'MAKL_cap')}}
    (OUT / 'J4_EXPLORATORY.json').write_text(json.dumps(X, indent=1) + '\n')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(15, 4.3))
        col = {'B7': 'C0', 'MAKL': 'C3', 'MAKL_cap': 'C1', 'DSW_1.0': '0.4'}
        for i, (m, lab) in enumerate(((DSW, 'switching clearance d_sw (m)'), (VA, 'violation in phase A (s)'),
                                      (FR, 'useful frames over the run (fps)'))):
            ax = axs[i]
            for a in ARMS:
                ys = [CM.get((v, 'LB-Q1', a), {}).get(m) for v in levels]
                if any(y is not None for y in ys):
                    ax.plot(levels, [y if y is not None else float('nan') for y in ys], 'o-', color=col[a], label=a)
            ax.set_xlabel('robot speed level (m/s)'); ax.set_ylabel(lab); ax.set_xticks(levels)
        axs[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(OUT / 'FIG_J4.png', dpi=150)
    except Exception as ex:
        (OUT / 'FIG_ERROR.txt').write_text(repr(ex) + '\n')
    print('J4_VERDICTS', json.dumps({k: v for k, v in V.items() if k not in ('R_detail', 'safety_record', 'phase_of_frame_differences',
                                                                            'H3_by_speed', 'H4_by_speed', 'R_MAKL_reported')}))
    print('J4_OUTCOME speed_term=%s distance_switch=%s margin_tradeoff_degraded=%s' % (
        V['speed_term'], V['distance_switch'], V['MAKL_MARGIN_TRADEOFF_DEGRADED']))


if __name__ == '__main__':
    main()
