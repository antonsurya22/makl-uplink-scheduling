#!/usr/bin/env python3
"""J0 - offline replay of recorded traces through all schedulers (EXPLORATORY; not a counted experiment).
Usage: j0_explore.py TRACES_DIR OUT_DIR
Purpose (JNCA_EXPERIMENT_PLAN J0): tune MAKL (m, u, peak vs waiting test, v source, d source, MAKL-T), see how
large the differences are, and apply the go/no-go gate before any J1 freeze.
Traces: E5 runs at C0 with the pedestrian at 0.5 m/s (F-N1 = camera Q1, F-N2 = Q2; nothing was dropped at C0, so the
received frames are the offers, as in E4) and E7 (= article E6) C0 runs at 0.5 m/s (Q2) for calibration.
Open loop: the robot trajectory (v, d) is the recorded C0 one; it does not react to the data age.
Metrics over W = trigger -> +25 s (E5 definitions): a_nav p95, age-speed violation time (nominal u = 0.5 m/s,
b = 0.2 m/s^2, d = clearance), camera useful rate F (frames received within 1.0 s of capture, per second) and
V (share of W with camera AoI > 1.0 s), drops per status (ledger), MAKL decisions.
GATE (from the plan): in the Q1, 0.18 Mbit/s, 25 s cell, mean over the Q1 traces: viol(MAKL) < viol(B3) AND
F(MAKL) > F(B7). If it fails, MAKL is changed before J1 (and J0 is repeated)."""
import csv, json, math, statistics, sys, time
from collections import Counter
from pathlib import Path
HERE = Path(__file__).resolve().parent
for p in (HERE / 'frozen_deps' / 'frozen', HERE / 'frozen_deps', HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import e4_harness as H
import j_links as J
import j_replay as R

CAPS = (0.10, 0.13, 0.15, 0.18, 0.21, 0.25)
DURS = (8, 25)
ARMS = {  # name -> (config, params, d_mode)
    'B0': ('B0', {}, 'raw360'), 'B1': ('B1', {}, 'raw360'), 'B2': ('B2', {}, 'raw360'), 'B3': ('B3', {}, 'raw360'),
    'B4': ('B4', {}, 'raw360'), 'B4m': ('B4m', {}, 'raw360'), 'B5': ('B5', {}, 'raw360'), 'B7': ('B7', {}, 'raw360'),
    'MAKL': ('MAKL', {}, 'raw360'),
    'MAKL_wait': ('MAKL_wait', {}, 'raw360'), 'MAKL_odom': ('MAKL_odom', {}, 'raw360'),
    'MAKL_cap': ('MAKL_cap', {}, 'raw360'), 'MAKL_true_c': ('MAKL_true_c', {}, 'raw360'),
    'MAKL_T': ('MAKL_T', {}, 'raw360'), 'MAKL_map': ('MAKL', {}, 'dynamic'),
}
for m in (0.0, 0.1, 0.2):
    for u in (0.08, 0.5):
        if (m, u) != (0.1, 0.5):
            ARMS[f'MAKL_m{m}_u{u}'] = ('MAKL', {'m': m, 'u': u}, 'raw360')
E6_MEASURED_LC_050 = {  # article E6 (E7 package), closed loop, Q2, 0.13 Mbit/s 25 s, pedestrian 0.5 m/s, n = 3
    'B0': (4.671, 13.367, 0.28), 'B1': (2.032, 13.267, 0.28), 'B2': (0.807, 1.5, 0.013), 'B3': (0.764, 2.967, 1.573)}


def load_runs(traces, study, want):
    st = json.loads((traces / study / 'STATE.json').read_text())
    out = []
    for tid, d in sorted(st['done'].items()):
        if not want(tid, d):
            continue
        td = traces / study / 'trials' / Path(d['evidence_dir']).name
        if not (td / 'E3B_FRAMES.jsonl').is_file() or not (td / 'E3B_GRID.jsonl').is_file():
            continue
        try:
            tr = H.load_trace(td, dict(d, trial_id=tid))
        except Exception as e:                                  # reported, never silently dropped
            print('J0_TRACE_SKIPPED', study, tid, repr(e)[:200], flush=True)
            continue
        grid = R.load_grid(td)
        out.append(dict(study=study, tid=tid, tr=tr, grid=grid, profile=tr['profile']))
    return out


def one(run, arm, cap, dur):
    cfg, prm, dmode = ARMS[arm]
    L = J.make_jlink(cfg, capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY, params=prm)
    states = R.state_series(run['grid'], dmode)
    t0 = time.perf_counter()
    res = R.replay(run['tr'], L, capacity_mbps=cap, duration_s=dur, offers='live', states=states)
    wall = time.perf_counter() - t0
    v = R.violation(res, run['grid'])
    T = res['T']; a = T; b = T + 25_000_000
    cam = [(now, stamp) for s, now, stamp, _, _ in res['recv'] if s == 'camera' and a <= now < b]
    F = sum(1 for now, stamp in cam if now - stamp <= 1_000_000) / 25.0
    cg = H.age_grid(res['recv'], 'camera', a, b)
    V = sum(1 for x in cg if x > 1.0) / len(cg)
    lines = [x for x in (L.ledger.lines if getattr(L, 'ledger', None) is not None else []) if not x.get('header')]
    st = Counter(f"{x['stream']}:{x['status']}" for x in lines if a <= x['offer_us'] < b)
    row = dict(study=run['study'], trace=run['tid'], profile=run['profile'], arm=arm, capacity_mbps=cap, duration_s=dur,
               a_nav_p95_s=v['a_nav_p95_s'], viol_s=v['viol_s'], cam_F_fps=round(F, 3), cam_V=round(V, 3),
               scan_delivered=st.get('scan:delivered', 0), scan_replaced=st.get('scan:replaced', 0),
               scan_dropped_codel=st.get('scan:dropped_codel', 0), scan_queue_drop=st.get('scan:queue_drop', 0),
               cam_delivered=st.get('camera:delivered', 0), cam_replaced=st.get('camera:replaced', 0),
               cam_expired=st.get('camera:expired', 0), cam_dropped_slo=st.get('camera:dropped_slo', 0),
               cam_dropped_codel=st.get('camera:dropped_codel', 0), wall_s=round(wall, 3))
    if hasattr(L, 'jdisc') and L.jdisc == 'makl':
        dl = [x for x in L.dlog if a <= x['t_us'] < b]
        row['makl_decisions'] = len(dl)
        row['makl_cam_share'] = round(sum(1 for x in dl if x['choice'] == 'camera') / len(dl), 3) if dl else None
        row['makl_contested_cam_share'] = (round(sum(1 for x in dl if x['q_scan'] and x['q_cam'] and x['choice'] == 'camera')
                                                 / max(1, sum(1 for x in dl if x['q_scan'] and x['q_cam'])), 3))
        row['makl_beta_med_s'] = statistics.median([x['beta'] for x in dl]) if dl else None
        ns = sorted(L.cpu_ns)
        row['cpu_med_us'] = round(ns[len(ns) // 2] / 1e3, 2) if ns else None
    return row


def mean(rows, key):
    xs = [r[key] for r in rows if r.get(key) is not None and isinstance(r[key], (int, float)) and math.isfinite(r[key])]
    return round(statistics.mean(xs), 3) if xs else None


def main():
    traces = Path(sys.argv[1]); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    e5 = load_runs(traces, 'E5', lambda tid, d: d.get('level') == 'C0' and abs(float(d.get('speed') or 0) - 0.5) < 1e-9
                   and d.get('arm') in ('F-N1', 'F-N2') and d.get('status') == 'SUCCEEDED')
    e7 = load_runs(traces, 'E7', lambda tid, d: d.get('level') == 'C0' and abs(float(d.get('speed') or 0) - 0.5) < 1e-9
                   and d.get('status') == 'SUCCEEDED')
    print('J0_TRACES', 'E5', len(e5), Counter(r['profile'] for r in e5), 'E7', len(e7), flush=True)
    rows = []
    for run in e5:
        for arm in ARMS:
            for cap in CAPS:
                for dur in DURS:
                    rows.append(one(run, arm, cap, dur))
        print('J0_TRACE_DONE', run['tid'], round(time.time() - t_start), 's', flush=True)
    for run in e7:                                   # calibration and extra Q2 traces: primary cell only
        for arm in ARMS:
            rows.append(one(run, arm, 0.13, 25))
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(out / 'J0_RESULTS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
    # ---- cell summaries (mean over traces)
    summ = {}
    for prof in ('Q1', 'Q2'):
        for cap in CAPS:
            for dur in DURS:
                cell = {}
                for arm in ARMS:
                    rr = [r for r in rows if r['profile'] == prof and r['capacity_mbps'] == cap and r['duration_s'] == dur
                          and r['arm'] == arm and r['study'] == 'E5']
                    if rr:
                        cell[arm] = {k: mean(rr, k) for k in ('a_nav_p95_s', 'viol_s', 'cam_F_fps', 'cam_V',
                                                              'makl_cam_share', 'makl_contested_cam_share', 'cpu_med_us')}
                        cell[arm]['n'] = len(rr)
                summ[f'{prof}_{cap}_{dur}s'] = cell
    calib = {}
    for arm, (a95, vio, F) in E6_MEASURED_LC_050.items():
        rr = [r for r in rows if r['study'] == 'E7' and r['arm'] == arm]
        calib[arm] = dict(replay_open_loop=dict(a_nav_p95_s=mean(rr, 'a_nav_p95_s'), viol_s=mean(rr, 'viol_s'),
                                                cam_F_fps=mean(rr, 'cam_F_fps'), n=len(rr)),
                          measured_closed_loop_E6=dict(a_nav_p95_s=a95, viol_s=vio, cam_F_fps=F, n=3))
    e7summ = {arm: {k: mean([r for r in rows if r['study'] == 'E7' and r['arm'] == arm], k)
                    for k in ('a_nav_p95_s', 'viol_s', 'cam_F_fps', 'cam_V')} for arm in ARMS}
    g = summ.get('Q1_0.18_25s', {})
    gate = None
    if g.get('MAKL') and g.get('B3') and g.get('B7'):
        gate = dict(viol_MAKL=g['MAKL']['viol_s'], viol_B3=g['B3']['viol_s'], F_MAKL=g['MAKL']['cam_F_fps'],
                    F_B7=g['B7']['cam_F_fps'], n_traces=g['MAKL']['n'])
        gate['passed'] = gate['viol_MAKL'] < gate['viol_B3'] and gate['F_MAKL'] > gate['F_B7']
    res = dict(label='J0 EXPLORATORY (open-loop replay; not a counted experiment)', gate_Q1_018_25s=gate,
               calibration_vs_E6_closed_loop=calib, E7_traces_Q2_013_25s=e7summ, cells=summ,
               n_rows=len(rows), wall_s=round(time.time() - t_start, 1))
    (out / 'J0_SUMMARY.json').write_text(json.dumps(res, indent=1, default=str) + '\n')
    print('J0_GATE', json.dumps(gate), flush=True)
    for key in ('Q1_0.18_25s', 'Q2_0.13_25s', 'Q1_0.21_25s', 'Q1_0.13_25s'):
        print('J0_CELL', key, flush=True)
        for arm in ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7', 'MAKL', 'MAKL_wait', 'MAKL_map', 'MAKL_true_c'):
            c = summ.get(key, {}).get(arm)
            if c:
                print(f"   {arm:12s} a95={c['a_nav_p95_s']} viol={c['viol_s']} F={c['cam_F_fps']} V={c['cam_V']}", flush=True)
    print('J0_DONE', len(rows), 'rows', round(time.time() - t_start), 's', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
