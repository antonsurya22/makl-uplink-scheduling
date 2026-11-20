#!/usr/bin/env python3
"""J1 per-run measures = the frozen E5 measures (e5_metrics.run_measures, E5 package, unchanged; W = trigger -> +25 s, also
for 8 s events) + the frozen E7 ledger summary (e7_metrics.ledger_summary on J1_LEDGER.jsonl) + the J1 items below.
  ledger: per stream in W, also the new statuses dropped_codel and dropped_slo (counts);
  a_nav p95 CLIPPED (X1 definition): frozen a_nav grid over [trigger, min(trigger + 25 s, last grid sample)];
  event duration MEASURED: recovery boundary - congestion boundary from the bridge's network_phase events;
  MAKL: decisions in W, share of decisions that chose the camera, share among contested decisions (scan and camera both
        waiting), median beta, decisions with v > 0, CPU time per decision (median, p99, max; whole run);
  MAKL input evidence: the last 'J1_STATE' line of the bridge log (counts of odometry, command and scan inputs)."""
import json, math, re, statistics, sys
from pathlib import Path
WS = Path.home() / 'ros2_ws'
for p in (WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102', WS / 'SCI_E5_PED_SPEED_20261101', WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import e7_metrics as M7
import e5_metrics as M5
import e3b_policy_logic as L


def pct(xs, q):
    xs = sorted(xs)
    return None if not xs else xs[min(len(xs) - 1, int(math.ceil(q * len(xs))) - 1)]


def anav_p95_clipped(grid, edge, t0):
    if t0 is None or not grid:
        return None
    gend = max(r['sim_s'] for r in grid if r.get('sim_s') is not None)
    t1 = min(t0 + L.WINDOW_S, gend)
    if t1 <= t0 + L.GRID_DT:
        return None
    a = [x for x in L.a_nav_grid(edge, t0, t1) if x is not None and math.isfinite(x)]
    return pct(a, .95)


def phase_times(res):
    ev = [e.get('event', {}) for e in (res.get('network_events') or [])]
    ph = {e.get('new_phase'): e for e in ev if e.get('event') == 'network_phase'}
    c = ph.get('congested', {}).get('boundary_sim_us')
    r = ph.get('recovery', {}).get('boundary_sim_us')
    return c, r, ph.get('congested', {}).get('new_bps')


def decisions(td, t0_us):
    p = Path(td) / 'J1_DECISIONS.jsonl'
    if not p.is_file():
        return {}
    D = []; C = []; CT = []; hdr = None
    for l in p.read_text(errors='replace').splitlines():
        if not l.strip():
            continue
        x = json.loads(l)
        if x.get('header'):
            hdr = x
        elif 'cpu_ns' in x:
            C.append(x['cpu_ns'])
        elif 'cputhread_ns' in x:
            CT.append(x['cputhread_ns'])
        else:
            D.append(x)
    W = [x for x in D if t0_us is not None and t0_us <= x['t_us'] < t0_us + 25_000_000]
    cont = [x for x in W if x['q_scan'] and x['q_cam']]
    C = sorted(C); CT = sorted(CT)
    return {'makl_header_config': (hdr or {}).get('config'), 'makl_decisions_all': len(D), 'makl_decisions_W': len(W),
            'makl_cam_share_W': round(sum(x['choice'] == 'camera' for x in W) / len(W), 4) if W else None,
            'makl_scan_chosen_W': sum(x['choice'] == 'scan' for x in W), 'makl_cam_chosen_W': sum(x['choice'] == 'camera' for x in W),
            'makl_contested_cam_share_W': round(sum(x['choice'] == 'camera' for x in cont) / len(cont), 4) if cont else None,
            'makl_beta_med_W_s': statistics.median([x['beta'] for x in W]) if W else None,
            'makl_decisions_v_pos_all': sum(1 for x in D if (x.get('v') or 0) > 0),
            'makl_d_med_W_m': statistics.median([x['d'] for x in W]) if W else None,
            'makl_cpu_med_us': round(C[len(C) // 2] / 1e3, 2) if C else None,
            'makl_cpu_p99_us': round(C[min(len(C) - 1, int(0.99 * len(C)))] / 1e3, 2) if C else None,
            'makl_cpu_max_us': round(C[-1] / 1e3, 2) if C else None,
            'makl_cputhread_med_us': round(CT[len(CT) // 2] / 1e3, 2) if CT else None,
            'makl_cputhread_p99_us': round(CT[min(len(CT) - 1, int(0.99 * len(CT)))] / 1e3, 2) if CT else None,
            'makl_cputhread_max_us': round(CT[-1] / 1e3, 2) if CT else None}


def bridge_state(td):
    p = Path(td) / 'c52_joint_bridge.log'
    if not p.is_file():
        return {}
    last = None
    for l in p.read_text(errors='replace').splitlines():
        if l.startswith('J1_STATE '):
            last = l
    if not last:
        return {'j1_state_line': None}
    kv = dict(re.findall(r'(\w+)=([-\d.]+)', last))
    return {'j1_state_line': last, 'j1_inputs_odom': int(kv.get('odom', 0)), 'j1_inputs_cmd': int(kv.get('cmd', 0)),
            'j1_inputs_scans': int(kv.get('scans', 0))}


def run_measures(td, res):
    td = Path(td)
    m = M5.run_measures(td, res)
    hdr, rows = M7.read_ledger(td / 'J1_LEDGER.jsonl')
    m['ledger_config'] = hdr.get('config') if hdr else None
    m['ledger_header'] = hdr
    m['ledger_lines'] = len(rows)
    t0 = res.get('trigger_sim_s')
    t0_us = int(round(t0 * 1e6)) if t0 is not None else None
    if rows and t0_us is not None:
        m.update({'L_' + k: v for k, v in M7.ledger_summary(rows, t0_us).items()})
        Wr = [r for r in rows if t0_us <= r['offer_us'] < t0_us + 25_000_000]
        for s in ('scan', 'camera'):
            for st in ('dropped_codel', 'dropped_slo'):
                m[f'L_{s}_{st}'] = sum(1 for r in Wr if r['stream'] == s and r['status'] == st)
        cw = [r['bytes'] for r in rows if r['stream'] == 'camera']
        m['L_camera_payload_mean_B'] = round(statistics.mean(cw), 1) if cw else None
        m['L_camera_payload_max_B'] = max(cw) if cw else None
    grid = M5.jl(td / 'E3B_GRID.jsonl')
    edge = [(r['recv_sim_s'], r['stamp_s']) for r in M5.jl(td / 'E3B_EDGE_SCAN.jsonl')]
    m['a_nav_p95_clipped_s'] = anav_p95_clipped(grid, edge, t0)
    c, r, bps = phase_times(res)
    m['event_congested_bps'] = bps
    m['event_duration_measured_s'] = round((r - c) / 1e6, 3) if (c is not None and r is not None) else None
    m.update(decisions(td, t0_us))
    m.update(bridge_state(td))
    return m
