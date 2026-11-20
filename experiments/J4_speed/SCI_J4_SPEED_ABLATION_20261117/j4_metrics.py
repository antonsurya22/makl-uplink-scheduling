#!/usr/bin/env python3
"""J4 per-run measures (protocol section 6) on top of the frozen J1 v2 measures (j1_metrics.run_measures, unchanged).
Times are simulation seconds; W = [t0, t0 + 25 s), t0 = network trigger (robot x >= 1.30 m, as J1-J3).
  t_ca     closest approach: the 10 Hz grid point with the smallest ground-truth pedestrian clearance between the
           pedestrian's start and t0 + 25 s (first one if tied), snapped to the grid of W.
  t_run    end of the run inside W = min(terminal time (goal reached), t0 + 25 s).
  phases   A = [t0, t_ca) (trigger -> closest approach); B = [t_ca, t0 + 25 s) for the violation (frozen W definition, so
           A + B = the frozen W violation) and [t_ca, t_run) for frames.
  F        useful frames per second with the frozen E3 rule (received within 1.0 s of capture; e3_policy_logic.F_metric):
           F_run over [t0, t_run) (PRIMARY), F_A, F_B; F_W = the frozen J1 value over 25 s (secondary).
  viol     frozen E5 eq1 (a_nav, nominal pedestrian speed, b = 0.2): viol_A, viol_B; viol_W = frozen J1 value.
  d_sw     switching clearance, from QUALIFYING decisions: in A, contested (scan and camera waiting) and with d set by
           the pedestrian (|d - (ground-truth clearance - 0.10)| <= 0.15 m; d = raw range - 0.22 m, the raw range ends at
           the pedestrian's surface). d_sw = the largest d* among qualifying scan-first decisions such that the window
           [d* - 0.15 m, d*] holds >= 3 qualifying decisions of which >= 50 % sent the scan first. Missing if fewer than 6
           qualifying decisions or no such d*. Also reported: d_first_scan = largest d of a qualifying scan-first decision.
  speed    implementation check (MAKL: speed used in a* = v_hat in every decision; MAKL_cap: = 0.15 in every decision);
           agreement = share of contested decisions in W with a finite predicted peak whose choice is unchanged when
           recomputed with the other arm's speed input (MAKL: 0.15; MAKL_cap: v_hat); sanity = the same recomputation
           with the logged speed must reproduce the logged choice.
  safety   from the 10 Hz grid after the pedestrian's start (AMENDMENT A1): a tick is a SAFETY tick if the mode is not
           NORMAL (decision mask), or the guard is latched, or the guard output is zero (cmd_final_v < 0.005) on this and
           the previous tick while the gated command is > 0.02 (the guard can only pass or zero the command; the frozen
           v0 test 'cmd_final_v < cmd_raw_v - 0.02' also caught the one-tick lag between the two logged commands when DWB
           changes its velocity sample, so it is replaced). A safety tick is a
           WALL tick if the raw minimum range is more than 0.15 m below the pedestrian's raw-equivalent distance
           (clearance + 0.12 m). First safety tick: time and raw range. Shield time = number of safety ticks x 0.1 s.
  pilot    speed window = from the pedestrian's start to the first tick with d = raw - 0.22 below D_LOW[V]: share of ticks
           with |v - V| <= 0.02 (v0, reported), median and minimum robot speed (A1), the largest planner command (before the
           gate); meeting = robot x when the pedestrian
           reaches the path (arrival from the frozen E5 timing) minus x_meet."""
import json, math, sys
from pathlib import Path
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
for p in (str(J1), str(P0J), str(E3B)):
    if p not in sys.path:
        sys.path.insert(0, p)
import j1_metrics as M1
M5 = M1.M5
L = M1.L
import e3_policy_logic as P3
import j_links as J
VIOL_KEY = 'viol_anav_unom_b0.2_s'
FIXV_SPEED = 0.15
D_LOW = {0.10: 0.80, 0.15: 1.00, 0.20: 1.00, 0.22: 1.00}   # lowest predicted switching d of the three switching arms (protocol 2)
PED_TOL = 0.15
WIN = 0.15
MIN_DEC = 6
DT = 0.1


def jl(p):
    p = Path(p)
    out = []
    if not p.is_file():
        return out
    for l in p.read_text(errors='replace').splitlines():
        if l.strip():
            try:
                out.append(json.loads(l))
            except Exception:
                pass
    return out


def snap(t, t0):
    return t0 + round((t - t0) / DT) * DT


def closest_approach(grid, ped_start, t0):
    if ped_start is None or t0 is None:
        return None, None
    g = [r for r in grid if r.get('sim_s') is not None and r.get('clearance_m') is not None
         and ped_start - 1e-9 <= r['sim_s'] < t0 + L.WINDOW_S]
    if not g:
        return None, None
    m = min(g, key=lambda r: (r['clearance_m'], r['sim_s']))
    return snap(m['sim_s'], t0), m['clearance_m']


def clear_at(grid_sorted, t):
    """Ground-truth clearance at the last grid tick at or before t."""
    last = None
    for r in grid_sorted:
        if r['sim_s'] <= t + 1e-9:
            last = r
        else:
            break
    return None if last is None else last.get('clearance_m')


def decisions(td):
    D = []
    for x in jl(Path(td) / 'J1_DECISIONS.jsonl'):
        if x.get('kind') == 'decision':
            D.append(x)
    return D


def d_switch(D, grid, t0, t_ca):
    gs = sorted((r for r in grid if r.get('sim_s') is not None and r.get('clearance_m') is not None), key=lambda r: r['sim_s'])
    q = []
    for x in D:
        t = x['t_us'] / 1e6
        if not (t0 - 1e-9 <= t < t_ca) or not (x.get('q_scan') and x.get('q_cam')) or x.get('choice') not in ('scan', 'camera'):
            continue
        c = clear_at(gs, t)
        if c is None or abs(x['d'] - (c - 0.10)) > PED_TOL:
            continue
        q.append(x)
    out = {'dsw_n_qualifying': len(q), 'd_sw_m': None,
           'd_first_scan_m': max((x['d'] for x in q if x['choice'] == 'scan'), default=None)}
    if len(q) < MIN_DEC:
        return out
    ok = []
    for c in q:
        if c['choice'] != 'scan':
            continue
        w = [x for x in q if c['d'] - WIN - 1e-9 <= x['d'] <= c['d'] + 1e-9]
        if len(w) >= 3 and sum(x['choice'] == 'scan' for x in w) / len(w) >= 0.5:
            ok.append(c['d'])
    if ok:
        out['d_sw_m'] = round(max(ok), 4)
    return out


def recompute(x, v):
    """MAKL's choice for a contested decision with speed input v (frozen formula; logged d and peak)."""
    pk = x.get('peak')
    if pk is None or (isinstance(pk, float) and not math.isfinite(pk)):
        return 'scan'
    beta = max(0.2, J.astar(v, x['d'], 0.5, 0.2, 0.1) - 0.1)
    return 'scan' if pk > beta else 'camera'


def speed_checks(D, config, t0):
    m = [x for x in D if 'v_hat' in x]
    out = {'impl_n': len(m), 'impl_ok': None, 'agree_share_W': None, 'sanity_share_W': None, 'v_hat_med_W': None}
    if config not in ('MAKL', 'MAKL_cap') or not m:
        return out
    if config == 'MAKL':
        out['impl_ok'] = all(abs(x['v'] - x['v_hat']) <= 1e-4 for x in m)
    else:
        out['impl_ok'] = all(abs(x['v'] - FIXV_SPEED) <= 1e-9 for x in m)
    W = [x for x in m if t0 is not None and t0 <= x['t_us'] / 1e6 < t0 + L.WINDOW_S]
    vh = sorted(x['v_hat'] for x in W if x['v_hat'] > 0.02)
    out['v_hat_med_W'] = vh[len(vh) // 2] if vh else None
    C = [x for x in W if x.get('q_scan') and x.get('q_cam') and x.get('choice') in ('scan', 'camera')
         and isinstance(x.get('peak'), (int, float)) and math.isfinite(x['peak'])]
    if C:
        alt = (lambda x: FIXV_SPEED) if config == 'MAKL' else (lambda x: x['v_hat'])
        out['agree_share_W'] = round(sum(recompute(x, alt(x)) == x['choice'] for x in C) / len(C), 4)
        out['sanity_share_W'] = round(sum(recompute(x, x['v']) == x['choice'] for x in C) / len(C), 4)
        out['agree_n_W'] = len(C)
    return out


def safety(grid, ped_start, t0):
    out = {'safety_ticks': 0, 'wall_safety_ticks': 0, 'first_safety_sim_s': None, 'first_safety_raw_m': None,
           'shield_time_s': 0.0}
    if ped_start is None:
        return out
    prev_zero = False
    for r in sorted((r for r in grid if r.get('sim_s') is not None), key=lambda r: r['sim_s']):
        raw_v = r.get('cmd_raw_v'); fin = r.get('cmd_final_v')
        zero = raw_v is not None and fin is not None and raw_v > 0.02 and fin < 0.005
        cut = zero and prev_zero
        prev_zero = zero
        if r['sim_s'] < ped_start - 1e-9 or (t0 is not None and r['sim_s'] >= t0 + L.WINDOW_S):
            continue
        act = (r.get('mode') not in (None, 0)) or bool(r.get('guard_latched')) or cut
        if not act:
            continue
        out['safety_ticks'] += 1
        if out['first_safety_sim_s'] is None:
            out['first_safety_sim_s'] = r['sim_s']; out['first_safety_raw_m'] = r.get('raw_scan_min_m')
        c = r.get('clearance_m'); raw = r.get('raw_scan_min_m')
        if c is not None and raw is not None and raw < (c + 0.12) - PED_TOL:
            out['wall_safety_ticks'] += 1
    out['shield_time_s'] = round(out['safety_ticks'] * DT, 2)
    return out


def speed_window(grid, ped_start, V):
    out = {'speedwin_ticks': 0, 'speedwin_share_within_0.02': None, 'speedwin_cmd_policy_max': None, 'speedwin_end_sim_s': None,
           'speedwin_v_median': None, 'speedwin_v_min': None}
    if ped_start is None or V not in D_LOW:
        return out
    g = sorted((r for r in grid if r.get('sim_s') is not None and r['sim_s'] >= ped_start - 1e-9), key=lambda r: r['sim_s'])
    win = []
    for r in g:
        raw = r.get('raw_scan_min_m')
        if raw is not None and raw - 0.22 < D_LOW[V]:
            out['speedwin_end_sim_s'] = r['sim_s']
            break
        win.append(r)
    v = [r['v_mps'] for r in win if r.get('v_mps') is not None]
    cp = [r['cmd_policy_v'] for r in win if r.get('cmd_policy_v') is not None]
    out['speedwin_ticks'] = len(v)
    if v:
        out['speedwin_share_within_0.02'] = round(sum(abs(x - V) <= 0.02 for x in v) / len(v), 4)
        vs = sorted(v); n = len(vs)
        out['speedwin_v_median'] = round(vs[n // 2] if n % 2 else (vs[n // 2 - 1] + vs[n // 2]) / 2, 4)
        out['speedwin_v_min'] = round(vs[0], 4)
    if cp:
        out['speedwin_cmd_policy_max'] = round(max(cp), 4)
    return out


def cmd_policy_max_moving(grid):
    xs = [r['cmd_policy_v'] for r in grid if r.get('cmd_policy_v') is not None]
    return round(max(xs), 4) if xs else None


def phase_measures(td, res, V, config):
    td = Path(td)
    grid = M5.jl(td / 'E3B_GRID.jsonl')
    edge = [(r['recv_sim_s'], r['stamp_s']) for r in M5.jl(td / 'E3B_EDGE_SCAN.jsonl')]
    frames = [(r['recv_sim_s'], r['stamp_s']) for r in M5.jl(td / 'E3B_FRAMES.jsonl')]
    cr = res.get('crossing') or {}
    cfg = cr.get('e5_cfg') or {}
    u = float(cfg.get('speed') or 0.5)
    t0 = res.get('trigger_sim_s'); term = res.get('terminal_sim_s')
    ped = cr.get('pedestrian_start_sim_s')
    out = {'j4_v_level': V}
    if t0 is None:
        return out
    t_run = min(term, t0 + L.WINDOW_S) if term is not None else t0 + L.WINDOW_S
    t_ca, c_ca = closest_approach(grid, ped, t0)
    out.update({'t_ca_after_trigger_s': round(t_ca - t0, 2) if t_ca is not None else None, 'clear_ca_m': c_ca,
                't_run_after_trigger_s': round(t_run - t0, 2)})
    if t_run > t0 + 0.5:
        out['F_run'] = round(P3.F_metric(frames, t0, t_run)[0], 4)
    if t_ca is not None:
        if t_ca > t0 + 0.5:
            out['F_A'] = round(P3.F_metric(frames, t0, t_ca)[0], 4)
            out['viol_A'] = M5.eq1(grid, edge, t0, t_ca, u).get(VIOL_KEY)
        if t_run > t_ca + 0.5:
            out['F_B'] = round(P3.F_metric(frames, t_ca, t_run)[0], 4)
        if t0 + L.WINDOW_S > t_ca:
            out['viol_B'] = M5.eq1(grid, edge, t_ca, t0 + L.WINDOW_S, u).get(VIOL_KEY)
    D = decisions(td)
    if t_ca is not None and config != 'B7':
        out.update(d_switch(D, grid, t0, t_ca))
    out.update(speed_checks(D, config, t0))
    out.update(safety(grid, ped, t0))
    out.update(speed_window(grid, ped, V))
    out['cmd_policy_max'] = cmd_policy_max_moving(grid)
    tm = M5.actor_timing(td, cfg) if cfg else None
    if tm and tm.get('arrival_sim_s') is not None:
        g = [r for r in grid if r.get('sim_s') is not None and r.get('robot_xy') and r['sim_s'] <= tm['arrival_sim_s'] + 1e-9]
        if g:
            xr = max(g, key=lambda r: r['sim_s'])['robot_xy'][0]
            out['robot_x_at_arrival'] = round(xr, 3)
            if cfg.get('x_meet') is not None:
                out['meet_error_m'] = round(xr - float(cfg['x_meet']), 3)
    return out


def run_measures(td, res, V, config):
    m = M1.run_measures(td, res)
    m.update(phase_measures(td, res, V, config))
    return m
