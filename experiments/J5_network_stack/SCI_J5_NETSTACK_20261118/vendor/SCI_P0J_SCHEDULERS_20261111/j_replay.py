#!/usr/bin/env python3
"""P0-J replay harness = the frozen E4 harness timing model (frozen_deps/e4_harness.py, SHA256 4857643f...) with
three additions needed for J0:
  1. any link object (frozen B0-B3 via e7_link.make_link, or the new j_links arms);
  2. the robot state for MAKL at every 0.1 s tick, from the recorded E3B_GRID.jsonl of the same run (open loop):
     v_odom = v_mps, v_cmd = cmd_policy_v (Nav2 output before the safety layer), d = raw_scan_min_m - 0.22
     ('raw360', default) or clearance_m to the pedestrian ('dynamic', J0 ablation only);
  3. 'live' offers: the reliability flag is passed exactly as the bridge passes it (Q1 frames after the goal are
     'reliable'); the link decides what that means (frozen K0 honours it with a 1 s expiry, K1 expires camera
     frames after 1 s by configuration, the others ignore it). This mirrors what J1 will run in the closed loop.
Timing model, tick order and offer rules are copied from e4_harness.replay; test_p0j.py T5 checks that this file
reproduces e4_harness.replay record by record and the frozen E4_RESULTS_ALL.csv rows exactly."""
from __future__ import annotations
import json, math, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
for p in (HERE / 'frozen_deps' / 'frozen', HERE / 'frozen_deps', HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import e4_harness as H                 # frozen: load_trace, metrics, age_grid, constants
import e3b_policy_logic as LB          # frozen: hold_grid, a_nav_grid, GRID_DT, WINDOW_S
import j_links as J

TICK = H.TICK
ROBOT_R = 0.22


def us(x):
    return int(round(x * 1e6))


def load_grid(tdir):
    p = Path(tdir) / 'E3B_GRID.jsonl'
    return H.jl(p) if p.is_file() else []


def state_series(grid, d_mode='raw360'):
    """[(t_us, v_odom, v_cmd, d)] sorted; None fields are carried as None (link keeps its last value)."""
    out = []
    for g in grid:
        s = g.get('sim_s')
        if s is None:
            continue
        if d_mode == 'raw360':
            d = g.get('raw_scan_min_m')
            d = None if d is None else d - ROBOT_R
        elif d_mode == 'dynamic':
            d = g.get('clearance_m')
        else:
            raise ValueError(d_mode)
        out.append((us(s), g.get('v_mps'), g.get('cmd_policy_v'), d))
    out.sort(key=lambda x: x[0])
    return out


def replay(tr, link, *, capacity_mbps, duration_s, horizon_after_s=5.0, offers='live', states=None,
           cam_expiry=None, scan_expiry=None):
    """Same loop as e4_harness.replay. `link` is a fresh link at 3 Mbit/s.
    offers='live'       : reliability as the bridge sets it; expiry left to the link ('stream').
    offers='e4_frozen'  : identical to e4_harness link='frozen' (for the equivalence test).
    offers='e4_factor'  : identical to e4_harness link='discipline', cam_mode='factor'.
    states: output of state_series (only used by links that have set_state)."""
    T = tr['trigger_us']; G = tr['goal_us']; t0 = (G // TICK) * TICK; t1 = T + us(duration_s + horizon_after_s)
    L = link
    L.advance(t0)
    bounds = [(T, int(capacity_mbps * 1e6)), (T + us(duration_s), 3_000_000)] if capacity_mbps is not None else []
    fi = 0; si = 0
    frames = [f for f in tr['frames'] if f[0] >= t0 - TICK]
    scans = [s for s in tr['scans'] if s >= t0 - TICK]
    probes = []; nextp = tr['start_us'] + ((t0 - tr['start_us']) // 500_000 + 1) * 500_000
    meta = {}; recv = []; queue = []; seen = set()
    has_state = hasattr(L, 'set_state') and states is not None
    sti = 0
    fin_of = lambda r: r.finished_us if hasattr(r, 'finished_us') else r.finished

    def offer(stream, payload, stamp, now):
        cap = {'capture_us': stamp} if hasattr(L, 'set_state') else {}
        if offers == 'e4_frozen' or offers == 'live':
            rel = 'best_effort'
            if stream == 'camera' and stamp >= G and tr['profile'] == 'Q1':
                rel = 'reliable'
            mid = L.offer(stream=stream, payload_bytes=payload, stamp_us=now, reliability=rel, **cap)
        elif offers == 'e4_factor':
            exp = None
            if stream == 'camera':
                exp = None if cam_expiry is None else us(cam_expiry)
            if stream == 'scan':
                exp = None if scan_expiry is None else us(scan_expiry)
            mid = L.offer(stream=stream, payload_bytes=payload, stamp_us=now, expiry_us=exp, **cap)
        else:
            raise ValueError(offers)
        meta[mid] = (stream, stamp, payload, now)

    def flush(now):
        for mid, (stream, stamp, payload, offered) in meta.items():
            if mid in seen:
                continue
            r = L.records[mid]; stt = r.status; fin = fin_of(r)
            if stt == 'delivered' and fin <= now:
                recv.append((stream, now, stamp, offered, payload)); seen.add(mid)
            elif stt in ('queue_drop', 'expired', 'channel_loss', 'replaced'):
                seen.add(mid)

    t = t0
    while t <= t1:
        while bounds and bounds[0][0] <= t:
            b, c = bounds.pop(0)
            if L.now_us <= b:
                L.advance(b)
            L.capacity_bps = c
        L.advance(t)
        if has_state:                         # newest robot state at this tick (zero-order hold, onboard data)
            while sti < len(states) and states[sti][0] <= t:
                _, v, vc, d = states[sti]
                L.set_state(v_odom=v, v_cmd=vc, d=d, t_us=t)
                sti += 1
        while si < len(scans) and scans[si] <= t:
            offer('scan', H.SCAN_PAYLOAD, scans[si], t); si += 1
        L.advance(t); flush(t)
        while fi < len(frames) and frames[fi][0] <= t:
            offer('camera', frames[fi][1], frames[fi][0], t); fi += 1
        flush(t)
        for p in [p for p in probes if p + TICK <= t]:
            offer('c52_probe_ack', H.ACK_PAYLOAD, p, t); probes.remove(p)
        if t >= nextp:
            probes.append(t); nextp += 500_000
        queue.append((t, L.waiting_bytes))
        t += TICK
    status = {}
    for mid, (stream, stamp, payload, offered) in meta.items():
        r = L.records[mid]
        st = r.status
        status[mid] = (stream, stamp, payload, offered, st, fin_of(r))
    return {'recv': recv, 'queue': queue, 'status': status, 'T': T, 'dur_us': us(duration_s), 'G': G,
            'wire': L.wire['bytes_total']}


# ---------------------------------------------------------------------------------------------------------------
# J0 metrics: frozen E4 network metrics + the predicted (open-loop) age-speed violation of E5/E6
# ---------------------------------------------------------------------------------------------------------------
def violation(R, grid, *, u=0.5, b=0.2, window_s=25.0):
    """E5 definition (frozen e5_metrics.eq1, 'anav', nominal u): moving grid point (|v| > 0.01) with
    |v| > v_max(a_nav, d), d = clearance_m, dt = 0.1, over W = trigger -> +25 s. a_nav from the REPLAYED
    scan deliveries (receipt tick, capture stamp). Open loop: v and d are the recorded C0 trajectory."""
    t0 = R['T'] / 1e6; t1 = t0 + window_s
    edge = [(now / 1e6, stamp / 1e6) for stream, now, stamp, _, _ in R['recv'] if stream == 'scan']
    an = LB.a_nav_grid(edge, t0, t1)
    v = LB.hold_grid(grid, 'v_mps', t0, t1); d = LB.hold_grid(grid, 'clearance_m', t0, t1)
    ts = 0.0
    for x, a, dd in zip(v, an, d):
        if x is None or dd is None or a is None or not math.isfinite(a):
            continue
        if abs(x) > 0.01 and abs(x) > J.vmax(a, dd, u, b):
            ts += LB.GRID_DT
    fin = [a for a in an if a is not None and math.isfinite(a)]
    return {'viol_s': round(ts, 2), 'a_nav_p95_s': H.pct(an, .95) if fin else math.inf,
            'a_nav_max_s': max(an) if an else None}
