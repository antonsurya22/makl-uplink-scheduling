"""Synthetic periodic sources for the P0-J unit tests (exact event times, no ROS).
Scan: 2,944 B every T_s = 0.2 s. Camera Q1: 3,164 B (3.1 kB JPEG + 64 B CDR) every 0.2 s; Q2: 2,764 B every 0.6 s.
Probe ACK: 96 B every 0.5 s (optional). All offered at their capture time (capture = offer)."""
import math
SCAN_B = 2944
CAM = {'Q1': (3164, 200_000), 'Q2': (2764, 600_000)}
ACK_B = 96


def wire(payload, mtu=1472, ovh=28):
    payload = max(1, payload)
    return payload + ovh * ((payload + mtu - 1) // mtu)


def run(link, *, profile='Q2', cap_bps=130_000, horizon_us=30_000_000, probes=False, phase_cam_us=37_000,
        phase_scan_us=0, state_fn=None):
    """Drive `link` with exact arrival events. Returns list of (mid, stream, capture_us, status, finish_us)."""
    link.capacity_bps = cap_bps
    cb, cp = CAM[profile]
    ev = []
    t = phase_scan_us
    while t < horizon_us:
        ev.append((t, 0, 'scan', SCAN_B))
        t += 200_000
    t = phase_cam_us
    while t < horizon_us:
        ev.append((t, 1, 'camera', cb))
        t += cp
    if probes:
        t = 11_000
        while t < horizon_us:
            ev.append((t, 2, 'c52_probe_ack', ACK_B))
            t += 500_000
    ev.sort()
    mids = []
    for t, _, s, b in ev:
        link.advance(t)
        if state_fn is not None:
            st = state_fn(t)
            link.set_state(v_odom=st[0], v_cmd=st[1], d=st[2], t_us=t)
        mids.append((link.offer(stream=s, payload_bytes=b, stamp_us=t), s, t))
    link.advance(horizon_us + 5_000_000)
    out = []
    for mid, s, t in mids:
        r = link.records[mid]
        fin = r.finished if hasattr(r, 'finished') else r.finished_us
        out.append((mid, s, t, r.status, fin))
    return out


def peak_ages(rows, stream='scan', t_from_us=0):
    """Edge age of `stream` just before each delivery after the first: t_k - capture of newest earlier delivery."""
    dl = sorted((fin, cap) for _, s, cap, st, fin in rows if s == stream and st == 'delivered')
    out = []
    newest = None
    for fin, cap in dl:
        if newest is not None and fin >= t_from_us:
            out.append((fin, (fin - newest) / 1e6))
        newest = cap if newest is None else max(newest, cap)
    return out
