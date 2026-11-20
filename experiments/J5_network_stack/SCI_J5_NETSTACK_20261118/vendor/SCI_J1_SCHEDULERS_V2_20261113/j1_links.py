#!/usr/bin/env python3
"""J1 uplink links = the frozen P0-J schedulers (~/ros2_ws/SCI_P0J_SCHEDULERS_20261111/j_links.py, imported unchanged,
package hash-checked by j1_run_all.py) with two J1 additions that do not change any scheduling rule:
  1. J1Link.next_capture_us: the bridge sets the capture stamp (message header) just before its unchanged offer call, so the
     new arms see the true capture time of scans and camera frames (as in the J0 replay). B0-B3 (frozen E6 classes) do not
     use capture times and are built unchanged.
  2. MAKL decisions and CPU times are written line by line to SCI_J1_DECISIONS (JSON lines) instead of a list in memory.
Arms in J1: B0 FIFO, B1 FIFO + 1 s camera expiry, B2 strict scan priority (packet level), B3 keep-last (frozen E6 K0-K3);
B4 FQ-CoDel, B5 max-age-first, B7 priority + keep-last, MAKL (P0-J). MAKL parameters = the P0-J defaults (J0 decision):
peak test, u = 0.5 m/s, b = 0.2 m/s^2, dt = 0.1 s, m = 0.1 s, beta_min = 0.2 s, SLO_m = 1.0 s, v = max(v_odom, v_cmd),
c_est = delivered wire rate over 2 s; d is supplied by the bridge (raw 360-degree /scan minimum - 0.22 m)."""
import json, os, sys, time
from pathlib import Path
P0J = Path(os.environ.get('SCI_J1_P0J_DIR') or (Path.home() / 'ros2_ws/SCI_P0J_SCHEDULERS_20261111'))
for p in (P0J / 'frozen_deps' / 'frozen', P0J / 'frozen_deps', P0J):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import j_links as J

J1_CONFIGS = ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
ROBOT_R = 0.22


class _Sink(list):
    """List stand-in that writes every appended item as one JSON line and keeps nothing in memory."""
    def __init__(self, fh, kind):
        super().__init__()
        self.fh = fh
        self.kind = kind
        self.n = 0

    def append(self, x):
        self.n += 1
        rec = {'cpu_ns': x} if self.kind == 'cpu' else ({'cputhread_ns': x} if self.kind == 'cputhread' else dict(x, kind='decision'))
        self.fh.write(json.dumps(rec, separators=(',', ':')) + '\n')


class J1Link(J.JQdiscLink):
    next_capture_us = None
    cputhread = None

    def _choose(self):
        """v2: also the THREAD CPU time of each MAKL decision (CLOCK_THREAD_CPUTIME_ID; not affected by other threads
        or processes, unlike the wall time measured inside P0-J). The decision itself is unchanged."""
        if self.cputhread is None:
            return super()._choose()
        t0 = time.thread_time_ns()
        r = super()._choose()
        dt = time.thread_time_ns() - t0
        if r is not None:                       # same calls as the P0-J wall-time series (one per message chosen)
            self.cputhread.append(dt)
        return r

    def offer(self, *, stream, payload_bytes, stamp_us, reliability='best_effort', expiry_us='stream', capture_us=None):
        if capture_us is None and self.next_capture_us is not None:
            capture_us = self.next_capture_us
        return super().offer(stream=stream, payload_bytes=payload_bytes, stamp_us=stamp_us, reliability=reliability,
                             expiry_us=expiry_us, capture_us=capture_us)


def is_makl(L):
    return getattr(L, 'jdisc', None) == 'makl'


def make_link(config, *, capacity_bps, queue_limit_bytes, delay_us, seed=771, jitter_us=0, packet_loss=0.0,
              ledger_path=None, decisions_path=None):
    """Same constructor values as the bridge's own uplink. B0-B3 -> frozen e7_link.make_link K0-K3 (via P0-J)."""
    if config not in J1_CONFIGS:
        raise ValueError('J1_UNKNOWN_CONFIG ' + str(config))
    c = J.CONFIGS[config]
    if 'base' in c:
        return J.make_jlink(config, capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes, delay_us=delay_us,
                            seed=seed, jitter_us=jitter_us, packet_loss=packet_loss, ledger_path=ledger_path)
    if jitter_us != 0 or packet_loss != 0:
        raise RuntimeError('J1_REQUIRES_NO_JITTER_NO_LOSS')
    prm = dict(c['params'])
    full = dict(J.MAKL_DEFAULTS if c['jdisc'] == 'makl' else (J.FQ_DEFAULTS if c['jdisc'] == 'fqcodel' else {}))
    full.update(prm)
    header = {'config': config, 'jdisc': c['jdisc'], 'params': prm, 'effective_params': full, 'expiry_us': {},
              'capacity_bps': capacity_bps, 'queue_limit_bytes': queue_limit_bytes, 'delay_us': delay_us,
              'p0j_dir': str(P0J)}
    L = J1Link(ledger=J.K.Ledger(ledger_path, header), capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes,
               delay_us=delay_us, jdisc=c['jdisc'], params=prm)
    L.j_config = config
    L.e7_config = config
    if decisions_path:
        fh = open(decisions_path, 'a', buffering=1)
        fh.write(json.dumps({'header': True, 'config': config, 'effective_params': full}) + '\n')
        L.dlog = _Sink(fh, 'decision')
        L.cpu_ns = _Sink(fh, 'cpu')
        if c['jdisc'] == 'makl':
            L.cputhread = _Sink(fh, 'cputhread')
    return L
