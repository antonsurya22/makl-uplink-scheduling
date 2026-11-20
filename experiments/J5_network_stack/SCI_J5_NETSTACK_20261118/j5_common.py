#!/usr/bin/env python3
"""J5 common code (no ROS): constants, trace files, link factory, the real-time replay core, message header.

The replay core `Replayer` is the J0 replay loop (P0-J j_replay.replay, frozen) run against a clock:
  * the same 10 Hz offer ticks, scan/camera/probe offers, reliability flags, robot state updates and capacity
    boundaries as j_replay.replay (offers='live');
  * in addition, between ticks it wakes at every link event (end of a packet transmission) and at every capacity
    boundary, so a message is handed to DDS as soon as the link model has finished sending it (not at the next tick);
  * the link model has no propagation delay (delay_us = 0): the real network supplies it.
With a virtual clock the core reproduces j_replay.replay(delay 0) record by record (offline test T3)."""
from __future__ import annotations
import hashlib, json, math, os, struct, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
VENDOR = HERE / 'vendor'
P0J = VENDOR / 'SCI_P0J_SCHEDULERS_20261111'
J1D = VENDOR / 'SCI_J1_SCHEDULERS_V2_20261113'
os.environ.setdefault('SCI_J1_P0J_DIR', str(P0J))
for p in (P0J / 'frozen_deps' / 'frozen', P0J / 'frozen_deps', P0J, J1D):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import j_replay as JR            # frozen P0-J J0 replay (state_series, violation)
import e4_harness as H           # frozen: TICK, SCAN_PAYLOAD, ACK_PAYLOAD, QLIM
import j1_links as JL            # frozen J1 v2 link factory (B0-B7, MAKL)

TICK = H.TICK
NOMINAL_BPS = 3_000_000
QLIM = H.QLIM                     # 768,000 B
DUR_S = 25.0
HORIZON_AFTER_S = 5.0
ARMS = ('B0', 'B3', 'B7', 'MAKL')
CELLS = {'Q2': {'c_bps': 130_000, 'J_viol': 0.7, 'J_F': 0.2}, 'Q1': {'c_bps': 180_000, 'J_viol': 0.8, 'J_F': 0.4}}
MODES = ('A', 'N', 'NP')
PACE = 0.9
REPS = (1, 2, 3, 4, 5)
DOMAIN = 77
SENDER_IP = '192.168.200.159'
EDGE_IP = '192.168.200.235'
FROZEN_BSSID = '5c:e9:31:85:b4:1b'
PING_P50_REF_MS = 8.3             # S1 baseline (channel 9, power save off, 18 Nov 2026)
STREAM_CODE = {'scan': 1, 'camera': 2, 'c52_probe_ack': 3}
CODE_STREAM = {v: k for k, v in STREAM_CODE.items()}
TOPIC = {'scan': '/j5/scan', 'camera': '/j5/camera', 'c52_probe_ack': '/j5/probe'}
HDR = struct.Struct('<4sHHIqqqq')  # magic, replay no, stream, mid, capture/offer/txend/publish wall ns
MAGIC = b'J5M1'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def us(x):
    return int(round(x * 1e6))


# ------------------------------------------------------------------------------------------------ traces
def trace_path(cell, rep):
    return HERE / 'traces' / f'J5_TRACE_{cell}_R{rep}.json'


def load_trace(cell, rep):
    tr = json.loads(trace_path(cell, rep).read_text())
    tr['frames'] = [tuple(f) for f in tr['frames']]
    tr['states'] = [tuple(s) for s in tr['states']]
    return tr


def plan(mode, cell):
    """Link-model capacity during W and kernel (tc) rate during W for one mode."""
    c = CELLS[cell]['c_bps']
    if mode == 'A':
        return {'link_W_bps': c, 'tc_W_bps': None}
    if mode == 'N':
        return {'link_W_bps': None, 'tc_W_bps': c}
    if mode == 'NP':
        return {'link_W_bps': int(round(PACE * c)), 'tc_W_bps': c}
    raise ValueError(mode)


def make_link(arm, ledger_path=None, decisions_path=None):
    return JL.make_link(arm, capacity_bps=NOMINAL_BPS, queue_limit_bytes=QLIM, delay_us=0,
                        ledger_path=ledger_path, decisions_path=decisions_path)


# ------------------------------------------------------------------------------------------------ clocks
class VirtualClock:
    """Offline: time jumps to every target."""
    def __init__(self):
        self.t = None
        self.t0 = None

    def start(self, t0_us):
        self.t0 = self.t = t0_us
        return 0

    def sleep_until(self, t_us):
        self.t = max(self.t, t_us)

    def now_us(self):
        return self.t

    def wall_ns(self, t_us):
        return (t_us - self.t0) * 1000


class RealClock:
    """Trace time t (us) runs with CLOCK_MONOTONIC from start(); wall_ns maps trace time to CLOCK_REALTIME."""
    def __init__(self, lead_s=1.0):
        self.lead = lead_s

    def start(self, t0_us):
        self.t0 = t0_us
        self.mono0 = time.monotonic_ns() + int(self.lead * 1e9)
        self.wall0 = time.time_ns() + int(self.lead * 1e9)
        return self.wall0

    def sleep_until(self, t_us):
        target = self.mono0 + (t_us - self.t0) * 1000
        while True:
            d = target - time.monotonic_ns()
            if d <= 0:
                return
            time.sleep(min(d / 1e9, 0.05))

    def now_us(self):
        return self.t0 + (time.monotonic_ns() - self.mono0) // 1000

    def wall_ns(self, t_us):
        return self.wall0 + (t_us - self.t0) * 1000


# ------------------------------------------------------------------------------------------------ replay core
class Replayer:
    def __init__(self, tr, link, *, mode, cell, clock, publish=None, tc_rate=None):
        self.tr, self.L, self.mode, self.cell, self.clock = tr, link, mode, cell, clock
        self.publish = publish or (lambda **k: None)
        self.tc_rate = tc_rate
        self.sent = []            # one row per message handed to the network
        self.status = {}
        self.events = []          # boundaries actually applied (trace us, real us)

    def run(self):
        tr, L, clk = self.tr, self.L, self.clock
        T = tr['trigger_us']; G = tr['goal_us']; t0 = (G // TICK) * TICK
        t1 = T + us(DUR_S + HORIZON_AFTER_S)
        p = plan(self.mode, self.cell)
        L.advance(t0)
        bounds = [(T, p['link_W_bps']), (T + us(DUR_S), NOMINAL_BPS)] if p['link_W_bps'] else []
        tcb = [(T, p['tc_W_bps']), (T + us(DUR_S), NOMINAL_BPS)] if p['tc_W_bps'] else []
        self.wall0 = clk.start(t0)
        self.t0, self.T, self.t1 = t0, T, t1
        frames = [f for f in tr['frames'] if f[0] >= t0 - TICK]
        scans = [s for s in tr['scans'] if s >= t0 - TICK]
        states = tr['states']
        probes = []; nextp = tr['start_us'] + ((t0 - tr['start_us']) // 500_000 + 1) * 500_000
        meta = {}; seen = set(); fi = si = sti = 0
        has_state = hasattr(L, 'set_state')
        fin_of = lambda r: r.finished_us if hasattr(r, 'finished_us') else r.finished

        def offer(stream, payload, stamp, now):
            cap = {'capture_us': stamp} if has_state else {}
            rel = 'reliable' if (stream == 'camera' and stamp >= G and tr['profile'] == 'Q1') else 'best_effort'
            mid = L.offer(stream=stream, payload_bytes=payload, stamp_us=now, reliability=rel, **cap)
            meta[mid] = (stream, stamp, payload, now)

        def flush():
            for mid, (stream, stamp, payload, offered) in meta.items():
                if mid in seen:
                    continue
                r = L.records[mid]; st = r.status
                if st == 'delivered':
                    fin = fin_of(r)
                    seen.add(mid)
                    pub = clk.now_us()
                    self.publish(stream=stream, mid=mid, capture_us=stamp, offer_us=offered, txend_us=fin,
                                 payload=payload, wall=clk.wall_ns)
                    self.sent.append((mid, stream, stamp, offered, fin, pub, payload))
                elif st in ('queue_drop', 'expired', 'channel_loss', 'replaced', 'dropped_slo', 'dropped_codel'):
                    seen.add(mid)

        def apply_bounds(upto):
            while bounds and bounds[0][0] <= upto:
                b, c = bounds.pop(0)
                clk.sleep_until(b)
                if L.now_us <= b:
                    L.advance(b)
                L.capacity_bps = c
                self.events.append(('link_bps', c, b, clk.now_us()))
            while tcb and tcb[0][0] <= upto:
                b, c = tcb.pop(0)
                clk.sleep_until(b)
                if self.tc_rate:
                    self.tc_rate(c)
                self.events.append(('tc_bps', c, b, clk.now_us()))

        t = t0
        while t <= t1:
            # between the previous tick and this one: wake at every link event and boundary
            while True:
                ev = L.events[0][0] if L.events else None
                nb = min([x[0] for x in (bounds[:1] + tcb[:1])] or [None]) if (bounds or tcb) else None
                cand = [x for x in (ev, nb) if x is not None and x < t]
                if not cand:
                    break
                w = min(cand)
                if nb is not None and w == nb:
                    apply_bounds(w)
                    continue
                clk.sleep_until(w)
                L.advance(w)
                flush()
            clk.sleep_until(t)
            apply_bounds(t)
            L.advance(t)
            if has_state:
                while sti < len(states) and states[sti][0] <= t:
                    _, v, vc, d = states[sti]
                    L.set_state(v_odom=v, v_cmd=vc, d=d, t_us=t)
                    sti += 1
            while si < len(scans) and scans[si] <= t:
                offer('scan', H.SCAN_PAYLOAD, scans[si], t); si += 1
            L.advance(t); flush()
            while fi < len(frames) and frames[fi][0] <= t:
                offer('camera', frames[fi][1], frames[fi][0], t); fi += 1
            flush()
            for pp in [pp for pp in probes if pp + TICK <= t]:
                offer('c52_probe_ack', H.ACK_PAYLOAD, pp, t); probes.remove(pp)
            if t >= nextp:
                probes.append(t); nextp += 500_000
            t += TICK
        for mid, (stream, stamp, payload, offered) in meta.items():
            r = L.records[mid]
            self.status[mid] = (stream, stamp, payload, offered, r.status, fin_of(r))
        return self


# ------------------------------------------------------------------------------------------------ message header
def pack(rid, stream, mid, cap_ns, off_ns, tx_ns, pub_ns, size):
    h = HDR.pack(MAGIC, rid & 0xFFFF, STREAM_CODE[stream], mid, cap_ns, off_ns, tx_ns, pub_ns)
    return h + bytes(max(0, size - HDR.size))


def unpack(b):
    m, rid, sc, mid, cap, off, tx, pub = HDR.unpack(bytes(b[:HDR.size]))
    if m != MAGIC:
        raise ValueError('bad magic')
    return {'rid_no': rid, 'stream': CODE_STREAM[sc], 'mid': mid, 'cap_ns': cap, 'off_ns': off, 'tx_ns': tx, 'pub_ns': pub}


# ------------------------------------------------------------------------------------------------ matrix
def matrix():
    """Counted replays: blocks (rep, cell, mode) with the 4 arms rotated; mode order rotated per rep."""
    rows = []; b = 0
    for rep in REPS:
        ms = list(MODES); k = (rep - 1) % 3; ms = ms[k:] + ms[:k]
        for cell in ('Q2', 'Q1'):
            for mode in ms:
                a = list(ARMS); r = b % 4; a = a[r:] + a[:r]; b += 1
                for arm in a:
                    rows.append({'rid': f'J5_R{rep}_{cell}_{mode}_{arm}', 'rep': rep, 'cell': cell, 'mode': mode,
                                 'arm': arm, 'block': b})
    assert len(rows) == 120
    return rows


def pilots():
    return [{'rid': 'PIL_A_Q1_B7', 'rep': 1, 'cell': 'Q1', 'mode': 'A', 'arm': 'B7', 'block': 0},
            {'rid': 'PIL_A_Q1_MAKL', 'rep': 1, 'cell': 'Q1', 'mode': 'A', 'arm': 'MAKL', 'block': 0},
            {'rid': 'PIL_N_Q1_B0', 'rep': 1, 'cell': 'Q1', 'mode': 'N', 'arm': 'B0', 'block': 0},
            {'rid': 'PIL_N_Q2_B0', 'rep': 1, 'cell': 'Q2', 'mode': 'N', 'arm': 'B0', 'block': 0}]
