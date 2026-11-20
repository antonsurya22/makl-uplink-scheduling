#!/usr/bin/env python3
"""P0-J: new uplink schedulers for the JNCA experiments (J0-J2), as subclasses of the frozen E4 DisciplineLink.

Physics are the parent's (frozen_deps/link_disciplines.py, SHA256 edebf71a...): integer microseconds, MTU 1472 B
payload + 28 B per packet, one serializer at capacity_bps with ceil rounding, fixed one-way delay, atomic tail-drop
admission against queue_limit_bytes (768,000 B), per-stream expiry. Only the SERVICE ORDER and the DROP rules are new.

Disciplines (config names used in J0/J1):
  B7  'prio_kl'   strict scan priority + keep-last (depth 1) on scan and camera; probe ACKs first.
  B5  'maxage'    keep-last (depth 1) on scan and camera; when the link is free, send the stream whose edge age
                  (sender-side: now - capture time of its newest fully transmitted message) is largest; ties -> scan.
  B4  'fqcodel'   RFC 8290 flow queueing (one flow per stream, new/old flow lists, DRR quantum 1514 B) with
                  RFC 8289 CoDel per flow (target 5 ms, interval 100 ms). Packet level. A CoDel head drop drops
                  the whole message (status 'dropped_codel'). Overflow uses the same shared 768,000 B tail drop as
                  all other arms (RFC 8290 drops from the fattest flow; documented deviation).
  P   'makl'      Algorithm 1 of the manuscript (peak-age test). Variants for J0 only, by parameters:
                  test='wait' (age of the waiting scan), v_mode='odom'|'cap', c_mode='true', thin_astar (MAKL-T),
                  beta_override (unit tests). d comes from the state provider (set_state); the J0 'map' variant
                  only feeds a different d.
  REF 'camfirst_kl' camera before scan, keep-last on both (reference for the unit test beta = infinity).
All new disciplines except B4 serve whole messages without preemption (as Propositions 1-2 and Algorithm 1 assume).
Note: the frozen E6 K2 'prio' serves packets, so a scan can pass between the two packets of a camera frame.

New terminal statuses: 'replaced' (keep-last), 'dropped_codel' (B4), 'dropped_slo' (MAKL: camera frame older than
SLO_m when the link is free). Like E6, every non-bridge status is shown to the bridge as 'queue_drop'; the ledger
keeps the true status.
"""
from __future__ import annotations
import json, math, sys, time
from collections import Counter, deque
from pathlib import Path
HERE = Path(__file__).resolve().parent
for p in (HERE / 'frozen_deps' / 'frozen', HERE / 'frozen_deps'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import link_disciplines as LD          # frozen E4 v3 (edebf71a...)
import e7_link as K                    # frozen E6 (afbcfd98...): K0-K3, Ledger, _LedgerMixin, RecView

PROBE = 'c52_probe_ack'
SCAN = 'scan'
CAM = 'camera'
KEEPLAST_STREAMS = (SCAN, CAM)
BRIDGE_STATUSES = ('delivered', 'queue_drop', 'channel_loss', 'expired')
NEW_DISCIPLINES = ('prio_kl', 'maxage', 'fqcodel', 'makl', 'camfirst_kl')


# ----------------------------------------------------------------------------------------------------------------
# Admissible age (Eq. 4 of the manuscript) and v_max (Eq. 3)
# ----------------------------------------------------------------------------------------------------------------
def astar(v, d, u=0.5, b=0.2, dt=0.1):
    """Largest admissible data age a*(v,d) = (d - v^2/(2b) - u v / b)/(v+u) - dt (may be negative)."""
    if v + u <= 0:
        return math.inf
    return (d - v * v / (2 * b) - u * v / b) / (v + u) - dt


def vmax(a, d, u=0.5, b=0.2, dt=0.1):
    """Eq. (3), identical to the frozen e5_metrics.vmax."""
    k = b * (a + dt) + u
    delta = d - u * (a + dt)
    return 0.0 if delta <= 0 else -k + math.sqrt(k * k + 2 * b * delta)


# ----------------------------------------------------------------------------------------------------------------
# Scheduler
# ----------------------------------------------------------------------------------------------------------------
MAKL_DEFAULTS = dict(u=0.5, b=0.2, dt_react=0.1, m=0.1, beta_min=0.2, slo_m=1.0, test='peak', v_mode='max',
                     v_cap=0.15, c_mode='wire2s', c_window_us=2_000_000, thin_astar=None, beta_override=None)
FQ_DEFAULTS = dict(quantum=1514, target_us=5_000, interval_us=100_000, maxpacket=1500, target_mode='fixed')


class JLink(LD.DisciplineLink):
    """Message-level per-stream scheduler (B5, B7, MAKL, REF) and packet-level FQ-CoDel (B4)."""

    def __init__(self, *, capacity_bps, queue_limit_bytes, delay_us, jdisc, expiry_us=None, params=None,
                 decision_log=True, mtu=1472, overhead=28):
        if jdisc not in NEW_DISCIPLINES:
            raise ValueError('J_UNKNOWN_DISCIPLINE ' + str(jdisc))
        # The parent only accepts its four names; 'prio' is a placeholder, every parent method that depends on
        # self.disc (_push/_pop/_queues_empty) is overridden below.
        super().__init__(capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes, delay_us=delay_us,
                         discipline='prio', expiry_us=expiry_us, mtu=mtu, overhead=overhead)
        self.disc = 'j:' + jdisc
        self.jdisc = jdisc
        p = dict(MAKL_DEFAULTS if jdisc in ('makl',) else {})
        if jdisc == 'fqcodel':
            p = dict(FQ_DEFAULTS)
        p.update(params or {})
        self.p = p
        self.mq = {}                 # stream -> deque of message ids (whole messages)
        self.pk = {}                 # mid -> deque of packets (mid, idx, wire) not yet started
        self.cur = None              # message in service (message-level disciplines)
        self.capture_us = {}         # mid -> capture time (defaults to offer time)
        self.sent_cap = {}           # stream -> capture time of the newest FULLY TRANSMITTED message (sender log)
        self.tau_sent = None         # MAKL: capture time of the last scan sent
        self.txw = deque()           # (tx_end_us, wire) for the delivered-wire-rate estimator
        self.state = {'v_odom': 0.0, 'v_cmd': 0.0, 'd': 0.0, 't_us': None}   # conservative until first update
        self.dlog = [] if decision_log else None
        self.cpu_ns = []
        self.decisions = Counter()
        # FQ-CoDel state per flow
        self.fq = {}                 # stream -> dict(q=deque packets, deficit, codel state)
        self.new_flows = deque()
        self.old_flows = deque()

    # ------------------------------------------------------------------ state from the robot (onboard only)
    def set_state(self, *, v_odom=None, v_cmd=None, d=None, t_us=None):
        if v_odom is not None:
            self.state['v_odom'] = abs(float(v_odom))
        if v_cmd is not None:
            self.state['v_cmd'] = abs(float(v_cmd))
        if d is not None:
            self.state['d'] = float(d)
        self.state['t_us'] = t_us

    # ------------------------------------------------------------------ helpers
    def _wire_of(self, mid):
        return sum(w for _, _, w in self.pk.get(mid, ()))

    def _drop_msg(self, mid, status):
        """Remove a queued message (not started) and finish it with `status`."""
        rest = self.pk.pop(mid, None)
        if rest:
            self.waiting_bytes -= sum(w for _, _, w in rest)
        self._finish(self.records[mid], status)

    def _sched(self, t, ev, data):
        if ev == 'tx_done':
            mid, i, w = data
            self.txw.append((t, w))
            r = self.records[mid]
            if i == r.total - 1:     # last packet of the message leaves the sender at t
                s = r.stream
                c = self.capture_us.get(mid, r.created)
                if s not in self.sent_cap or c > self.sent_cap[s]:
                    self.sent_cap[s] = c
        super()._sched(t, ev, data)

    def c_est(self):
        """Delivered wire rate over the last window (Eq. 3 of v11.1); 'true' = oracle (J0 sensitivity only)."""
        if self.p.get('c_mode') == 'true':
            return float(self.capacity_bps)
        now = self.now_us
        win = self.p.get('c_window_us', 2_000_000)
        while self.txw and self.txw[0][0] <= now - win:
            self.txw.popleft()
        tot = sum(w for t, w in self.txw if t <= now)
        return 8.0 * tot * 1e6 / win

    # ------------------------------------------------------------------ parent hooks
    def _queues_empty(self):
        if self.jdisc == 'fqcodel':
            return not any(f['q'] for f in self.fq.values())
        return self.cur is None and not any(self.mq.get(s) for s in self.mq)

    def offer(self, *, stream, payload_bytes, stamp_us, reliability='best_effort', expiry_us='stream',
              capture_us=None):
        # keep-last: replace this stream's older message that has not started (message-level disciplines)
        if self.jdisc in ('prio_kl', 'maxage', 'makl', 'camfirst_kl') and stream in KEEPLAST_STREAMS:
            for old in list(self.mq.get(stream, ())):
                r2 = self.records[old]
                if r2.status == 'queued' and not r2.started:
                    self.mq[stream].remove(old)
                    self._drop_msg(old, 'replaced')
        nid = self.next_id + 1
        self.capture_us[nid] = stamp_us if capture_us is None else capture_us
        mid = super().offer(stream=stream, payload_bytes=payload_bytes, stamp_us=stamp_us, expiry_us=expiry_us)
        if mid != nid:
            raise RuntimeError('J_ID_MISMATCH')
        return mid

    def _push(self, stream, pk):
        mid = pk[0]
        if self.jdisc == 'fqcodel':
            f = self.fq.get(stream)
            if f is None:
                f = self.fq[stream] = dict(q=deque(), deficit=0, first_above=0, dropping=False, count=0,
                                           lastcount=0, drop_next=0)
            if not f['q'] and stream not in self.new_flows and stream not in self.old_flows:
                self.new_flows.append(stream)
                f['deficit'] = self.p['quantum']
            f['q'].append((pk, self.now_us))
            return
        if mid not in self.pk:
            self.pk[mid] = deque()
            self.mq.setdefault(stream, deque()).append(mid)
        self.pk[mid].append(pk)

    def _pop(self):
        if self.jdisc == 'fqcodel':
            return self._fq_dequeue()
        while True:
            if self.cur is not None:
                q = self.pk.get(self.cur)
                if q and self.records[self.cur].status == 'queued':
                    pk = q.popleft()
                    if not q:
                        self.pk.pop(self.cur, None)
                        self.cur = None
                    return pk
                if q:                                   # expired while in service: parent skips its packets
                    pk = q.popleft()
                    if not q:
                        self.pk.pop(self.cur, None)
                        self.cur = None
                    return pk
                self.pk.pop(self.cur, None)
                self.cur = None
            self._purge()
            t0 = time.perf_counter_ns()
            mid = self._choose()
            dt = time.perf_counter_ns() - t0
            if mid is None:
                return None
            if self.jdisc == 'makl':
                self.cpu_ns.append(dt)
            self.cur = mid

    def _purge(self):
        """Drop from the queues messages that ended while waiting (expired); their bytes leave waiting_bytes."""
        for s, q in self.mq.items():
            for mid in list(q):
                if self.records[mid].status != 'queued':
                    q.remove(mid)
                    rest = self.pk.pop(mid, None)
                    if rest:
                        self.waiting_bytes -= sum(w for _, _, w in rest)

    def _head(self, stream):
        q = self.mq.get(stream)
        return q[0] if q else None

    def _take(self, stream):
        return self.mq[stream].popleft()

    def _others(self):
        return [s for s in self.mq if s not in (PROBE, SCAN, CAM) and self.mq[s]]

    # ------------------------------------------------------------------ choice per discipline
    def _choose(self):
        if self._head(PROBE) is not None:
            self.decisions['probe'] += 1
            return self._take(PROBE)
        j = self.jdisc
        if j == 'prio_kl':
            order = [SCAN, CAM] + self._others()
        elif j == 'camfirst_kl':
            order = [CAM, SCAN] + self._others()
        elif j == 'maxage':
            hs, hm = self._head(SCAN), self._head(CAM)
            if hs is not None and hm is not None:
                now = self.now_us
                age = lambda s: (now - self.sent_cap[s]) if s in self.sent_cap else math.inf
                order = [SCAN, CAM] if age(SCAN) >= age(CAM) else [CAM, SCAN]
            else:
                order = [SCAN, CAM]
            order += self._others()
        elif j == 'makl':
            return self._makl()
        else:
            raise RuntimeError('J_NO_CHOICE')
        for s in order:
            if self._head(s) is not None:
                self.decisions[s] += 1
                return self._take(s)
        return None

    def _makl(self):
        p = self.p
        now = self.now_us
        hm = self._head(CAM)
        if hm is not None and now - self.capture_us.get(hm, self.records[hm].created) >= p['slo_m'] * 1e6:
            self.mq[CAM].popleft()
            self._drop_msg(hm, 'dropped_slo')
            hm = None
        hs = self._head(SCAN)
        st = self.state
        if p['v_mode'] == 'max':
            v = max(st['v_odom'], st['v_cmd'])
        elif p['v_mode'] == 'odom':
            v = st['v_odom']
        elif p['v_mode'] == 'cap':
            v = p['v_cap']
        else:
            raise ValueError('J_V_MODE')
        a = astar(v, st['d'], p['u'], p['b'], p['dt_react'])
        beta = max(p['beta_min'], a - p['m']) if p['beta_override'] is None else p['beta_override']
        c = self.c_est()
        peak = None
        choice = None
        if hs is not None:
            ls = self._wire_of(hs)
            lm = self._wire_of(hm) if hm is not None else 0
            if p['test'] == 'peak':
                ref = self.tau_sent
            else:                       # 'wait': age of the waiting scan (brief version)
                ref = self.capture_us.get(hs, self.records[hs].created)
            if ref is None or c <= 0:
                peak = math.inf
            else:
                peak = (now - ref) / 1e6 + 8.0 * (lm + ls) / c + self.delay / 1e6
            if hm is None or peak > beta:
                thin = p['thin_astar']
                if (hm is None and thin is not None and a > thin and self.tau_sent is not None
                        and (now - self.tau_sent) / 1e6 < beta / 2):
                    choice = None       # MAKL-T: hold the scan; keep-last replaces it with the next one
                else:
                    choice = SCAN
            else:
                choice = CAM
        elif hm is not None:
            choice = CAM
        if choice is None and hs is None and hm is None:
            oth = self._others()
            if oth:
                self.decisions[oth[0]] += 1
                return self._take(oth[0])
        if self.dlog is not None and (hs is not None or hm is not None):
            self.dlog.append({'t_us': now, 'choice': choice, 'astar': round(a, 4), 'beta': round(beta, 4),
                              'peak': (round(peak, 4) if peak is not None and math.isfinite(peak) else peak),
                              'c_est_bps': round(c), 'v': round(v, 4), 'd': round(st['d'], 4),
                              'q_scan': hs is not None, 'q_cam': hm is not None})
        if choice is None:
            self.decisions['hold'] += 1
            return None
        self.decisions[choice] += 1
        mid = self._take(choice)
        if choice == SCAN:
            self.tau_sent = self.capture_us.get(mid, self.records[mid].created)
        return mid

    # ------------------------------------------------------------------ FQ-CoDel (RFC 8290 + RFC 8289)
    def _target(self):
        t = self.p['target_us']
        if self.p.get('target_mode') == 'mtu':
            t = max(t, math.ceil(8 * self.p['maxpacket'] * 1e6 / self.capacity_bps))
        return t

    def _codel_dodequeue(self, f, now):
        while f['q']:
            pk, tin = f['q'].popleft()
            if self.records[pk[0]].status != 'queued':
                self.waiting_bytes -= pk[2]     # packet of a message that already ended: discard, no link time
                continue
            fb = sum(x[0][2] for x in f['q'])  # bytes still queued in this flow
            soj = now - tin
            ok = False
            if soj < self._target() or fb <= self.p['maxpacket']:
                f['first_above'] = 0
            else:
                if f['first_above'] == 0:
                    f['first_above'] = now + self.p['interval_us']
                elif now >= f['first_above']:
                    ok = True
            return pk, ok, False
        f['first_above'] = 0
        return None, False, False

    def _control_law(self, t, count):
        return t + int(self.p['interval_us'] / math.sqrt(count))

    def _codel_drop(self, pk):
        mid = pk[0]
        r = self.records[mid]
        self.waiting_bytes -= pk[2]
        if r.status == 'queued':
            self._finish(r, 'dropped_codel')
        self.decisions['codel_drop'] += 1

    def _codel_dequeue(self, f, now):
        """RFC 8289 Section 5.5 dequeue(), per flow. Returns a packet or None."""
        pk, ok, _ = self._codel_dodequeue(f, now)
        if pk is None:
            f['dropping'] = False
            return None
        if f['dropping']:
            if not ok:
                f['dropping'] = False
            while f['dropping'] and now >= f['drop_next']:
                self._codel_drop(pk)
                f['count'] += 1
                pk, ok, _ = self._codel_dodequeue(f, now)
                if pk is None:
                    f['dropping'] = False
                    return None
                if not ok:
                    f['dropping'] = False
                else:
                    f['drop_next'] = self._control_law(f['drop_next'], f['count'])
        elif ok:
            self._codel_drop(pk)
            pk, ok, _ = self._codel_dodequeue(f, now)
            f['dropping'] = True
            delta = f['count'] - f['lastcount']
            if delta > 1 and now - f['drop_next'] < 16 * self.p['interval_us']:
                f['count'] = delta
            else:
                f['count'] = 1
            f['drop_next'] = self._control_law(now, f['count'])
            f['lastcount'] = f['count']
        return pk

    def _fq_dequeue(self):
        now = self.now_us
        while True:
            if self.new_flows:
                lst = self.new_flows
            elif self.old_flows:
                lst = self.old_flows
            else:
                return None
            s = lst[0]
            f = self.fq[s]
            if f['deficit'] <= 0:
                f['deficit'] += self.p['quantum']
                lst.popleft()
                self.old_flows.append(s)
                continue
            pk = self._codel_dequeue(f, now)
            if pk is None:
                lst.popleft()
                if lst is self.new_flows:
                    self.old_flows.append(s)
                continue
            f['deficit'] -= pk[2]
            if self.records[pk[0]].status == 'queued':
                self.decisions[s] += 1
            return pk

    def queued_entries(self):
        if self.jdisc == 'fqcodel':
            return [x[0] for f in self.fq.values() for x in f['q']]
        return [pk for q in self.pk.values() for pk in q]


# ----------------------------------------------------------------------------------------------------------------
# Ledger + bridge-facing wrapper (same pattern as the frozen e7_link.QdiscLink)
# ----------------------------------------------------------------------------------------------------------------
class JQdiscLink(K._LedgerMixin, JLink):
    def __init__(self, *, ledger=None, **kw):
        super().__init__(**kw)
        self._ledger_init(ledger)
        self.tx_log = deque(maxlen=1)
        self.dropped = Counter()

    def _queued_entries(self):
        return self.queued_entries()

    def _cam_queued(self):
        return sum(w for mid, i, w in self._queued_entries()
                   if self.records[mid].stream == CAM and self.records[mid].status == 'queued')

    def offer(self, *, stream, payload_bytes, stamp_us, reliability='best_effort', expiry_us='stream',
              capture_us=None):
        if reliability != 'best_effort':
            self.reliable_offers += 1
        nid = self.next_id + 1
        self._offer_note(nid, stream, payload_bytes, self._cam_queued(), reliability != 'best_effort')
        mid = super().offer(stream=stream, payload_bytes=payload_bytes, stamp_us=stamp_us, expiry_us=expiry_us,
                            capture_us=capture_us)
        self.records[mid].__class__ = K.RecView
        return mid

    def _sched(self, t, ev, data):
        if ev == 'tx_done':
            self._tx_note(data[0], self.now_us)
        super()._sched(t, ev, data)

    def _finish(self, r, st):
        if r.status != 'queued':
            return
        super()._finish(r, st)
        self._finish_note(r.mid, st)
        if st not in BRIDGE_STATUSES:
            r.status = 'queue_drop'          # bridge-facing; the ledger keeps the true status
            self.dropped[(r.stream, st)] += 1

    def outcomes(self):
        return dict(Counter(r.status for r in self.records.values()))

    def prune(self, keep):
        live = {pk[0] for pk in self._queued_entries()}
        if self.cur is not None:
            live.add(self.cur)
        for _, _, ev, data in self.events:
            live.add(data[0] if ev == 'tx_done' else data)
        for mid, r in list(self.records.items()):
            if r.status != 'queued' and mid not in live and mid not in keep:
                del self.records[mid]
                self.capture_us.pop(mid, None)


# ----------------------------------------------------------------------------------------------------------------
# Configurations (J0/J1 arm names) and factory
# ----------------------------------------------------------------------------------------------------------------
CONFIGS = {
    'B0': {'base': 'K0'}, 'B1': {'base': 'K1'}, 'B2': {'base': 'K2'}, 'B3': {'base': 'K3'},   # frozen E6 code
    'B4': {'jdisc': 'fqcodel', 'params': {}},
    'B4m': {'jdisc': 'fqcodel', 'params': {'target_mode': 'mtu'}},                            # J0 only
    'B5': {'jdisc': 'maxage', 'params': {}},
    'B7': {'jdisc': 'prio_kl', 'params': {}},
    'MAKL': {'jdisc': 'makl', 'params': {}},
    'MAKL_wait': {'jdisc': 'makl', 'params': {'test': 'wait'}},                               # J0 only
    'MAKL_odom': {'jdisc': 'makl', 'params': {'v_mode': 'odom'}},                             # J0 only
    'MAKL_cap': {'jdisc': 'makl', 'params': {'v_mode': 'cap'}},                               # J0 only
    'MAKL_true_c': {'jdisc': 'makl', 'params': {'c_mode': 'true'}},                           # J0 only
    'MAKL_T': {'jdisc': 'makl', 'params': {'thin_astar': 2.0}},                               # J0 only
    'REF_camfirst': {'jdisc': 'camfirst_kl', 'params': {}},                                   # tests only
}


def make_jlink(config, *, capacity_bps, queue_limit_bytes, delay_us, seed=771, jitter_us=0, packet_loss=0.0,
               ledger_path=None, name='uplink', params=None, expiry_us=None, ledger=True):
    """Same constructor values as the bridge. B0-B3 are built by the frozen e7_link.make_link (unchanged)."""
    if config not in CONFIGS:
        raise ValueError('J_UNKNOWN_CONFIG ' + str(config))
    c = CONFIGS[config]
    if 'base' in c:
        L = K.make_link(c['base'], capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes, delay_us=delay_us,
                        seed=seed, jitter_us=jitter_us, packet_loss=packet_loss, ledger_path=ledger_path, name=name)
        L.j_config = config
        return L
    if jitter_us != 0 or packet_loss != 0:
        raise RuntimeError('J_REQUIRES_NO_JITTER_NO_LOSS')       # J2 adds these with its own tests
    prm = dict(c['params'])
    prm.update(params or {})
    header = {'config': config, 'jdisc': c['jdisc'], 'params': prm, 'expiry_us': expiry_us or {},
              'capacity_bps': capacity_bps, 'queue_limit_bytes': queue_limit_bytes, 'delay_us': delay_us}
    led = K.Ledger(ledger_path, header) if ledger else None
    L = JQdiscLink(ledger=led, capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes, delay_us=delay_us,
                   jdisc=c['jdisc'], expiry_us=expiry_us, params=prm)
    L.j_config = config
    L.e7_config = config
    return L
