#!/usr/bin/env python3
"""J2 link impairments, applied in the same way to every J2 arm (B2, B3 = frozen E6 K2, K3; B4, B5, B7, MAKL = P0-J via
J1). The scheduler is unchanged; only what happens on the wire after a packet is sent changes.

Model (per packet, after its transmission ends at t; nominal arrival t + delay):
  jitter   : arrival = max(previous arrival, t + delay + max(U(-J, +J), -delay)) -> one-way delay in [0, delay + J]
             (never before the end of transmission) and packets stay in order on the wire
  loss     : 'bernoulli' p per packet, or Gilbert-Elliott in time ('ge_time': the channel alternates between a good
             and a bad state with exponential durations, means mean_good_s and mean_bad_s; a packet whose transmission
             ends in the bad state is lost with probability h, never in the good state). A lost packet still used link
             time. The message ends with the bridge
             status 'channel_loss' at the moment its last packet would have arrived (no retransmission; MAC
             retransmissions are assumed inside the residual loss rate).
  capacity : 'square' trace while the link is congested (nominal capacity < 1 Mbit/s): the serializer uses
             nominal * hi and nominal * lo alternately every half_period_s, starting with hi at the moment the bridge set
             the nominal capacity (the congestion boundary); hi + lo = 2, so the mean equals the nominal value. The
             bridge still reads and logs the nominal value, so its phase events are unchanged. A new value applies from
             the next packet that starts (as for every capacity change in the frozen link).
Random draws are COMMON RANDOM NUMBERS: each draw is a hash of (run seed, purpose, stream, message stamp, packet index),
and the Gilbert-Elliott state is a timeline drawn from the run seed alone. So the draw a packet gets does not depend on
how much other traffic the arm sent; all arms of one block (same seed) see the same channel. With all impairments off
the link is identical to the J1 link (unit-tested).

The event loop below is the frozen link_disciplines.DisciplineLink.advance loop (E4 v3) with one added branch for the
'lost' event; everything else is copied unchanged."""
import hashlib, heapq, json, random

CONDITIONS = {  # name -> impairment parameters (frozen in the J2 protocol)
    'none': {},
    'jit10': {'jitter_us': 10_000},
    'jit50': {'jitter_us': 50_000},
    'loss1': {'loss': {'model': 'bernoulli', 'p': 0.01}},
    'loss5': {'loss': {'model': 'bernoulli', 'p': 0.05}},
    'ge': {'loss': {'model': 'ge_time', 'mean_good_s': 3.75, 'mean_bad_s': 0.25, 'h': 0.5}},   # bad 6.25 % of time, mean loss 3.1 %
    'capsq': {'capacity': {'model': 'square', 'hi': 1.0 + 0.07 / 0.13, 'lo': 1.0 - 0.07 / 0.13, 'half_period_s': 1.0}},
}


class ImpairMixin:
    """Mix in before the link class: type('Imp' + cls.__name__, (ImpairMixin, cls), {})."""

    def j2_init(self, params, seed):
        self.j2 = dict(params or {})
        self.j2_seed = seed
        self.j2_last_arr = -1
        self.j2_ge_rng = random.Random(seed)   # Gilbert-Elliott timeline only
        self.j2_ge_edges = [0]                 # state changes; state good on [e0, e1), bad on [e1, e2), ...
        self.j2_ge_idx = 0
        self.j2_pk_sched = {}          # mid -> packets whose arrival was decided
        self.j2_doomed = set()
        self.j2_stats = {'packets': 0, 'lost_packets': 0, 'lost_msgs': 0, 'jitter_abs_us_sum': 0, 'reordered_clamped': 0,
                         'cap_changes': 0, 'bad_state_packets': 0}
        self.j2_cap_t0 = None
        self.j2_cap_last = None

    # ---------------- capacity: the bridge reads and writes the NOMINAL value (its phase events stay exact);
    # ---------------- only the serializer (_start_next) uses the time-varying effective value.
    def __setattr__(self, k, v):
        if k == 'capacity_bps' and hasattr(self, 'j2') and v != self.__dict__.get('capacity_bps'):
            self.j2_cap_t0 = self.now_us                  # congestion / recovery boundary = start of the trace
        object.__setattr__(self, k, v)

    def j2_effective_capacity(self):
        nom = self.__dict__['capacity_bps']
        c = self.j2.get('capacity')
        if not c or nom >= 1_000_000:
            return nom
        k = int((self.now_us - self.j2_cap_t0) // int(c['half_period_s'] * 1e6))
        v = int(round(nom * (c['hi'] if k % 2 == 0 else c['lo'])))
        if v != self.j2_cap_last:
            self.j2_stats['cap_changes'] += 1
            self.j2_cap_last = v
        return v

    def _start_next(self):
        nom = self.__dict__['capacity_bps']
        self.__dict__['capacity_bps'] = self.j2_effective_capacity()
        try:
            super()._start_next()
        finally:
            self.__dict__['capacity_bps'] = nom

    # ---------------- per-packet wire impairments ----------------
    def _j2_u(self, purpose, r, k):
        """Uniform [0, 1) from the run seed and the packet identity (stream, stamp, packet index)."""
        h = hashlib.sha256(f'{self.j2_seed}|{purpose}|{r.stream}|{r.created}|{k}'.encode()).hexdigest()
        return int(h[:13], 16) / float(16 ** 13)

    def j2_ge_bad_at(self, t):
        L = self.j2['loss']; E = self.j2_ge_edges
        while E[-1] <= t:
            mean = L['mean_bad_s'] if len(E) % 2 == 0 else L['mean_good_s']
            E.append(E[-1] + max(1, int(round(self.j2_ge_rng.expovariate(1.0 / mean) * 1e6))))
        lo, hi = 0, len(E) - 1                            # last edge <= t
        while lo < hi:
            m = (lo + hi + 1) // 2
            if E[m] <= t:
                lo = m
            else:
                hi = m - 1
        return lo % 2 == 1

    def _j2_lost(self, r, k, t):
        L = self.j2.get('loss')
        if not L:
            return False
        if L['model'] == 'bernoulli':
            return self._j2_u('loss', r, k) < L['p']
        if L['model'] != 'ge_time':
            raise ValueError(L['model'])
        if self.j2_ge_bad_at(t):
            self.j2_stats['bad_state_packets'] += 1
            return self._j2_u('loss', r, k) < L['h']
        return False

    def _sched(self, t, ev, data):
        if ev != 'arrival' or not hasattr(self, 'j2'):
            return super()._sched(t, ev, data)
        mid = data
        r = self.records[mid]
        self.j2_stats['packets'] += 1
        k = self.j2_pk_sched.get(mid, 0)                  # packet index inside the message
        J = self.j2.get('jitter_us', 0)
        ta = t
        if J:
            ta = t + min(J, int(self._j2_u('jitter', r, k) * (2 * J + 1)) - J)
            ta = max(ta, t - self.delay)                  # never before the end of transmission
            self.j2_stats['jitter_abs_us_sum'] += abs(ta - t)
        if ta < self.j2_last_arr:
            ta = self.j2_last_arr
            self.j2_stats['reordered_clamped'] += 1
        self.j2_last_arr = ta
        if self._j2_lost(r, k, t - self.delay):          # state at the end of transmission
            self.j2_stats['lost_packets'] += 1
            self.j2_doomed.add(mid)
        n = self.j2_pk_sched[mid] = k + 1
        if mid in self.j2_doomed:
            if n == r.total:                              # last packet: the message ends as channel_loss
                self.j2_stats['lost_msgs'] += 1
                return super()._sched(ta, 'lost', mid)
            return None                                   # an earlier packet of a doomed message: nothing to deliver
        return super()._sched(ta, 'arrival', mid)

    def advance(self, target):
        """Frozen DisciplineLink.advance (E4 v3) + the 'lost' branch."""
        if target < self.now_us:
            raise ValueError('backwards')
        while self.events and self.events[0][0] <= target:
            t, _, ev, data = heapq.heappop(self.events); self.now_us = t
            if ev == 'tx_done':
                mid, i, w = data; self.busy = False; self.wire['bytes_total'] += w; r = self.records[mid]
                if r.status == 'queued':
                    self._sched(t + self.delay, 'arrival', mid)
                elif mid in self.j2_doomed:
                    pass
                self._start_next()
            elif ev == 'arrival':
                r = self.records[data]
                if r.status == 'queued':
                    r.got += 1
                    if r.got == r.total:
                        self._finish(r, 'delivered')
            elif ev == 'expire':
                r = self.records[data]
                if r.status == 'queued':
                    self._finish(r, 'expired')
            elif ev == 'lost':
                r = self.records[data]
                if r.status == 'queued':
                    self._finish(r, 'channel_loss')
        self.now_us = target

    def j2_summary(self):
        s = dict(self.j2_stats)
        s['loss_rate_packets'] = round(s['lost_packets'] / s['packets'], 5) if s['packets'] else None
        s['mean_abs_jitter_ms'] = round(s['jitter_abs_us_sum'] / s['packets'] / 1000, 3) if s['packets'] else None
        return s


def impair(link, params, seed):
    """Turn an existing (fresh, no traffic) J1 link into the impaired version of the same class."""
    if link.records or link.events:
        raise RuntimeError('J2_IMPAIR_AFTER_TRAFFIC')
    cls = link.__class__
    link.__class__ = type('Imp' + cls.__name__, (ImpairMixin, cls), {})
    link.j2_init(params, seed)
    link.j2_cap_t0 = link.now_us
    return link


def seed_for(condition, rep, profile):
    """Same seed for all arms of one (condition, start shift, profile) block: paired arms see the same random stream."""
    import hashlib
    return int(hashlib.sha256(f'J2|{condition}|{rep}|{profile}'.encode()).hexdigest()[:12], 16)


def params_json(condition):
    return json.dumps(CONDITIONS[condition], sort_keys=True)
