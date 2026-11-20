#!/usr/bin/env python3
"""J3 links: the frozen J1 v2 links (j1_links.py, imported unchanged) plus ONE new arm family, the distance switch.
DSW_x (x = threshold in m): depth-1 queues for scan and camera (keep-last, exactly as B7, jdisc 'prio_kl'), probes first;
when the link becomes free:
  d <  x : scan first (= B7, priority + keep-last);
  d >= x : the head with the OLDER capture time first (= keep-last service order, as B3).
d = the same input as MAKL: minimum valid range of the raw /scan message being offered - 0.22 m, supplied by the
unchanged J1 bridge. No speed input, no capacity input. Each decision with a waiting message is logged (same JSON-lines
file as MAKL; 'beta' is written as -1.0 = not applicable, so the frozen J1 metrics can read the file)."""
import json, os, sys
from pathlib import Path
J1 = Path.home() / 'ros2_ws' / 'SCI_J1_SCHEDULERS_V2_20261113'
if str(J1) not in sys.path:
    sys.path.insert(0, str(J1))
import j1_links as JL
J = JL.J
SCAN, CAM, PROBE = J.SCAN, J.CAM, J.PROBE
DSW_THRESHOLDS = {'DSW_0.8': 0.8, 'DSW_1.0': 1.0, 'DSW_1.2': 1.2}
J3_CONFIGS = ('B7', 'MAKL') + tuple(DSW_THRESHOLDS)
_ORIG_MAKE = JL.make_link
_ORIG_IS_MAKL = JL.is_makl


class DSWLink(JL.J1Link):
    d_th = None
    j3_dsw = True

    def _choose(self):
        if self._head(PROBE) is not None:
            self.decisions['probe'] += 1
            return self._take(PROBE)
        hs, hm = self._head(SCAN), self._head(CAM)
        d = self.state['d']
        if d < self.d_th:
            order = [SCAN, CAM]
        elif hs is not None and hm is not None:
            cap = lambda m: self.capture_us.get(m, self.records[m].created)
            order = [SCAN, CAM] if cap(hs) <= cap(hm) else [CAM, SCAN]
        else:
            order = [SCAN, CAM]
        choice = None
        for s in order:
            if self._head(s) is not None:
                choice = s
                break
        if self.dlog is not None and (hs is not None or hm is not None):
            v = max(self.state['v_odom'], self.state['v_cmd'])
            self.dlog.append({'t_us': self.now_us, 'choice': choice, 'beta': -1.0, 'd_th': self.d_th, 'v': round(v, 4),
                              'd': round(d, 4), 'q_scan': hs is not None, 'q_cam': hm is not None})
        if choice is not None:
            self.decisions[choice] += 1
            return self._take(choice)
        for s in self._others():
            self.decisions[s] += 1
            return self._take(s)
        return None


def make_link(config, *, capacity_bps, queue_limit_bytes, delay_us, seed=771, jitter_us=0, packet_loss=0.0,
              ledger_path=None, decisions_path=None):
    if config not in DSW_THRESHOLDS:
        return _ORIG_MAKE(config, capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes, delay_us=delay_us, seed=seed,
                          jitter_us=jitter_us, packet_loss=packet_loss, ledger_path=ledger_path, decisions_path=decisions_path)
    if jitter_us != 0 or packet_loss != 0:
        raise RuntimeError('J3_REQUIRES_NO_JITTER_NO_LOSS')
    th = DSW_THRESHOLDS[config]
    header = {'config': config, 'jdisc': 'prio_kl', 'params': {'d_th_m': th}, 'effective_params': {'d_th_m': th},
              'rule': 'd < d_th: scan first; else older head first; depth-1 queues; probes first', 'expiry_us': {},
              'capacity_bps': capacity_bps, 'queue_limit_bytes': queue_limit_bytes, 'delay_us': delay_us}
    L = DSWLink(ledger=J.K.Ledger(ledger_path, header), capacity_bps=capacity_bps, queue_limit_bytes=queue_limit_bytes,
                delay_us=delay_us, jdisc='prio_kl', params={})
    L.d_th = th
    L.j_config = config
    L.e7_config = config
    if decisions_path:
        fh = open(decisions_path, 'a', buffering=1)
        fh.write(json.dumps({'header': True, 'config': config, 'effective_params': {'d_th_m': th}}) + '\n')
        L.dlog = JL._Sink(fh, 'decision')
        L.cpu_ns = JL._Sink(fh, 'cpu')
    return L


def is_makl(L):
    """The J1 bridge gives the scheduler its onboard inputs (d from the raw scan; v_odom, v_cmd) only if this is true."""
    return _ORIG_IS_MAKL(L) or getattr(L, 'j3_dsw', False)


def install():
    """Patch the J1 link module before the J1 bridge is imported (the bridge looks these names up at run time)."""
    JL.J1_CONFIGS = tuple(JL.J1_CONFIGS) + tuple(DSW_THRESHOLDS)
    JL.make_link = make_link
    JL.is_makl = is_makl
