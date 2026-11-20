"""J5b sender runner = frozen J5 RobotRunner plus two J5b additions (J5B_PROTOCOL_FREEZE.md section 5):
 1. before LOCK: system versions must equal those recorded at the J5 lock (no upgrade since J5);
 2. after the J5 pilots: a 60 s background pilot (all nodes up, nothing replayed) on wlan0, recorded before any counted
    replay: qdisc bytes per second (mean, and the largest 2 s window) and the UDP/TCP datagram counters of the kernel."""
from __future__ import annotations
import json, time
from pathlib import Path
import j5b_common as B
from j5b_common import C
import j5_robot_core as RC

BG_S = 60.0


def snmp():
    try:
        L = Path('/proc/net/snmp').read_text().splitlines()
        out = {}
        for i in range(0, len(L) - 1, 2):
            k = L[i].split(); v = L[i + 1].split()
            if k[0] in ('Udp:', 'Tcp:'):
                d = dict(zip(k[1:], v[1:]))
                out[k[0][:-1]] = {x: int(d[x]) for x in ('OutDatagrams', 'OutSegs') if x in d}
        return out
    except Exception:
        return None


class J5BRunner(RC.RobotRunner):
    def __init__(self, *a, j5_state=None, **k):
        super().__init__(*a, **k)
        self.j5_state = Path(j5_state) if j5_state else Path.home() / B.J5_RESULTS_REL / 'STATE.json'

    def lock(self):
        if not self.st.get('lock'):
            v = self.sys.versions()
            if self.j5_state.is_file():
                j5v = json.loads(self.j5_state.read_text())['lock']['session']['versions']
                ok = v == j5v
                self.st['versions_vs_j5'] = {'j5_state': str(self.j5_state), 'equal': ok}
                if not ok:
                    self.halt('system versions differ from the J5 lock (upgrade since J5?)', persist=False,
                              now=v, j5=j5v)
            else:
                self.st['versions_vs_j5'] = {'j5_state': None, 'equal': None}
            self.save()
        super().lock()
        if not self.st['lock'].get('j5b_freeze'):
            fz = B.HERE / 'J5B_FREEZE_HASHES.json'
            self.st['lock']['j5b_freeze'] = C.sha(fz) if fz.is_file() else None
            self.st['lock']['pace'] = C.PACE; self.save()

    def background(self):
        """60 s, nothing replayed: what else leaves wlan0 (DDS discovery and heartbeats, chrony, SSH, ...)."""
        dur = BG_S if self.use_monitor else 0.0
        s0 = snmp(); q = []; t_end = time.monotonic() + dur
        while True:
            x = self.sys.qdisc_stats()
            if x:
                q.append((time.time_ns(), x['sent_bytes']))
            if time.monotonic() >= t_end:
                break
            time.sleep(0.1)
        s1 = snmp()
        rate = (8 * (q[-1][1] - q[0][1]) / ((q[-1][0] - q[0][0]) / 1e9)) if len(q) > 1 and q[-1][0] > q[0][0] else None
        w2 = []
        for i, (t, b) in enumerate(q):
            j = next((j for j in range(i, len(q)) if q[j][0] - t >= 2e9), None)
            if j is not None:
                w2.append(8 * (q[j][1] - b) / ((q[j][0] - t) / 1e9))
        d = {k: {x: s1[k][x] - s0[k][x] for x in s1[k]} for k in s1} if s0 and s1 else None
        return {'dur_s': dur, 'n_samples': len(q), 'mean_bps': rate, 'max_2s_bps': max(w2) if w2 else None,
                'p90_2s_bps': sorted(w2)[int(0.9 * (len(w2) - 1))] if w2 else None, 'snmp_delta': d}

    def run_pilots(self):
        super().run_pilots()              # the frozen J5 pilots S2-S6 (halts if any fails)
        P = self.st['pilots']
        if not P.get('background'):
            self.say('=== PILOT background (60 s, nothing replayed; not counted) ===')
            P['background'] = self.background(); self.save()
            self.say('J5B_BACKGROUND', json.dumps(P['background']))

    def run(self):
        super().run()
