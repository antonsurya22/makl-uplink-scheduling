#!/usr/bin/env python3
"""J5 receiver ("edge") logic without ROS. Every data message is stamped with the edge clock (CLOCK_REALTIME, synced
by chrony) as the first action of the callback and written as one JSON line to edge/<rid>.jsonl.
Control requests from the sender: hello, begin, end, file, halted, all_done (the last runs the analysis)."""
from __future__ import annotations
import hashlib, json, math, os, subprocess, sys, time
from pathlib import Path
import j5_common as C


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q; f = math.floor(k); c = math.ceil(k)
    return xs[f] if f == c else xs[f] + (xs[c] - xs[f]) * (k - f)


class EdgeCore:
    def __init__(self, base, run_analysis=True):
        self.base = Path(base); (self.base / 'edge').mkdir(parents=True, exist_ok=True)
        (self.base / 'sender').mkdir(parents=True, exist_ok=True)
        self.cur = {}            # rid_no -> (rid, file handle)
        self.unknown = 0
        self.run_analysis = run_analysis
        self.done = False

    def on_data(self, data, recv_ns):
        try:
            h = C.unpack(data)
        except Exception:
            self.unknown += 1; return
        cur = self.cur.get(h['rid_no'])
        if cur is None:
            self.unknown += 1; return
        h['recv_ns'] = recv_ns; h['size'] = len(data)
        cur[1].write(json.dumps(h, separators=(',', ':')) + '\n')

    def on_ctl(self, kind, body):
        if kind == 'hello':
            return {'ok': True, 'host': os.uname().nodename, 'edge_ip_expected': C.EDGE_IP, 'time_ns': time.time_ns()}
        if kind == 'begin':
            rid, rno = body['rid'], body['rid_no']
            for k in [k for k, v in self.cur.items() if v[0] == rid]:
                self.cur.pop(k)[1].close()
            fh = open(self.base / 'edge' / f'{rid}.jsonl', 'w', buffering=1)
            self.cur[rno] = (rid, fh)
            return {'ok': True}
        if kind == 'end':
            rid = body['rid']; rows = []
            for k in [k for k, v in self.cur.items() if v[0] == rid]:
                self.cur.pop(k)[1].close()
            p = self.base / 'edge' / f'{rid}.jsonl'
            if p.is_file():
                rows = [json.loads(l) for l in p.read_text().splitlines() if l.strip()]
            ow = [(r['recv_ns'] - r['pub_ns']) / 1e6 for r in rows]
            n = {}
            for r in rows:
                n[r['stream']] = n.get(r['stream'], 0) + 1
            return {'ok': True, 'n_total': len(rows), 'n': n, 'ow_p50_ms': pct(ow, .5), 'ow_p99_ms': pct(ow, .99),
                    'ow_max_ms': max(ow) if ow else None, 'unknown_total': self.unknown}
        if kind == 'halted':
            (self.base / 'SENDER_HALTED.json').write_text(json.dumps(body) + '\n')
            return {'ok': True}
        if kind == 'all_done':
            (self.base / 'STATE_SENDER.json').write_text(json.dumps(body['state'], indent=1) + '\n')
            out = {'ok': True}
            if self.run_analysis:
                p = subprocess.run([sys.executable, str(C.HERE / 'j5_analysis.py'), str(self.base), str(self.base / 'final')],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                (self.base / 'ANALYSIS_CONSOLE.log').write_text(p.stdout)
                out['analysis_rc'] = p.returncode; out['analysis_tail'] = p.stdout[-1500:]
            self.done = True
            return out
        return {'ok': False, 'why': 'unknown request ' + kind}

    def on_file(self, rid, name, text, sha):
        if '/' in rid or '/' in name or name.startswith('.'):
            return {'ok': False, 'why': 'bad name'}
        d = self.base / 'sender' / rid; d.mkdir(parents=True, exist_ok=True)
        (d / name).write_text(text)
        good = hashlib.sha256(text.encode()).hexdigest() == sha
        return {'ok': good}
