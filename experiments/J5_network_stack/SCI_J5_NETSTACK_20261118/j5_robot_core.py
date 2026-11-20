#!/usr/bin/env python3
"""J5 sender ("robot") orchestration, without ROS (the transport and the system commands are injected).
Order (protocol J5_PROTOCOL_FREEZE.md sections 6-7):
  LOCK   versions, interface, BSSID, channel, power save, clock, edge reachable -> STATE['lock'] (once)
  PILOTS S2 DDS + data on wlan0, S3 ceiling (iperf3 + one-way delay), S4 tc rate, S5 real-time fidelity,
         S6 MAKL decision time; any failure halts J5 before the counted matrix
  MATRIX 30 blocks x 4 arms = 120 counted replays; a block is repeated once if a baseline ping, a BSSID change, the
         clock or a missing edge log makes it invalid; a block that fails twice is reported missing
  DONE   all_done to the edge (the edge runs the analysis)
Every replay: tc set (modes N/NP), monitors on, ctl begin, real-time replay, drain, tc removed, ctl end (edge summary),
files uploaded to the edge (sent messages, statuses, link ledger, decisions, tc samples, health, meta)."""
from __future__ import annotations
import datetime, hashlib, json, math, os, statistics as st, threading, time
from pathlib import Path
import j5_common as C

DRAIN_S = 2.0
CLOCK_MAX_S = 0.002


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q; f = math.floor(k); c = math.ceil(k)
    return xs[f] if f == c else xs[f] + (xs[c] - xs[f]) * (k - f)


def le(x, lim):
    return x is not None and x <= lim


SESSION_RETRIES = 10


class Halt(SystemExit):
    pass


class Monitor(threading.Thread):
    """Samples BSSID (1 Hz) and the root qdisc of the interface (10 Hz) while a replay runs."""
    def __init__(self, sysops, period_tc=0.1, period_bssid=1.0):
        super().__init__(daemon=True)
        self.sys = sysops; self.stop_ev = threading.Event(); self.tc = []; self.bssid = []
        self.ptc = period_tc; self.pb = period_bssid

    def run(self):
        nb = 0.0
        while not self.stop_ev.is_set():
            t = time.time_ns()
            q = self.sys.qdisc_stats()
            if q:
                self.tc.append(dict(q, wall_ns=t))
            if time.monotonic() >= nb:
                self.bssid.append({'wall_ns': t, **self.sys.link_info()}); nb = time.monotonic() + self.pb
            self.stop_ev.wait(self.ptc)

    def stop(self):
        self.stop_ev.set(); self.join(timeout=3)


class RobotRunner:
    def __init__(self, transport, sysops, base, clock_factory=C.RealClock, monitor=True):
        self.tp, self.sys, self.base = transport, sysops, Path(base)
        self.clock_factory = clock_factory; self.use_monitor = monitor
        self.now_ns = time.time_ns            # tests replace it with the virtual clock
        self.base.mkdir(parents=True, exist_ok=True)
        self.state_p = self.base / 'STATE.json'
        self.st = json.loads(self.state_p.read_text()) if self.state_p.is_file() else \
            {'created': now(), 'done': {}, 'blocks': {}, 'pilots': {}, 'halted': None, 'rid_no': 0}

    # ------------------------------------------------------------------ state
    def save(self):
        t = self.state_p.with_suffix('.new'); t.write_text(json.dumps(self.st, indent=1, default=str) + '\n')
        os.replace(t, self.state_p)

    def halt(self, why, persist=True, **k):
        if not persist:                       # before LOCK nothing is counted: fix the setup and run again
            self.say(json.dumps(k, default=str)[:2000])
            raise Halt('J5_NOT_STARTED ' + why)
        self.st['halted'] = dict(why=why, at=now(), **k); self.save()
        try:
            self.tp.request('halted', {'why': why}, timeout=5)
        except Exception:
            pass
        raise Halt('J5_HALT ' + why)

    def say(self, *a):
        print(*a, flush=True)

    # ------------------------------------------------------------------ checks
    def session_checks(self):
        s = self.sys
        out = {'versions': s.versions(), 'iface_ip': s.iface_ip(), 'link': s.link_info(), 'power_save': s.power_save_off(),
               'throttled': s.throttled(), 'clock': s.clock(), 'env': s.env()}
        ok = {
            'ip': out['iface_ip'] == C.SENDER_IP,
            'bssid': (out['link'] or {}).get('bssid') == C.FROZEN_BSSID,
            'power_save_off': out['power_save'] is True,
            'not_throttled': out['throttled'] in (None, 0),
            'clock': self.clock_ok(out['clock']),
            'env': out['env'].get('ROS_DOMAIN_ID') == str(C.DOMAIN) and out['env'].get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
            and 'wlan0' in (out['env'].get('CYCLONEDDS_URI') or ''),
        }
        lock = self.st.get('lock')
        if lock:
            ok['versions_unchanged'] = out['versions'] == lock['session']['versions']
            ok['channel_unchanged'] = (out['link'] or {}).get('freq') == lock['session']['link'].get('freq')
        out['ok'] = ok
        return out

    def clock_ok(self, c):
        return bool(c) and c.get('ref') == C.EDGE_IP and abs(c.get('offset_s', 1)) <= CLOCK_MAX_S

    def baseline(self):
        r = self.sys.ping(C.EDGE_IP, count=50)
        ok = r is not None and r['loss_pct'] <= 1.0 and r['p50_ms'] <= 2 * C.PING_P50_REF_MS
        return ok, r

    # ------------------------------------------------------------------ one replay
    def replay(self, row, tag):
        rid = row['rid'] + tag
        self.st['rid_no'] += 1; rno = self.st['rid_no']; self.save()
        d = self.base / 'replays' / rid; d.mkdir(parents=True, exist_ok=True)
        tr = C.load_trace(row['cell'], row['rep'])
        link = C.make_link(row['arm'], ledger_path=str(d / 'LEDGER.jsonl'), decisions_path=str(d / 'DECISIONS.jsonl'))
        p = C.plan(row['mode'], row['cell'])
        health = {'clock_before': self.sys.clock()}
        lb = self.sys.link_info()
        if p['tc_W_bps']:
            self.sys.tc_replace(C.NOMINAL_BPS)
        mon = Monitor(self.sys) if self.use_monitor else None
        q0 = self.sys.qdisc_stats()
        if mon:
            mon.start()
        rep = self.tp.request('begin', {'rid': rid, 'rid_no': rno}, timeout=15)
        if not rep or rep.get('ok') is not True:
            if mon:
                mon.stop()
            self.sys.tc_delete()
            return {'rid': rid, 'valid': False, 'why': 'edge did not open the log'}

        def publish(stream, mid, capture_us, offer_us, txend_us, payload, wall):
            self.tp.publish_data(stream, C.pack(rno, stream, mid, wall(capture_us), wall(offer_us), wall(txend_us),
                                               self.now_ns(), payload))
        clk = self.clock_factory()
        R = C.Replayer(tr, link, mode=row['mode'], cell=row['cell'], clock=clk, publish=publish,
                       tc_rate=(self.sys.tc_change if p['tc_W_bps'] else None))
        t_start = time.time_ns()
        R.run()
        time.sleep(DRAIN_S if self.use_monitor else 0)
        if mon:
            mon.stop()
        q1 = self.sys.qdisc_stats()
        if p['tc_W_bps']:
            self.sys.tc_delete()
        health['clock_after'] = self.sys.clock()
        summ = self.tp.request('end', {'rid': rid}, timeout=30) or {}
        la = self.sys.link_info()
        bss = [b.get('bssid') for b in [lb] + (mon.bssid if mon else []) + [la]]
        health.update({'bssids': sorted({b for b in bss if b}), 'tc_sent_bytes_delta':
                       (q1 or {}).get('sent_bytes', 0) - (q0 or {}).get('sent_bytes', 0), 'edge': summ,
                       'wall_start_ns': t_start})
        lat = [(pub - fin) / 1e3 for (_, _, _, _, fin, pub, _) in R.sent]
        health['publish_late_ms'] = {'p50': pct(lat, .5), 'p99': pct(lat, .99), 'max': max(lat) if lat else None}
        valid, why = True, []
        if any(b != C.FROZEN_BSSID for b in bss):
            valid = False; why.append('bssid')
        if not (self.clock_ok(health['clock_before']) and self.clock_ok(health['clock_after'])):
            valid = False; why.append('clock')
        if not summ.get('n_total'):
            valid = False; why.append('edge received nothing')
        meta = {**row, 'rid': rid, 'rid_no': rno, 'tag': tag, 't0_us': R.t0, 'T_us': R.T, 't1_us': R.t1,
                'wall0_ns': R.wall0, 'events': R.events, 'valid': valid, 'why': why, 'health': health, 'plan': p,
                'trace': C.trace_path(row['cell'], row['rep']).name, 'trace_sha256': C.sha(C.trace_path(row['cell'], row['rep']))}
        files = {
            'meta.json': json.dumps(meta, default=str),
            'sent.jsonl': ''.join(json.dumps({'mid': a, 'stream': b, 'cap_us': c, 'off_us': dd, 'tx_us': e, 'pub_us': f,
                                              'payload': g}) + '\n' for a, b, c, dd, e, f, g in R.sent),
            'status.jsonl': ''.join(json.dumps({'mid': m, 'stream': v[0], 'cap_us': v[1], 'payload': v[2], 'off_us': v[3],
                                                'status': v[4], 'fin_us': v[5]}) + '\n' for m, v in R.status.items()),
            'tc.jsonl': ''.join(json.dumps(x) + '\n' for x in (mon.tc if mon else [])),
            'bssid.jsonl': ''.join(json.dumps(x) + '\n' for x in (mon.bssid if mon else [])),
            'LEDGER.jsonl': (d / 'LEDGER.jsonl').read_text() if (d / 'LEDGER.jsonl').is_file() else '',
            'DECISIONS.jsonl': (d / 'DECISIONS.jsonl').read_text() if (d / 'DECISIONS.jsonl').is_file() else '',
        }
        for n, txt in files.items():
            (d / n).write_text(txt)
            r = self.tp.send_file(rid, n, txt)
            if not r or r.get('ok') is not True:
                valid = False; why.append('upload ' + n)
        meta['valid'] = valid; meta['why'] = why
        (d / 'meta.json').write_text(json.dumps(meta, default=str))
        self.tp.send_file(rid, 'meta.json', json.dumps(meta, default=str))
        self.say(f"   {rid}: valid={valid} {why} sent={len(R.sent)} edge={summ.get('n_total')} "
                 f"ow_p50_ms={summ.get('ow_p50_ms')} late_p99_ms={health['publish_late_ms']['p99']} "
                 f"tc_delta={health['tc_sent_bytes_delta']}")
        return meta

    # ------------------------------------------------------------------ stages
    def lock(self):
        if self.st.get('lock'):
            return
        hs = self.tp.request('hello', {}, timeout=60)
        if not hs or not hs.get('ok'):
            self.halt('edge not reachable over DDS (S2): start JALANKAN_J5_EDGE.sh on the VM first', persist=False)
        s = self.session_checks()
        if not all(s['ok'].values()):
            self.halt('session checks failed at lock', persist=False, checks=s['ok'], session=s)
        self.st['lock'] = {'at': now(), 'session': s, 'edge_hello': hs, 'package_freeze': C.sha(C.HERE / 'J5_FREEZE_HASHES.json')
                           if (C.HERE / 'J5_FREEZE_HASHES.json').is_file() else None}
        self.save(); self.say('J5_LOCKED', json.dumps(s['ok']))

    def run_pilots(self):
        P = self.st['pilots']
        if P.get('passed'):
            return
        res = {}
        for row in C.pilots():
            if row['rid'] in P:
                res[row['rid']] = P[row['rid']]; continue
            self.say(f"=== PILOT {row['rid']} (not counted) ===")
            m = self.replay(row, '')
            P[row['rid']] = m; res[row['rid']] = m; self.save()
        ip = self.sys.iperf(C.EDGE_IP)
        P['iperf'] = ip
        a, mk, n1, n2 = (res['PIL_A_Q1_B7'], res['PIL_A_Q1_MAKL'], res['PIL_N_Q1_B0'], res['PIL_N_Q2_B0'])
        pub_bytes = sum(json.loads(l)['payload'] for l in (self.base / 'replays' / 'PIL_A_Q1_B7' / 'sent.jsonl').read_text().splitlines())
        cpu = self.decision_times(self.base / 'replays' / 'PIL_A_Q1_MAKL' / 'DECISIONS.jsonl')
        checks = {
            'S2_dds_and_data_on_wlan0': a['valid'] and a['health']['tc_sent_bytes_delta'] >= 0.9 * pub_bytes,
            'S3_ceiling_iperf_2Mbps': bool(ip) and ip.get('recv_bps', 0) >= 1.9e6 and ip.get('lost_pct', 100) <= 2.0,
            'S3_one_way_p50_le_10ms': le(a['health']['edge'].get('ow_p50_ms'), 10.0),
            'S4_tc_rate_Q1': self.tc_rate_ok(n1), 'S4_tc_rate_Q2': self.tc_rate_ok(n2),
            'S5_realtime_fidelity': le(a['health']['publish_late_ms']['p50'], 10.0) and le(a['health']['publish_late_ms']['p99'], 50.0),
            'S6_makl_decision_p99_le_5ms': cpu.get('thread_p99_ms') is not None and cpu['thread_p99_ms'] <= 5.0,
            'all_pilots_valid': all(x['valid'] for x in (a, mk, n1, n2)),
        }
        P['checks'] = checks; P['decision_time'] = cpu; P['passed'] = all(checks.values()); self.save()
        self.say('J5_PILOTS', json.dumps(checks), json.dumps(cpu))
        if not P['passed']:
            self.halt('pilot failed (amendment required)', checks=checks)

    def tc_rate_ok(self, m):
        """Bytes the qdisc sent during W divided by 25 s, within 5 % of c (needs a backlog, which mode N has)."""
        d = self.base / 'replays' / m['rid']
        tc = [json.loads(l) for l in (d / 'tc.jsonl').read_text().splitlines()] if (d / 'tc.jsonl').is_file() else []
        if not tc:
            return False
        w0 = m['wall0_ns'] + (m['T_us'] - m['t0_us']) * 1000; w1 = w0 + int(C.DUR_S * 1e9)
        a = min(tc, key=lambda x: abs(x['wall_ns'] - w0)); b = min(tc, key=lambda x: abs(x['wall_ns'] - w1))
        rate = 8 * (b['sent_bytes'] - a['sent_bytes']) / ((b['wall_ns'] - a['wall_ns']) / 1e9)
        c = C.CELLS[m['cell']]['c_bps']
        m['tc_rate_W_bps'] = rate
        return abs(rate - c) <= 0.05 * c

    @staticmethod
    def decision_times(p):
        wall = []; thr = []
        if Path(p).is_file():
            for l in Path(p).read_text().splitlines():
                x = json.loads(l)
                if 'cpu_ns' in x:
                    wall.append(x['cpu_ns'] / 1e6)
                if 'cputhread_ns' in x:
                    thr.append(x['cputhread_ns'] / 1e6)
        return {'n': len(thr), 'thread_p50_ms': pct(thr, .5), 'thread_p99_ms': pct(thr, .99), 'thread_max_ms': max(thr) if thr else None,
                'wall_p50_ms': pct(wall, .5), 'wall_p99_ms': pct(wall, .99), 'wall_max_ms': max(wall) if wall else None}

    def run_matrix(self):
        rows = C.matrix(); blocks = {}
        for r in rows:
            blocks.setdefault(r['block'], []).append(r)
        for b, rs in blocks.items():
            B = self.st['blocks'].setdefault(str(b), {'attempts': [], 'status': None})
            if B['status'] in ('valid', 'missing'):
                continue
            while len(B['attempts']) < 2 and B['status'] is None:
                k = len(B['attempts']) + 1; tag = '' if k == 1 else '_a2'
                for w in range(SESSION_RETRIES + 1):          # short drifts (clock, roaming) get time to settle
                    s = self.session_checks()
                    if all(s['ok'].values()):
                        break
                    if w < SESSION_RETRIES:
                        self.say(f'=== block {b}: session check not ok {s["ok"]}; waiting 30 s ({w + 1}/{SESSION_RETRIES}) ===')
                        time.sleep(30 if self.use_monitor else 0)
                else:
                    self.halt('session checks failed', block=b, checks=s['ok'])
                ok, ping = self.baseline()
                att = {'k': k, 'at': now(), 'ping': ping, 'replays': [], 'valid': False}
                if not ok:
                    self.say(f'=== block {b}: baseline ping bad {ping}; waiting 60 s ===')
                    B['attempts'].append(att); self.save()
                    time.sleep(60 if self.use_monitor else 0); continue
                self.say(f"=== block {b} attempt {k}: rep {rs[0]['rep']} {rs[0]['cell']} mode {rs[0]['mode']} "
                         f"arms {[r['arm'] for r in rs]} ping p50 {ping['p50_ms']} ms ===")
                good = True
                for r in rs:
                    m = self.replay(r, tag)
                    att['replays'].append(m['rid']); self.st['done'][m['rid']] = {k2: m.get(k2) for k2 in
                                                                                   ('rid', 'rep', 'cell', 'mode', 'arm', 'block', 'tag', 'valid', 'why')}
                    good &= bool(m.get('valid'))
                    self.save()
                att['valid'] = good; B['attempts'].append(att)
                if good:
                    B['status'] = 'valid'
                self.save()
            if B['status'] is None:
                B['status'] = 'missing'; self.save()
                self.say(f'=== block {b}: missing after two attempts ===')

    def run(self):
        if self.st.get('halted'):
            raise Halt('J5_HALTED_REVIEW_REQUIRED ' + json.dumps(self.st['halted']))
        self.lock(); self.run_pilots(); self.run_matrix()
        self.st['completed'] = now(); self.save()
        rep = self.tp.request('all_done', {'state': self.st}, timeout=600)
        self.say('J5_DONE', json.dumps(rep or {})[:400])
