#!/usr/bin/env python3
"""J5 offline tests (no ROS, no network). Usage: test_j5_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. provenance: vendored P0-J matches its freeze; vendored j1_links.py matches the J1 freeze; if the frozen packages
    are on this computer (the VM), the vendored files are byte-identical to them
 1. traces: 10 files, profiles, offers; if the J1 results are on this computer, rebuilding them gives the same bytes
 2. message header round trip
 3. replay core with a virtual clock = frozen j_replay.replay (delay 0) record by record: 4 arms x 2 cells, modes A
    (link at c in W), N (link at 3 Mbit/s) and NP (link at 0.9 c)
 4. plan, matrix (120 replays, 30 blocks, arm and mode rotation), pilots
 5. parsers of tc, chronyc, iw, ping, iperf3 output
 6. analysis helpers: peak ages, J1 rule
 7. end to end in one process (loopback transport, fake system commands, virtual clock): lock, pilots, 30 blocks,
    a block repeated after a BSSID change, a block repeated after a bad baseline ping, edge logs, uploads, analysis
 7b. ROS glue logic (j5_ros) over a stand-in for rclpy: requests, replies, data, file upload, sender restart
 8. every script compiles"""
import hashlib, json, os, py_compile, shutil, sys, tempfile, time
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
WS = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.home() / 'ros2_ws'
sys.path.insert(0, str(HERE))
import j5_common as C
ok = True


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


# 0 ---------------------------------------------------------------------------------------------------------------
z = json.loads((C.P0J / 'P0J_FREEZE_HASHES.json').read_text())['sha256']
check('vendored P0-J = its freeze (and freeze file e5bccfc2)', all(sha(C.P0J / f) == h for f, h in z.items())
      and sha(C.P0J / 'P0J_FREEZE_HASHES.json').startswith('e5bccfc2'))
z1 = json.loads((C.J1D / 'J1_FREEZE_HASHES.json').read_text())
check('vendored j1_links.py = J1 v2 freeze (freeze file befd9065)', z1['sha256']['j1_links.py'] == sha(C.J1D / 'j1_links.py')
      and sha(C.J1D / 'J1_FREEZE_HASHES.json').startswith('befd9065'))
vm_p0j = WS / 'SCI_P0J_SCHEDULERS_20261111'; vm_j1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
if vm_p0j.is_dir() and vm_j1.is_dir():
    same = all(sha(vm_p0j / f) == sha(C.P0J / f) for f in z) and sha(vm_j1 / 'j1_links.py') == sha(C.J1D / 'j1_links.py')
    check('vendored files byte-identical to the frozen packages on this computer', same)
else:
    print('SKIP frozen packages not on this computer (sender): vendored copy checked against its freeze only', flush=True)

# 1 ---------------------------------------------------------------------------------------------------------------
trs = {(c, r): C.load_trace(c, r) for c in ('Q1', 'Q2') for r in C.REPS}
check('10 traces, profile = cell, C0 B7 sources, >= 140 scans', all(t['profile'] == c and '_C0-' + c + '_B7_' in t['source_dir']
      and len(t['scans']) >= 140 for (c, r), t in trs.items()))
j1res = WS / 'results/SCI_J1_SCHEDULERS_V2_20261113/J1_MAIN/trials'
if j1res.is_dir():
    import make_traces as MT
    tmp = Path(tempfile.mkdtemp()); MT.build(j1res, tmp)
    check('traces rebuilt from the J1 results on this computer are byte-identical', all(
        sha(tmp / p.name) == sha(p) for p in sorted((HERE / 'traces').glob('*.json'))))
else:
    print('SKIP J1 results not on this computer: traces checked by the package freeze only', flush=True)

# 2 ---------------------------------------------------------------------------------------------------------------
b = C.pack(7, 'camera', 123, 10, 20, 30, 40, 3000)
h = C.unpack(b)
check('header round trip, message length = payload', len(b) == 3000 and h == {'rid_no': 7, 'stream': 'camera', 'mid': 123,
      'cap_ns': 10, 'off_ns': 20, 'tx_ns': 30, 'pub_ns': 40})

# 3 ---------------------------------------------------------------------------------------------------------------
same = True; n = 0
for cell in ('Q1', 'Q2'):
    tr = trs[(cell, 1)]
    for mode in C.MODES:
        for arm in C.ARMS:
            R = C.Replayer(tr, C.make_link(arm), mode=mode, cell=cell, clock=C.VirtualClock()).run()
            cap = C.plan(mode, cell)['link_W_bps']
            ref = C.JR.replay(tr, C.make_link(arm), capacity_mbps=(cap / 1e6) if cap else None, duration_s=C.DUR_S,
                              states=tr['states'])
            a = {m: (v[0], v[1], v[4], v[5]) for m, v in R.status.items()}
            bb = {m: (v[0], v[1], v[4], v[5]) for m, v in ref['status'].items()}
            same &= a == bb and len(R.sent) == sum(1 for v in a.values() if v[2] == 'delivered'); n += 1
check(f'replay core = frozen j_replay.replay record by record ({n} combinations)', same)
R = C.Replayer(trs[('Q1', 1)], C.make_link('B7'), mode='N', cell='Q1', clock=C.VirtualClock(),
               tc_rate=lambda c: None).run()
check('mode N: tc set to c at the trigger and back to 3 Mbit/s 25 s later; link stays at 3 Mbit/s',
      [e[:3] for e in R.events] == [('tc_bps', 180000, R.T), ('tc_bps', 3000000, R.T + 25_000_000)])
R = C.Replayer(trs[('Q2', 1)], C.make_link('MAKL'), mode='NP', cell='Q2', clock=C.VirtualClock(),
               tc_rate=lambda c: None).run()
check('mode NP: link at 0.9 c and tc at c in W', sorted(e[:2] for e in R.events) ==
      sorted([('link_bps', 117000), ('link_bps', 3000000), ('tc_bps', 130000), ('tc_bps', 3000000)]))

# 4 ---------------------------------------------------------------------------------------------------------------
rows = C.matrix(); blocks = {}
for r in rows:
    blocks.setdefault(r['block'], []).append(r)
modes_by_rep = {rep: [blocks[b][0]['mode'] for b in sorted(blocks) if blocks[b][0]['rep'] == rep][:3] for rep in C.REPS}
check('matrix: 120 replays, 30 blocks of 4 arms, all 4 arm rotations, mode order rotated per rep, unique ids',
      len(rows) == 120 and len(blocks) == 30 and all(sorted(r['arm'] for r in v) == sorted(C.ARMS) for v in blocks.values())
      and len({tuple(r['arm'] for r in v) for v in blocks.values()}) == 4 and len({tuple(v) for v in modes_by_rep.values()}) == 3
      and len({r['rid'] for r in rows}) == 120)
check('plan: A link c; N tc c; NP link 0.9 c + tc c', C.plan('A', 'Q1') == {'link_W_bps': 180000, 'tc_W_bps': None}
      and C.plan('N', 'Q2') == {'link_W_bps': None, 'tc_W_bps': 130000} and C.plan('NP', 'Q1') == {'link_W_bps': 162000, 'tc_W_bps': 180000})

# 5 ---------------------------------------------------------------------------------------------------------------
import j5_sysops as SO
outs = {}
SO.sh = lambda cmd, timeout=20: outs.get(cmd[0] if cmd[0] != 'sudo' else cmd[2], (0, ''))
so = SO.SysOps()
outs[SO.TC] = (0, 'qdisc tbf 8001: root refcnt 2 rate 130Kbit burst 1600b lat 47.1s \n Sent 123456 bytes 789 pkt (dropped 0, overlimits 12 requeues 0) \n backlog 4500b 3p requeues 0\n')
q1 = so.qdisc_stats()
outs[SO.TC] = (0, 'qdisc mq 0: root \n Sent 9876543 bytes 12345 pkt (dropped 0, overlimits 0 requeues 1) \n backlog 0b 0p requeues 1\nqdisc fq_codel 0: parent :4 limit 10240p\n Sent 1 bytes 1 pkt (dropped 0, overlimits 0 requeues 0)\n backlog 0b 0p requeues 0\n')
q2 = so.qdisc_stats()
outs['chronyc'] = (0, 'C0A8C8EB,192.168.200.235,4,1700000000.123,0.000123,-0.000045,0.000210,-12.345,0.001,0.010,0.002,0.0005,2.0,Normal\n')
cl = so.clock()
outs[SO.IW] = (0, 'Connected to 5c:e9:31:85:b4:1b (on wlan0)\n\tSSID: Sweet_Dumpling\n\tfreq: 2452\n\tsignal: -27 dBm\n\ttx bitrate: 72.2 MBit/s\n')
li = so.link_info()
outs['ping'] = (0, ''.join(f'3000 bytes from 192.168.200.235: icmp_seq={i} ttl=64 time={5 + i * 0.1:.2f} ms\n' for i in range(50))
                + '50 packets transmitted, 50 received, 0% packet loss, time 9800ms\n')
pg = so.ping('x')
outs['iperf3'] = (0, json.dumps({'end': {'sum': {'bits_per_second': 2.0e6, 'lost_percent': 0.5, 'jitter_ms': 1.2}}}))
ip = so.iperf('x')
check('parsers: tbf and mq root stats, chronyc, iw link, ping, iperf3',
      q1['kind'] == 'tbf' and q1['sent_bytes'] == 123456 and q1['backlog_bytes'] == 4500 and q2['kind'] == 'mq'
      and q2['sent_bytes'] == 9876543 and q2['backlog_bytes'] == 0 and cl['ref'] == '192.168.200.235' and abs(cl['offset_s'] - 0.000123) < 1e-12
      and li == {'bssid': '5c:e9:31:85:b4:1b', 'freq': 2452, 'signal_dbm': -27, 'tx_mbps': 72.2}
      and pg['n'] == 50 and pg['loss_pct'] == 0.0 and abs(pg['p50_ms'] - 7.5) < 1e-9 and abs(ip['recv_bps'] - 1.99e6) < 1,
      (q1, q2, cl, li, pg, ip))

# 6 ---------------------------------------------------------------------------------------------------------------
import j5_analysis as AN
pk = AN.peaks([(1_000_000, 900_000), (1_200_000, 1_100_000), (1_500_000, 1_000_000), (1_700_000, 1_600_000)], 1_000_000)
check('peak ages: newest capture before each delivery (a late older scan does not lower the reference)',
      pk == [0.3, 0.4, 0.6])
t = AN.clearly_greater({1: 2, 2: 2, 3: 2, 4: 2, 5: 2}, {1: 1, 2: 1, 3: 1, 4: 1, 5: 1.5}, 0.5)
check('J1 rule', t['clearly_greater'] and not AN.clearly_greater({1: 2, 2: 2, 3: 2}, {1: 1, 2: 1, 3: 1}, 0.5)['clearly_greater'])

# 7 ---------------------------------------------------------------------------------------------------------------
import j5_robot_core as RC
from j5_edge_core import EdgeCore


class WallVC(C.VirtualClock):
    base = 1_800_000_000_000_000_000

    def start(self, t0_us):
        super().start(t0_us); WallVC.base += 100_000_000_000; self.wall0 = WallVC.base
        return self.wall0

    def wall_ns(self, t_us):
        return self.wall0 + (t_us - self.t0) * 1000


class FakeSys:
    def __init__(self):
        self.sent = 0; self.bss_bad = set(); self.ping_bad = set(); self.cur = None; self.calls = 0

    def versions(self): return {'kernel': 'k', 'packages': ['p 1']}
    def iface_ip(self): return C.SENDER_IP
    def link_info(self):
        return {'bssid': 'aa:aa:aa:aa:aa:aa' if self.cur in self.bss_bad else C.FROZEN_BSSID, 'freq': 2452, 'signal_dbm': -27}
    def power_save_off(self): return True
    def throttled(self): return 0
    def clock(self): return {'ref': C.EDGE_IP, 'offset_s': 0.0003}
    def env(self): return {'ROS_DOMAIN_ID': '77', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'CYCLONEDDS_URI': '<...wlan0...>'}
    def ping(self, ip, count=50):
        self.calls += 1
        bad = self.calls in self.ping_bad
        return {'n': 50, 'loss_pct': 0.0, 'p50_ms': 30.0 if bad else 8.0, 'p99_ms': 20.0, 'max_ms': 25.0}
    def iperf(self, ip): return {'recv_bps': 1.99e6, 'lost_pct': 0.1}
    def tc_replace(self, r): pass
    def tc_change(self, r): pass
    def tc_delete(self): pass
    def qdisc_stats(self): return {'kind': 'mq', 'sent_bytes': self.sent, 'backlog_bytes': 0}


class Loop:
    def __init__(self, edge, fs):
        self.edge, self.fs = edge, fs; self.seq = 0

    def request(self, kind, body, timeout=30):
        if kind == 'begin':
            self.fs.cur = body['rid']
        return self.edge.on_ctl(kind, json.loads(json.dumps(body, default=str)))

    def send_file(self, rid, name, text):
        return self.edge.on_file(rid, name, text, hashlib.sha256(text.encode()).hexdigest())

    def publish_data(self, stream, b):
        self.fs.sent += len(b) + 120
        h = C.unpack(b)
        self.edge.on_data(b, h['pub_ns'] + 4_000_000)


base = Path(tempfile.mkdtemp()); edge = EdgeCore(base / 'edge_side'); fs = FakeSys(); tp = Loop(edge, fs)
fs.bss_bad = {'J5_R1_Q2_A_B3'}            # first attempt of one block sees another BSSID
clocks = []
def factory():
    c = WallVC(); clocks.append(c); return c
runner = RC.RobotRunner(tp, fs, base / 'robot_side', clock_factory=factory, monitor=False)
runner.now_ns = lambda: clocks[-1].wall_ns(clocks[-1].now_us())
RC.RobotRunner.tc_rate_ok = lambda self, m: True            # needs real tc samples; checked on the Pi (S4)
fs.ping_bad = {3}                                          # third block: first baseline ping bad
import contextlib, io
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    runner.run()
S = json.loads((base / 'robot_side' / 'STATE.json').read_text())
bl = S['blocks']
check('end to end: pilots passed, 30 blocks valid, 120 counted replays (+4 invalid first attempts kept on record)',
      S['pilots']['passed'] and all(v['status'] == 'valid' for v in bl.values()) and len(bl) == 30
      and sum(len([a for a in v['attempts'] if a['valid']][-1]['replays']) for v in bl.values()) == 120 and len(S['done']) == 124,
      json.dumps(S['pilots'].get('checks')))
b_bss = [k for k, v in bl.items() if len(v['attempts']) == 2 and v['attempts'][0]['replays']]
b_png = [k for k, v in bl.items() if len(v['attempts']) == 2 and not v['attempts'][0]['replays']]
check('BSSID change -> that block repeated with _a2 (all 4 arms); bad baseline ping -> block repeated', len(b_bss) == 1 and len(b_png) == 1
      and all(r.endswith('_a2') for r in bl[b_bss[0]]['attempts'][1]['replays']), (b_bss, b_png))
V = json.loads((base / 'edge_side' / 'final' / 'J5_VERDICTS.json').read_text())
R5 = (base / 'edge_side' / 'final' / 'J5_REPLAYS.csv').read_text().splitlines()
check('edge: logs and uploads complete; analysis ran on 120 counted replays (repeated block counted once)',
      edge.done and V['n_counted'] == 120 and len(R5) == 121 and (base / 'edge_side' / 'final' / 'J5_PLATFORM.json').is_file(),
      (base / 'edge_side' / 'ANALYSIS_CONSOLE.log').read_text()[-600:])
row = next(l for l in R5[1:] if ',A,B0,' in l or l.split(',')[3:5] == ['A', 'B0'])
check('loopback one-way delay seen by the analysis = 4 ms', '4.0' in row, row[:200])
check('mode A in loopback reproduces the J0 ordering: B0 p95 scan age clearly greater than B7, both cells (T1 part)',
      all(json.loads(l.split(',', 1)[1].strip('"').replace('""', '"'))['clearly_greater'] for l in
          (base / 'edge_side' / 'final' / 'J5_TESTS.csv').read_text().splitlines() if l.startswith('T1_') and '_B0_gt_B7_' in l))

# 7b --------------------------------------------------------------------------------------------------------------
# ROS glue logic with a stand-in for rclpy (asynchronous in-process bus); the real DDS path is checked by pilot S2
import queue, threading, types
saved = {k: sys.modules.get(k) for k in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.executors', 'std_msgs', 'std_msgs.msg', 'j5_ros')}
bus = {}; q = queue.Queue()
def deliver():
    while True:
        cb, m = q.get()
        if cb is None:
            return
        cb(m)
class _Pub:
    def __init__(s, t): s.t = t
    def publish(s, m): [q.put((cb, m)) for cb in bus.get(s.t, [])]
    def get_subscription_count(s): return len(bus.get(s.t, []))
class _Node:
    def __init__(s, name): pass
    def create_publisher(s, typ, t, qos): return _Pub(t)
    def create_subscription(s, typ, t, cb, qos): bus.setdefault(t, []).append(cb)
    def destroy_node(s): pass
class _Ex:
    def add_node(s, n): pass
    def spin(s): pass
    def shutdown(s): pass
class _Msg:
    data = None
m_r = types.ModuleType('rclpy'); m_r.init = lambda: None; m_r.shutdown = lambda: None
m_n = types.ModuleType('rclpy.node'); m_n.Node = _Node
m_q = types.ModuleType('rclpy.qos'); m_q.QoSProfile = lambda **k: k
m_q.ReliabilityPolicy = m_q.HistoryPolicy = m_q.DurabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=3, VOLATILE=4)
m_e = types.ModuleType('rclpy.executors'); m_e.SingleThreadedExecutor = _Ex
m_s = types.ModuleType('std_msgs'); m_m = types.ModuleType('std_msgs.msg')
m_m.String = type('String', (_Msg,), {}); m_m.UInt8MultiArray = type('UInt8MultiArray', (_Msg,), {})
sys.modules.update({'rclpy': m_r, 'rclpy.node': m_n, 'rclpy.qos': m_q, 'rclpy.executors': m_e, 'std_msgs': m_s, 'std_msgs.msg': m_m})
sys.modules.pop('j5_ros', None)
try:
    import j5_ros as JRS
    th = threading.Thread(target=deliver, daemon=True); th.start()
    eb = Path(tempfile.mkdtemp()); ec = EdgeCore(eb, run_analysis=False); JRS.EdgeNode(ec)
    t1 = JRS.RosTransport()
    r_h = t1.request('hello', {}, timeout=5); r_b = t1.request('begin', {'rid': 'X1', 'rid_no': 9}, timeout=5)
    t1.publish_data('scan', C.pack(9, 'scan', 1, 1, 2, 3, time.time_ns(), 3000))
    time.sleep(0.2)
    r_e = t1.request('end', {'rid': 'X1'}, timeout=5); r_f = t1.send_file('X1', 'meta.json', '{"a":1}')
    t2 = JRS.RosTransport()                      # restarted sender: seq 1 again, must not get the cached hello of t1
    r_b2 = t2.request('begin', {'rid': 'X2', 'rid_no': 10}, timeout=5)
    q.put((None, None))
    check('ROS glue (stand-in bus): hello, begin, data, end, file; a restarted sender gets fresh replies',
          r_h['ok'] and r_b == {'ok': True} and r_e['n_total'] == 1 and r_f['ok'] and r_b2 == {'ok': True}
          and (eb / 'sender' / 'X1' / 'meta.json').read_text() == '{"a":1}' and (eb / 'edge' / 'X2.jsonl').is_file(), (r_h, r_e, r_f, r_b2))
finally:
    for k, v in saved.items():
        if v is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = v

# 8 ---------------------------------------------------------------------------------------------------------------
cc = True; tmpc = Path(tempfile.mkdtemp())
for f in sorted(HERE.glob('*.py')) + sorted((HERE / 'tests').glob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(tmpc / (f.name + 'c')))
    except Exception as ex:
        cc = False; print(ex)
check('every script compiles', cc)
print('J5_OFFLINE_TESTS', 'PASS' if ok else 'FAIL', flush=True)
sys.exit(0 if ok else 1)
