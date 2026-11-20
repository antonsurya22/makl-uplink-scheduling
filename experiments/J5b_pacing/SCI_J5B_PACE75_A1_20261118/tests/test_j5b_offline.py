#!/usr/bin/env python3
"""J5b offline tests (no ROS, no network). Usage: test_j5b_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. the vendored J5 package equals its freeze (981b567d...); if the J5 package is installed on this computer, the
    vendored copy is byte-identical to it
 1. the one changed constant: pace 0.75 c in mode NP; modes A and N unchanged; assumption (A1) holds in Q1
 2. matrix: 40 counted Q1 replays (A and NP75, 5 traces x 4 arms), then 20 Q2 demo replays; rotations; unique ids
 3. replay core in mode NP at 0.75 c = frozen j_replay.replay at 0.135 / 0.0975 Mbit/s (all 4 arms, both cells)
 4. end to end in one process (loopback, fake system commands, virtual clock): versions check against J5, J5 pilots,
    background pilot, 15 blocks, J5b analysis; a version change since J5 stops before LOCK without a persisted halt
 5. every script compiles
The vendored J5 tests (vendor/SCI_J5_NETSTACK_20261118/tests/test_j5_offline.py) are run separately by the scripts."""
import contextlib, hashlib, io, json, py_compile, sys, tempfile, time
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
WS = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.home() / 'ros2_ws'
sys.path.insert(0, str(HERE))
import j5b_common as B
from j5b_common import C
ok = True


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


# 0 ---------------------------------------------------------------------------------------------------------------
z = json.loads((B.J5DIR / 'J5_FREEZE_HASHES.json').read_text())['sha256']
check('vendored J5 package = its freeze (freeze file 981b567d)', all(sha(B.J5DIR / f) == h for f, h in z.items())
      and sha(B.J5DIR / 'J5_FREEZE_HASHES.json').startswith('981b567d'))
inst = WS / 'SCI_J5_NETSTACK_20261118'
if inst.is_dir():
    check('vendored J5 package byte-identical to the J5 package installed here',
          all((inst / f).is_file() and sha(inst / f) == h for f, h in z.items())
          and sha(inst / 'J5_FREEZE_HASHES.json') == sha(B.J5DIR / 'J5_FREEZE_HASHES.json'))
else:
    print('SKIP J5 package not installed on this computer', flush=True)

# 1 ---------------------------------------------------------------------------------------------------------------
check('one constant changed: NP = link 0.75 c + tc c; A and N as in J5; (A1) 8*3000/135000 = 0.178 s < 0.2 s in Q1',
      C.PACE == 0.75 and C.plan('NP', 'Q1') == {'link_W_bps': 135000, 'tc_W_bps': 180000}
      and C.plan('NP', 'Q2') == {'link_W_bps': 97500, 'tc_W_bps': 130000}
      and C.plan('A', 'Q1') == {'link_W_bps': 180000, 'tc_W_bps': None} and C.plan('N', 'Q1') == {'link_W_bps': None, 'tc_W_bps': 180000}
      and 8 * 3000 / 135000 < 0.2 and abs(B.PRED_LINK_S['Q1'] - 0.0914) < 1e-4)

# 2 ---------------------------------------------------------------------------------------------------------------
rows = C.matrix(); blocks = {}
for r in rows:
    blocks.setdefault(r['block'], []).append(r)
cnt = [r for r in rows if r['part'] == 'counted']; demo = [r for r in rows if r['part'] == 'demo']
check('matrix: 15 blocks of 4 arms; 40 counted Q1 (A and NP: 5 traces x 4 arms each) first, then 20 Q2 NP demo; unique ids',
      len(rows) == 60 and len(blocks) == 30 // 2 and all(sorted(r['arm'] for r in v) == sorted(C.ARMS) for v in blocks.values())
      and all(len({r['mode'] for r in v}) == 1 for v in blocks.values())
      and sorted((r['mode'], r['rep'], r['arm']) for r in cnt) == sorted((m, rep, a) for m in ('A', 'NP') for rep in C.REPS for a in C.ARMS)
      and all(r['cell'] == 'Q1' for r in cnt) and all(r['cell'] == 'Q2' and r['mode'] == 'NP' for r in demo)
      and max(r['block'] for r in cnt) < min(r['block'] for r in demo) and len({r['rid'] for r in rows}) == 60)

# 3 ---------------------------------------------------------------------------------------------------------------
same = True
for cell in ('Q1', 'Q2'):
    tr = C.load_trace(cell, 1)
    for arm in C.ARMS:
        R = C.Replayer(tr, C.make_link(arm), mode='NP', cell=cell, clock=C.VirtualClock(), tc_rate=lambda c: None).run()
        ref = C.JR.replay(tr, C.make_link(arm), capacity_mbps=C.CELLS[cell]['c_bps'] * 0.75 / 1e6, duration_s=C.DUR_S,
                          states=tr['states'])
        same &= {m: (v[0], v[1], v[4], v[5]) for m, v in R.status.items()} == \
            {m: (v[0], v[1], v[4], v[5]) for m, v in ref['status'].items()}
check('replay core in NP at 0.75 c = frozen j_replay.replay at 0.75 c, record by record (4 arms x 2 cells)', same)

# 4 ---------------------------------------------------------------------------------------------------------------
import j5b_robot_core as BRC
import j5_robot_core as RC
from j5b_edge_core import J5BEdgeCore


class WallVC(C.VirtualClock):
    base = 1_800_000_000_000_000_000

    def start(self, t0_us):
        super().start(t0_us); WallVC.base += 100_000_000_000; self.wall0 = WallVC.base
        return self.wall0

    def wall_ns(self, t_us):
        return self.wall0 + (t_us - self.t0) * 1000


class FakeSys:
    VER = {'kernel': 'k', 'packages': ['p 1']}

    def __init__(self):
        self.sent = 0; self.cur = None

    def versions(self): return dict(self.VER)
    def iface_ip(self): return C.SENDER_IP
    def link_info(self): return {'bssid': C.FROZEN_BSSID, 'freq': 2452, 'signal_dbm': -27}
    def power_save_off(self): return True
    def throttled(self): return 0
    def clock(self): return {'ref': C.EDGE_IP, 'offset_s': 0.0003}
    def env(self): return {'ROS_DOMAIN_ID': '77', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'CYCLONEDDS_URI': '<...wlan0...>'}
    def ping(self, ip, count=50): return {'n': 50, 'loss_pct': 0.0, 'p50_ms': 8.0, 'p99_ms': 20.0, 'max_ms': 25.0}
    def iperf(self, ip): return {'recv_bps': 1.99e6, 'lost_pct': 0.1}
    def tc_replace(self, r): pass
    def tc_change(self, r): pass
    def tc_delete(self): pass
    def qdisc_stats(self): return {'kind': 'mq', 'sent_bytes': self.sent, 'backlog_bytes': 0}


class Loop:
    def __init__(self, edge, fs, np_delay_ns=0):
        self.edge, self.fs, self.np_delay = edge, fs, np_delay_ns; self.cur = ''

    def request(self, kind, body, timeout=30):
        if kind == 'begin':
            self.cur = body['rid']
        return self.edge.on_ctl(kind, json.loads(json.dumps(body, default=str)))

    def send_file(self, rid, name, text):
        return self.edge.on_file(rid, name, text, hashlib.sha256(text.encode()).hexdigest())

    def publish_data(self, stream, b):
        self.fs.sent += len(b) + 120
        h = C.unpack(b)
        self.edge.on_data(b, h['pub_ns'] + 4_000_000 + (self.np_delay if '_NP75_' in self.cur else 0))


def runner(base, fs, j5_state, np_delay_ns=0):
    edge = J5BEdgeCore(base / 'edge_side', j5_cells=base / 'none.csv')
    clocks = []

    def factory():
        c = WallVC(); clocks.append(c); return c
    r = BRC.J5BRunner(Loop(edge, fs, np_delay_ns), fs, base / 'robot_side', clock_factory=factory, monitor=False, j5_state=j5_state)
    r.now_ns = lambda: clocks[-1].wall_ns(clocks[-1].now_us())
    return r, edge


RC.RobotRunner.tc_rate_ok = lambda self, m: True          # needs real tc samples; checked on the Pi (S4)
base = Path(tempfile.mkdtemp())
j5s = base / 'J5_STATE.json'; j5s.write_text(json.dumps({'lock': {'session': {'versions': FakeSys.VER}}}))
# a version change since J5: stops before LOCK, nothing persisted as halted
fs2 = FakeSys(); fs2.VER = {'kernel': 'k2', 'packages': ['p 1']}
r2, _ = runner(base / 'bad', fs2, j5s)
try:
    with contextlib.redirect_stdout(io.StringIO()):
        r2.run()
    stopped = False
except SystemExit as e:
    stopped = 'J5_NOT_STARTED' in str(e)
st2 = json.loads((base / 'bad' / 'robot_side' / 'STATE.json').read_text()) if (base / 'bad' / 'robot_side' / 'STATE.json').is_file() else {}
check('version change since J5 -> J5_NOT_STARTED before LOCK, no persisted halt', stopped and not st2.get('halted') and not st2.get('lock'))
fs = FakeSys(); r, edge = runner(base / 'good', fs, j5s)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    r.run()
S = json.loads((base / 'good' / 'robot_side' / 'STATE.json').read_text())
check('end to end: versions = J5, LOCK records pace 0.75, J5 pilots passed, background pilot recorded, 15 blocks valid',
      S['versions_vs_j5']['equal'] is True and S['lock']['pace'] == 0.75 and S['pilots']['passed']
      and 'mean_bps' in S['pilots']['background'] and len(S['blocks']) == 15
      and all(v['status'] == 'valid' for v in S['blocks'].values()), json.dumps(S['pilots'].get('background')))
fin = base / 'good' / 'edge_side' / 'final'
V = json.loads((fin / 'J5B_VERDICTS.json').read_text()) if (fin / 'J5B_VERDICTS.json').is_file() else {}
T = {l.split(',', 1)[0]: l for l in (fin / 'J5B_TESTS.csv').read_text().splitlines()[1:]} if (fin / 'J5B_TESTS.csv').is_file() else {}
check('J5b analysis ran: 40 counted + 20 demo; loopback (4 ms network, no queue) -> K2, K3, R1, R2 hold, CONTROL_RESTORED;'
      ' K1 not evaluable without tc samples',
      V.get('n_counted') == 40 and V.get('n_demo') == 20 and V.get('verdict') == 'CONTROL_RESTORED'
      and V.get('K1_kernel_queue_short') is False and V.get('K2_net_delay_small') and V.get('K3_follows_link_model') and 'R1_MAKL_NP75_minus_A_anav' in T and 'R2_MAKL_viol_NP75_gt_A' in T
      and (fin / 'J5B_PILOTS.json').is_file(), (base / 'good' / 'edge_side' / 'ANALYSIS_CONSOLE.log').read_text()[-800:])
R1 = json.loads(T['R1_MAKL_NP75_minus_A_anav'].split(',', 1)[1].strip('"').replace('""', '"')) if T else {}
check('R1 in loopback: measured MAKL rise NP75 - A = the frozen prediction (+0.196 s); limit = prediction + J_age',
      R1.get('n_pairs') == 5 and abs(R1.get('mean_diff', 9) - R1.get('predicted_mean_diff', 0)) < 1e-3
      and abs(R1.get('predicted_mean_diff', 0) - 0.1964) < 1e-3, R1)
fs3 = FakeSys(); r3, _ = runner(base / 'queue', fs3, j5s, np_delay_ns=600_000_000)
with contextlib.redirect_stdout(io.StringIO()):
    r3.run()
V3 = json.loads((base / 'queue' / 'edge_side' / 'final' / 'J5B_VERDICTS.json').read_text())
check('a 0.6 s queue below the scheduler in NP75 -> K2, K3, R1 fail and CONTROL_NOT_RESTORED',
      V3['verdict'] == 'CONTROL_NOT_RESTORED' and not V3['K2_net_delay_small'] and not V3['K3_follows_link_model']
      and not V3['R1_age_restored'], V3)
import make_predictions as MP
tmpp = Path(tempfile.mkdtemp()) / 'p.json'
tmpp.write_text(json.dumps(MP.predict(), sort_keys=True, indent=0) + '\n')
check('frozen predictions file = recomputed from the frozen replay core', sha(tmpp) == sha(HERE / 'predictions' / 'J5B_PREDICTIONS.json'))

# 5 ---------------------------------------------------------------------------------------------------------------
cc = True; tmpc = Path(tempfile.mkdtemp())
for f in sorted(HERE.glob('*.py')) + sorted((HERE / 'tests').glob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(tmpc / (f.name + 'c')))
    except Exception as ex:
        cc = False; print(ex)
check('every J5b script compiles', cc)
print('J5B_OFFLINE_TESTS', 'PASS' if ok else 'FAIL', flush=True)
sys.exit(0 if ok else 1)
