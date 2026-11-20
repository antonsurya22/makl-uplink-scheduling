#!/usr/bin/env python3
"""J3 offline tests (no ROS, no Gazebo). Usage: test_j3_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. provenance: E5, E7, P0-J and J1 v2 packages match their freezes;
 1. j3_run_trial.py = J1 v2 j1_run_trial.py + declared edits (byte check);
 2. distance switch on synthetic periodic traffic: with d < threshold it equals B7 record by record; with d >= threshold
    it serves the older head first (never a newer head while an older one waits); ledger conservation; threshold and
    d logged in every decision;
 3. LIVE J3 BRIDGE (ROS stubbed; real bridge class chain + J1 launcher + J3 factory) with an E3-B C0 trace at 0.13 Mbit/s:
    equals the replay with the same link record by record for B7, MAKL and DSW_0.8 / 1.0 / 1.2, with d below and above
    the threshold; the bridge passes d to the switch; ledger and decision headers carry the arm name;
 4. run-all helpers: matrix 75 runs, 15 blocks, 5 rotations, deltas as J1; config, inputs and gate checks;
 5. j3_analysis end to end on synthetic STATEs: designed outcomes SUPPORTED, TUNED_SWITCH_BETTER, THRESHOLD_INSENSITIVE;
 6. every script compiles."""
import contextlib, hashlib, io, json, math, os, py_compile, random, sys, tempfile, types
from collections import Counter
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
WS = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.home() / 'ros2_ws'
os.environ['HOME'] = str(WS.parent)
E5 = WS / 'SCI_E5_PED_SPEED_20261101'; E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'; P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'; E3BRES = WS / 'results/SCI_E3B_EDGE_NAV_20261031/E3B_MAIN'
ok = True
RANGE = [1.12]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


for P, name, pref in ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
                      (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2'), (J1, 'J1_FREEZE_HASHES.json', 'befd9065')):
    z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if sha(P / f) != h]
    check(P.name + ' unchanged', not bad and sha(P / name).startswith(pref), bad)
sys.path.insert(0, str(HERE / 'tests'))
import j3_run_trial_edits as ED
check('j3_run_trial.py = J1 v2 j1_run_trial.py + declared edits', ED.derive((J1 / 'j1_run_trial.py').read_text()) == (HERE / 'j3_run_trial.py').read_text())

# ---------------- ROS stubs (as the J1 test) ----------------
class Any:
    def __init__(self, *a, **k): pass
    def __call__(self, *a, **k): return Any()
    def __getattr__(self, n): return Any()


def stub(name, **kw):
    m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m


class Obj:
    def __init__(self, **k): self.__dict__.update(k)


def Time(us): return Obj(sec=us // 1_000_000, nanosec=(us % 1_000_000) * 1000)


class Header:
    def __init__(self, us=0): self.stamp = Time(us); self.frame_id = 'x'


class LaserScan:
    def __init__(self, us=0):
        self.header = Header(us); self.ranges = [float('inf'), RANGE[0], 3.0]; self.range_min = 0.12; self.range_max = 3.5


class Image:
    def __init__(self, us=0, nbytes=0): self.header = Header(us); self.nbytes = nbytes; self.width = 1; self.height = 1; self.encoding = 'rgb8'


class CompressedImage:
    def __init__(self, header=None, data=b''): self.header = header or Header(); self.format = 'jpeg'; self.data = data


class Msg:
    def __init__(self, data=None, **k): self.data = data; self.header = Header(); self.__dict__.update(k)


def serialize_message(m):
    if isinstance(m, LaserScan): return b'\0' * 2944
    if isinstance(m, CompressedImage): return b'\0' * (len(m.data) + 64)
    return b''


class Log:
    def __getattr__(self, n): return lambda *a, **k: None


class Pub:
    def __init__(self, node, topic): self.node = node; self.topic = topic; self.out = []
    def publish(self, m): self.out.append((self.node.clock_us, m))


class Node:
    def __init__(self, name, **k): self._pubs = {}; self._subs = []
    def create_publisher(self, typ, topic, qos): p = Pub(self, topic); self._pubs[topic] = p; return p
    def create_subscription(self, typ, topic, cb, qos): self._subs.append((topic, cb)); return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return Log()
    def destroy_node(self): pass


POL = Obj(KEEP_LAST=1, BEST_EFFORT=1, RELIABLE=2, VOLATILE=1)
stub('rclpy', init=lambda *a, **k: None, ok=lambda: True, spin_once=lambda *a, **k: None, shutdown=lambda: None)
stub('rclpy.node', Node=Node)
stub('rclpy.qos', QoSProfile=Any, ReliabilityPolicy=POL, DurabilityPolicy=POL, HistoryPolicy=POL, qos_profile_sensor_data='SENSOR')
stub('rclpy.serialization', serialize_message=serialize_message)
stub('sensor_msgs'); stub('sensor_msgs.msg', LaserScan=LaserScan, Image=Image, CompressedImage=CompressedImage)
stub('std_msgs'); stub('std_msgs.msg', String=Msg, Float32=Msg, UInt8=Msg)
stub('rosgraph_msgs'); stub('rosgraph_msgs.msg', Clock=lambda us=0: Obj(clock=Time(us)))
stub('naw_interfaces'); stub('naw_interfaces.msg', NetworkState=lambda: Obj(header=Header()))
stub('nav_msgs'); stub('nav_msgs.msg', Odometry='Odometry')
stub('geometry_msgs'); stub('geometry_msgs.msg', Twist='Twist')
stub('cv2')
for p in (WS / 'GATE_B1_Link_Core', E7 / 'e4_frozen', E7, P0J / 'tests', J1, HERE):
    sys.path.insert(0, str(p))
TOKEN = 'j3' + 'a' * 46
os.environ.update(SCI_SIM_ONLY='YES', SCI_D3_OWNED_EXCLUSIVE='YES', SCI_D3_SCOPE_TOKEN=TOKEN)
import e4_harness as H
import j1_links as JL
import j3_links as L3
import j_synth as SY
import j_replay as RP

# ---------------- 2. distance switch on synthetic traffic ----------------
mk = lambda cfg: L3.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000)
near = lambda t: (0.12, 0.15, 0.5)          # d = 0.5 m: below every threshold
far = lambda t: (0.12, 0.15, 2.5)           # d = 2.5 m: above every threshold
same = True
for cfg in L3.DSW_THRESHOLDS:
    for prof, c in (('Q2', 130_000), ('Q1', 180_000), ('Q1', 130_000)):
        a = SY.run(mk('B7'), profile=prof, cap_bps=c, probes=True, state_fn=near)
        b = SY.run(mk(cfg), profile=prof, cap_bps=c, probes=True, state_fn=near)
        same &= [x[1:] for x in a] == [x[1:] for x in b]
check('DSW with d below the threshold = B7 record by record (3 thresholds x 3 cells)', same)
L = mk('DSW_1.0'); orig = L3.DSWLink._choose; viol = []; ndec = []
def spy(self, _o=orig):
    hs, hm = self._head(L3.SCAN), self._head(L3.CAM)
    r = _o(self)
    if r is not None and hs is not None and hm is not None and r in (hs, hm):
        cap = lambda m: self.capture_us.get(m, self.records[m].created)
        older = hs if cap(hs) <= cap(hm) else hm
        ndec.append(1)
        if r != older:
            viol.append(r)
    return r
L3.DSWLink._choose = spy
rr = SY.run(L, profile='Q1', cap_bps=130_000, probes=True, state_fn=far)
L3.DSWLink._choose = orig
lines = [x for x in L.ledger.lines if not x.get('header')]
cam = sum(1 for x in rr if x[1] == 'camera' and x[3] == 'delivered')
check('DSW with d above the threshold serves the older head first (contested decisions) and delivers camera frames',
      not viol and len(ndec) > 20 and cam > 0, f'contested {len(ndec)}, wrong order {len(viol)}, camera delivered {cam}')
check('DSW ledger conservation (terminal + queued = offered)', len(lines) + sum(1 for x in rr if x[3] == 'queued') == len(rr))
check('DSW decision log carries threshold and d', L.dlog and all(x['d_th'] == 1.0 and x['d'] == 2.5 and x['beta'] == -1.0 for x in L.dlog))

# ---------------- 3. live J3 bridge vs replay ----------------
import j3_bridge as J3B
M = J3B.JB.M
st = json.loads((E3BRES / 'STATE.json').read_text())
tid, d = next((k, v) for k, v in sorted(st['done'].items()) if v['level'] == 'C0' and v['arm'] == 'F-N2')
TR = H.load_trace(E3BRES / 'trials' / Path(d['evidence_dir']).name, dict(d, trial_id=tid))
TICK = H.TICK


def offers_at(tr, t, t_start):
    s = any(t - TICK < x <= t for x in tr['scans']); f = any(t - TICK < x <= t for x, _ in tr['frames'])
    return s or f or (t - TICK - t_start) % 500_000 == 0


def drive(cfg, rng, cap=0.13):
    tr = TR
    t0 = (tr['goal_us'] // TICK) * TICK; t_start = t0 - 300_000
    tr = dict(tr, start_us=t_start, scans=[s for s in tr['scans'] if s > t0 - TICK], frames=[f for f in tr['frames'] if f[0] > t0 - TICK])
    T = ((tr['trigger_us'] + TICK - 1) // TICK) * TICK
    while offers_at(tr, T, t_start):
        T += TICK
    tr['trigger_us'] = T; t1 = T + 30_000_000
    RANGE[0] = rng
    RL = L3.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY)
    states = [(t0 - 10 * TICK, 0.12, 0.15, rng - 0.22)]
    R = RP.replay(tr, RL, capacity_mbps=cap, duration_s=25, offers='live', states=states)
    tmp = Path(tempfile.mkdtemp())
    plan = {'normal_mbps': 3.0, 'congest_mbps': cap, 'recovery_mbps': 3.0, 'congest_after_trigger_s': 0.0, 'recover_after_trigger_s': 25.0}
    os.environ.update(SCI_D3_SCHEDULES_JSON=json.dumps([plan]), SCI_J1_CONFIG=cfg, SCI_J1_LEDGER=str(tmp / 'L.jsonl'),
                      SCI_J1_DECISIONS=str(tmp / 'D.jsonl'))
    sys.argv = ['j3_bridge.py', '--mode', 'Q2', '--capacity-mbps', '3.0', '--queue-kb', '768', '--report-sim-s', '5']
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        node = M.JointC52Bridge(M.parse_args())
    node._make_output = lambda msg: (CompressedImage(header=msg.header, data=b'\0' * (msg.nbytes - 64)), 'compressed')
    subs = dict(node._subs)
    if '/odom' in subs:
        subs['/odom'](Obj(twist=Obj(twist=Obj(linear=Obj(x=0.12))))); subs['/e2_nav_cmd_policy'](Obj(linear=Obj(x=0.15)))
    ep = '1' * 32; gu = '2' * 32; si = fi = 0; t = t_start
    with contextlib.redirect_stdout(out):
        while t <= t1:
            while si < len(tr['scans']) and tr['scans'][si] <= t:
                node.on_scan(LaserScan(tr['scans'][si])); si += 1
            while fi < len(tr['frames']) and tr['frames'][fi][0] <= t:
                node.on_camera(Image(tr['frames'][fi][0], tr['frames'][fi][1])); fi += 1
            node.on_clock(Obj(clock=Time(t)))
            if t == t_start + TICK:
                node.on_executed(Msg(data=5))
                node.on_episode_control(Msg(data=json.dumps({'scope_token': TOKEN, 'event': 'begin', 'episode_id': ep, 'goal_uuid': gu,
                                                             'reset_verified': True, 'goal_accepted': True})))
            if t == T:
                node.on_episode_control(Msg(data=json.dumps({'scope_token': TOKEN, 'event': 'trigger', 'episode_id': ep, 'goal_uuid': gu,
                                                             'pedestrian_trigger_ack': True})))
            t += TICK
    import e7_metrics as M7
    hdr, rows = M7.read_ledger(tmp / 'L.jsonl')
    MAP = {'replaced': 'queue_drop', 'dropped_codel': 'queue_drop', 'dropped_slo': 'queue_drop'}
    rep = sorted((v[0], v[3], v[2], MAP.get(v[4], v[4]), v[5]) for v in R['status'].values() if v[4] != 'queued')
    liv = sorted((r['stream'], r['offer_us'], r['bytes'], MAP.get(r['status'], r['status']), r['finish_us']) for r in rows if r['offer_us'] >= t0)
    same = rep == liv
    log = out.getvalue()
    D = [json.loads(l) for l in (tmp / 'D.jsonl').read_text().splitlines()]
    dec = [x for x in D if x.get('kind') == 'decision']
    hdr_ok = D[0].get('header') and D[0].get('config') == cfg
    good = same and len(rep) > 100 and f'J1_BRIDGE config={cfg} ' in log and hdr.get('config') == cfg and hdr_ok
    if cfg.startswith('DSW_'):
        th = float(cfg[4:]); dd = {round(x['d'], 3) for x in dec if x['t_us'] >= t0}
        good = good and 'makl_inputs=odom' in log and dd == {round(rng - 0.22, 3)} and all(x['d_th'] == th for x in dec)
    check(f'live J3 bridge = replay: {cfg} with d = {rng - 0.22:.2f} m', good,
          f"msgs {len(liv)}/{len(rep)}, decisions {len(dec)}, camera chosen {sum(x['choice'] == 'camera' for x in dec)}")
    return R


for cfg, rng in (('B7', 1.12), ('MAKL', 1.12), ('DSW_0.8', 1.12), ('DSW_1.0', 1.12), ('DSW_1.2', 1.12), ('DSW_1.0', 1.52),
                 ('DSW_1.2', 1.62)):
    drive(cfg, rng)
RANGE[0] = 1.12

# ---------------- 4. run-all helpers ----------------
src = (HERE / 'j3_run_all.py').read_text(); RA = types.ModuleType('j3ra'); RA.__file__ = str(HERE / 'j3_run_all.py')
exec(compile(src.split('def main():')[0], 'j3_run_all', 'exec'), RA.__dict__)
rows = RA.matrix(); blocks = [rows[i:i + 5] for i in range(0, 75, 5)]
check('matrix: 75 runs, 15 blocks of 5 arms, 5 rotations, deltas -0.4 ... +0.4, 3 cells',
      len(rows) == 75 and len(blocks) == 15 and all(sorted(r['config'] for r in b) == sorted(RA.ARMS) and len({(r['cell'], r['rep']) for r in b}) == 1 for b in blocks)
      and len({tuple(r['config'] for r in b) for b in blocks}) == 5 and sorted({r['delta_s'] for r in rows}) == [-0.4, -0.2, 0.0, 0.2, 0.4]
      and {r['cell'] for r in rows} == {'LB-Q1', 'LC-Q1', 'LC-Q2'})


def evs(t, bps):
    return {'network_events': [{'event': {'event': 'trigger', 'trigger_sim_us': t}},
                               {'event': {'event': 'network_phase', 'new_phase': 'congested', 'boundary_sim_us': t, 'new_bps': bps}}],
            'goal_sim_s': 30.0, 'terminal_sim_s': 60.0}


g1, g2 = RA.gates()
good = dict(evs(42_000_000, 130_000), status='SUCCEEDED', a_nav_logged=True, bridge_config_line='J1_BRIDGE config=DSW_0.8 link=DSWLink',
            j3_dsw={'d_th_m': 0.8, 'decisions_W': 60},
            e5={'ledger_config': 'DSW_0.8', 'arrival_error_s': 0.01, 'j1_inputs_odom': 5, 'j1_inputs_cmd': 5, 'j1_inputs_scans': 9,
                'makl_decisions_all': 50, 'makl_decisions_W': 30, 'makl_scan_chosen_W': 20, 'makl_cam_chosen_W': 10, 'event_congested_bps': 130000})
bad1 = json.loads(json.dumps(good)); bad1['j3_dsw']['d_th_m'] = 1.0
bad2 = json.loads(json.dumps(good)); bad2['e5']['j1_inputs_scans'] = 0
mg = json.loads(json.dumps(good)); mg['bridge_config_line'] = 'J1_BRIDGE config=MAKL link=J1Link'; mg['e5']['ledger_config'] = 'MAKL'
mg['e5']['makl_decisions_v_pos_all'] = 9; mg.pop('j3_dsw')
check('gate and input checks: pass when all present; fail on wrong threshold or missing d input; MAKL gate passes',
      all(RA.gate_checks(good, g1).values()) and not all(RA.gate_checks(bad1, g1).values()) and not all(RA.gate_checks(bad2, g1).values())
      and all(RA.gate_checks(mg, g2).values()) and not RA.config_ok(good, 'DSW_1.0') and RA.config_ok(good, 'DSW_0.8'))

# ---------------- 5. analysis on synthetic STATEs ----------------
import j3_analysis as JA
BASE = {'LB-Q1': {'B7': (1.30, 2.28), 'MAKL': (1.42, 3.22), 'DSW_0.8': (2.50, 3.20), 'DSW_1.0': (1.90, 2.72), 'DSW_1.2': (1.80, 2.40)},
        'LC-Q1': {'B7': (2.40, 0.51), 'MAKL': (2.37, 1.36), 'DSW_0.8': (6.40, 2.63), 'DSW_1.0': (2.83, 1.53), 'DSW_1.2': (2.57, 1.05)},
        'LC-Q2': {'B7': (1.62, 0.41), 'MAKL': (1.96, 0.88), 'DSW_0.8': (2.47, 1.35), 'DSW_1.0': (2.03, 0.85), 'DSW_1.2': (1.97, 0.67)}}
stubA = types.ModuleType('e3b_analysis'); stubA.per_run = lambda tid, r, td: {'F_fps': r['_F'], 'P1_s': 19.3, 'V': 0.1, 'S': 0.0, 'a_nav_p95_s': 0.7}
sys.modules['e3b_analysis'] = stubA
rnd = random.Random(5)


def synth(over):
    done = {}
    for row in rows:
        v, f_ = over.get((row['cell'], row['config']), BASE[row['cell']][row['config']])
        done[row['trial_id']] = {'evidence_dir': '/nonexistent', 'j1_config': row['config'], 'j1_cell': row['cell'], 'j1_rep': row['rep'],
                                 'j1_delta_s': row['delta_s'], 'status': 'SUCCEEDED', 'order': len(done) + 1, '_F': f_ + rnd.uniform(-.02, .02),
                                 'e5': {'W_viol_anav_unom_b0.2_s': v + rnd.uniform(-.05, .05), 'W_min_clearance_m': 0.52, 'contact_class': None}}
    return done


for over, want in (({}, 'SUPPORTED'), ({('LC-Q2', 'DSW_1.0'): (1.96, 1.40)}, 'TUNED_SWITCH_BETTER'),
                   ({('LC-Q1', 'DSW_0.8'): (2.40, 1.40), ('LB-Q1', 'DSW_1.2'): (1.42, 3.10), ('LC-Q1', 'DSW_1.2'): (2.40, 1.30)}, 'THRESHOLD_INSENSITIVE')):
    tdir = Path(tempfile.mkdtemp()); (tdir / 'STATE.json').write_text(json.dumps({'done': synth(over)}))
    sys.argv = ['j3_analysis.py', str(tdir / 'STATE.json'), str(tdir / 'out')]
    with contextlib.redirect_stdout(io.StringIO()):
        JA.main()
    V = json.loads((tdir / 'out' / 'J3_VERDICTS.json').read_text())
    check(f'analysis on designed synthetic data -> {want}', V['outcome'] == want and V['n_runs'] == 75 and V['R_B7_MAKL_reproduce_J1'],
          json.dumps({k: V[k] for k in V if k.startswith('H') and not k.endswith('by_cell')}))
# ---------------- 6. compile ----------------
bad = []
for f in sorted(HERE.rglob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(Path(tempfile.mkdtemp()) / 'x.pyc'))
    except Exception as ex:
        bad.append((f.name, repr(ex)))
check('every package script compiles', not bad, bad)
print('J3_OFFLINE_TESTS', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
