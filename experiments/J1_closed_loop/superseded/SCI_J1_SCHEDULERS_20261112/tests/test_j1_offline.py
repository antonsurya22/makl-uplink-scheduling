#!/usr/bin/env python3
"""J1 offline tests (no ROS, no Gazebo). Usage: test_j1_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. provenance: E5, E7 (article E6) and P0-J packages match their freezes; the E3-B files J1 builds on match SHA256.json;
 1. j1_run_trial.py = E7 e7_run_trial.py + the declared edits (byte check);
 2. LIVE J1 BRIDGE vs REPLAY: the real bridge class chain (B5 -> C2 -> C4 -> C5.2 -> joint -> J1 launcher, race fix
    included) with ROS stubbed, driven tick by tick with the E3-B C0 F-N2 traces at 0.13 Mbit/s (25 s) and at C0:
    B0-B3 must equal the frozen E4 replay (as the E7 test); B4, B5, B7 and MAKL must equal the P0-J replay
    (j_replay.replay, capture stamps, same constant robot state) record by record (tolerance 1 ms) with the same edge
    receptions. MAKL is checked at two constant states (margin small and large) and must choose both streams;
 3. MAKL inputs: subscriptions to /odom and /e2_nav_cmd_policy exist only for MAKL; d = min valid range - 0.22 m
    (inf/NaN/out-of-range ignored; range_max - 0.22 if none valid); v from odometry and Nav2 command; decisions file has a
    header, decisions and CPU lines; J1_STATE line printed;
 4. capacity plans: every J1 cell plan parses with the frozen TriggeredPlan rules and recovers at +8 s / +25 s; plan_ok
    accepts the right event list and rejects a wrong duration or capacity;
 5. j1_metrics on a recorded E7 trial (ledger renamed): the frozen E5 measures are unchanged; ledger summary, clipped
    a_nav, event duration and the MAKL decision summary work;
 6. j1_analysis end to end on a synthetic STATE (240 runs): verdict logic gives the designed answers;
 7. matrix: 240 unique runs, 30 blocks of 8 arms, 8 rotations, delta per rep; gate rows;
 8. every package script compiles."""
import contextlib, hashlib, importlib, io, json, math, os, py_compile, random, shutil, sys, tempfile, types
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
WS = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.home() / 'ros2_ws'
os.environ['HOME'] = str(WS.parent)
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'; E5 = WS / 'SCI_E5_PED_SPEED_20261101'; E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'
P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
E3BRES = WS / 'results/SCI_E3B_EDGE_NAV_20261031/E3B_MAIN'; E7RES = WS / 'results/SCI_E7_QUEUE_DISCIPLINE_20261102/E7_MAIN'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


ok = True


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


# ---------------- 0. provenance ----------------
for P, name, pref in ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
                      (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2')):
    z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if sha(P / f) != h]
    check(P.name + ' unchanged', not bad and sha(P / name).startswith(pref), bad)
pins = json.loads((E3B / 'SHA256.json').read_text())
used = [f for f in pins if f.startswith('bridge_optional/') or f in ('e3b_policy_logic.py', 'run_trial.py', 'e3b_trial.py')]
check('E3-B files used by J1 match SHA256.json', used and all(sha(E3B / f) == pins[f] for f in used), [f for f in used if sha(E3B / f) != pins[f]])
# ---------------- 1. derivation ----------------
sys.path.insert(0, str(HERE / 'tests'))
import j1_run_trial_edits as ED
check('j1_run_trial.py = E7 e7_run_trial.py + declared edits', ED.derive((E7 / 'e7_run_trial.py').read_text()) == (HERE / 'j1_run_trial.py').read_text())


# ---------------- ROS stubs (as the E7 test) ----------------
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
    RANGES = [3.0]
    def __init__(self, us=0, ranges=None):
        self.header = Header(us); self.ranges = list(LaserScan.RANGES if ranges is None else ranges)
        self.range_min = 0.12; self.range_max = 3.5


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
for p in (WS / 'GATE_B1_Link_Core', E7 / 'e4_frozen', E7, HERE, HERE / 'tests'):
    sys.path.insert(0, str(p))
TOKEN = 'j1' + 'a' * 46
os.environ.update(SCI_SIM_ONLY='YES', SCI_D3_OWNED_EXCLUSIVE='YES', SCI_D3_SCOPE_TOKEN=TOKEN)
import e4_harness as H
import shared_link
import e7_link as K
import j1_links as JL
import j_replay as RP                      # P0-J replay (frozen)
import j1_bridge as JB
M = JB.M
check('P0-J modules come from the P0-J package', Path(JL.J.__file__).resolve().parent == P0J.resolve()
      and Path(RP.__file__).resolve().parent == P0J.resolve(), JL.J.__file__)
# ---------------- traces ----------------
st = json.loads((E3BRES / 'STATE.json').read_text()); TR = []
for tid, d in sorted(st['done'].items()):
    if d['level'] == 'C0' and d['arm'] == 'F-N2':
        TR.append(H.load_trace(E3BRES / 'trials' / Path(d['evidence_dir']).name, dict(d, trial_id=tid)))
check('three E3-B C0 F-N2 traces', len(TR) == 3, [t['trial_id'] for t in TR])
TICK = H.TICK
_src = (HERE / 'j1_run_all.py').read_text(); RA0 = types.ModuleType('j1ra0'); RA0.__file__ = str(HERE / 'j1_run_all.py')
exec(compile(_src.split('def main():')[0], 'j1_run_all', 'exec'), RA0.__dict__)


def replay_e4(tr, cfg, cap):
    c = K.CONFIGS[{'B0': 'K0', 'B1': 'K1', 'B2': 'K2', 'B3': 'K3'}[cfg]]
    if c['link'] == 'shared_link':
        return H.replay(tr, capacity_mbps=cap, duration_s=25, link='frozen')
    ce = c['expiry_us'].get('camera')
    return H.replay(tr, capacity_mbps=cap, duration_s=25, link='discipline', discipline=c['discipline'], cam_mode='factor',
                    cam_expiry=None if ce is None else ce / 1e6, scan_expiry=None)


def replay_j(tr, cfg, cap, state):
    L = JL.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY)
    t0 = (tr['goal_us'] // TICK) * TICK
    states = [(t0 - 10 * TICK, state[0], state[1], state[2])] if state else None
    R = RP.replay(tr, L, capacity_mbps=cap, duration_s=25, offers='live', states=states)
    return R, L


def offers_at(tr, t, t_start):
    s = any(t - TICK < x <= t for x in tr['scans']); f = any(t - TICK < x <= t for x, _ in tr['frames'])
    return s or f or (t - TICK - t_start) % 500_000 == 0


def drive(tr, cfg, cap, state=None, d_ranges=None):
    t0 = (tr['goal_us'] // TICK) * TICK; t_start = t0 - 300_000
    tr = dict(tr, start_us=t_start, scans=[s for s in tr['scans'] if s > t0 - TICK], frames=[f for f in tr['frames'] if f[0] > t0 - TICK])
    T = ((tr['trigger_us'] + TICK - 1) // TICK) * TICK
    while offers_at(tr, T, t_start):
        T += TICK
    tr['trigger_us'] = T; t1 = T + 30_000_000
    if cfg in ('B0', 'B1', 'B2', 'B3'):
        R = replay_e4(tr, cfg, cap); RL = None
    else:
        R, RL = replay_j(tr, cfg, cap, state)
    tmp = Path(tempfile.mkdtemp()); led = tmp / 'J1_LEDGER.jsonl'; dec = tmp / 'J1_DECISIONS.jsonl'
    plan = {'normal_mbps': 3.0, 'congest_mbps': cap if cap else 0.5, 'recovery_mbps': 3.0, 'congest_after_trigger_s': 0.0,
            'recover_after_trigger_s': 25.0}
    os.environ.update(SCI_D3_SCHEDULES_JSON=json.dumps([plan]), SCI_J1_CONFIG=cfg, SCI_J1_LEDGER=str(led), SCI_J1_DECISIONS=str(dec))
    sys.argv = ['j1_bridge.py', '--mode', 'Q2', '--capacity-mbps', '3.0', '--queue-kb', '768', '--report-sim-s', '5']
    LaserScan.RANGES = d_ranges if d_ranges is not None else [3.0]
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        node = M.JointC52Bridge(M.parse_args())
    node._make_output = lambda msg: (CompressedImage(header=msg.header, data=b'\0' * (msg.nbytes - 64)), 'compressed')
    subs = dict(node._subs)
    if state and '/odom' in subs:
        subs['/odom'](Obj(twist=Obj(twist=Obj(linear=Obj(x=-state[0])))))        # sign ignored (|v|)
        subs['/e2_nav_cmd_policy'](Obj(linear=Obj(x=state[1])))
    ep = '1' * 32; gu = '2' * 32; si = fi = 0; scans = tr['scans']; frames = tr['frames']; t = t_start
    with contextlib.redirect_stdout(out):
        while t <= t1:
            while si < len(scans) and scans[si] <= t:
                node.on_scan(LaserScan(scans[si])); si += 1
            while fi < len(frames) and frames[fi][0] <= t:
                node.on_camera(Image(frames[fi][0], frames[fi][1])); fi += 1
            node.on_clock(Obj(clock=Time(t)))
            if t == t_start + TICK:
                node.on_executed(Msg(data=5))
                node.on_episode_control(Msg(data=json.dumps({'scope_token': TOKEN, 'event': 'begin', 'episode_id': ep, 'goal_uuid': gu,
                                                             'reset_verified': True, 'goal_accepted': True})))
            if t == T and cap is not None:
                node.on_episode_control(Msg(data=json.dumps({'scope_token': TOKEN, 'event': 'trigger', 'episode_id': ep, 'goal_uuid': gu,
                                                             'pedestrian_trigger_ack': True})))
            t += TICK
    hdr, rows = __import__('e7_metrics').read_ledger(led)
    return tr, R, RL, node, hdr, rows, t0, dec, out.getvalue()


def compare(cfg, cap, tr0, state=None, d_ranges=None, label=''):
    tr, R, RL, node, hdr, rows, t0, dec, log = drive(tr0, cfg, cap, state, d_ranges)
    MAP = {'replaced': 'queue_drop', 'dropped_codel': 'queue_drop', 'dropped_slo': 'queue_drop'}
    rep = sorted((v[0], v[3], v[2], MAP.get(v[4], v[4]), v[5]) for v in R['status'].values() if v[4] != 'queued')
    liv = sorted((r['stream'], r['offer_us'], r['bytes'], MAP.get(r['status'], r['status']), r['finish_us']) for r in rows if r['offer_us'] >= t0)
    same_keys = [a[:4] for a in rep] == [b[:4] for b in liv]
    dmax = max([abs(a[4] - b[4]) for a, b in zip(rep, liv)] or [0]) / 1000 if same_keys else math.inf
    sp = sorted((tt, stp) for st_, tt, stp, _, _ in R['recv'] if st_ == 'scan'); cp = sorted((tt, stp) for st_, tt, stp, _, _ in R['recv'] if st_ == 'camera')
    bs = sorted((tt, m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000) for tt, m in node._pubs['/gate_b2/scan_edge'].out)
    bc = sorted((tt, m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000) for tt, m in node._pubs['/gate_c2/camera_edge_compressed'].out)
    edge_same = sp == bs and cp == bc
    good = hdr and hdr['config'] == RA0.LEDGER_NAME.get(cfg, cfg) and same_keys and dmax <= 1.0 and edge_same and len(rep) > 100
    n = lambda s: sum(1 for r in rows if r['status'] == s and r['offer_us'] >= t0)
    check(f"live J1 bridge = replay {cfg}{label} {'C0' if cap is None else cap} {tr0['trial_id']}", good,
          f"msgs {len(liv)}/{len(rep)}, max |dt| {dmax} ms, edge scans {len(bs)}/{len(sp)}, frames {len(bc)}/{len(cp)}, "
          f"replaced {n('replaced')}, codel {n('dropped_codel')}, slo {n('dropped_slo')}, expired {n('expired')}, link {type(node.link).__name__}")
    return node, dec, log, RL


for tr0 in TR[:2]:
    for cfg in ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7'):
        compare(cfg, 0.13, tr0)
for cfg in ('B0', 'B3', 'B4', 'B5', 'B7'):
    compare(cfg, None, TR[0])
# MAKL at two constant states: d from ranges (min valid - 0.22), v = max(odom, cmd)
for tr0 in TR:
    for label, v, vc, rmin in (('[tight]', 0.12, 0.15, 1.12), ('[loose]', 0.05, 0.08, 2.72)):
        ranges = [float('inf'), float('nan'), 0.05, rmin, 3.0, 9.0]       # inf, NaN, below range_min, above range_max ignored
        node, dec, log, RL = compare('MAKL', 0.13, tr0, state=(v, vc, rmin - 0.22), d_ranges=ranges, label=label)
        lines = [json.loads(l) for l in dec.read_text().splitlines() if l.strip()]
        D = [x for x in lines if x.get('kind') == 'decision']; C = [x for x in lines if 'cpu_ns' in x]
        ch = {x['choice'] for x in D}
        check(f'MAKL{label} inputs and decision log {tr0["trial_id"]}',
              lines[0].get('header') and len(D) > 50 and len(C) >= len(D)
              and {'scan', 'camera'} <= ch and all(abs(x['d'] - round(rmin - 0.22, 4)) < 1e-9 for x in D)
              and all(abs(x['v'] - max(v, vc)) < 1e-9 for x in D) and 'J1_STATE scans=' in log,
              f"decisions {len(D)}, cpu {len(C)}, choices {sorted(c for c in ch if c)}, cam share "
              f"{round(sum(x['choice'] == 'camera' for x in D) / max(1, len(D)), 3)}")
# ---------------- 3. MAKL inputs: subscriptions only for MAKL; d with no valid return ----------------
tr, R, RL, node, hdr, rows, t0, dec, log = drive(TR[0], 'B7', None)
check('no /odom or command subscription for a non-MAKL arm', not any(t in ('/odom', '/e2_nav_cmd_policy') for t, _ in node._subs))
tr, R, RL, node, hdr, rows, t0, dec, log = drive(TR[0], 'MAKL', None, state=(0.1, 0.1, 3.28), d_ranges=[float('inf')] * 5)
check('MAKL subscribes /odom (sensor QoS) and /e2_nav_cmd_policy; d = range_max - 0.22 if no valid return',
      {'/odom', '/e2_nav_cmd_policy'} <= {t for t, _ in node._subs} and abs(node.link.state['d'] - (3.5 - 0.22)) < 1e-9)
# ---------------- 4. capacity plans ----------------
import joint_bridge_logic as JBL
src = (HERE / 'j1_run_all.py').read_text(); RA = types.ModuleType('j1ra'); RA.__file__ = str(HERE / 'j1_run_all.py')
exec(compile(src.split('def main():')[0], 'j1_run_all', 'exec'), RA.__dict__)
good = True; det = {}
for c in RA.CELLS:
    p = RA.plan_for(c[0], c[2]); det[RA.cell_name(c)] = p
    if c[0] != 'C0':
        q = JBL.TriggeredPlan.parse(p); good &= q.recover_after_trigger_s == float(c[2]) and abs(q.congest_mbps - {'LC': 0.13, 'LB': 0.18}[c[0]]) < 1e-12
import e3b_policy_logic as L3
good &= det['C0-Q2'] == L3.plan('C0') and det['LC-Q2-25s'] == L3.plan('LC')
check('cell plans parse with the frozen rules; 25 s plans = the frozen E3-B/E5 plans', good, json.dumps(det)[:300])


def evs(t, bps, rec_after):
    e = [{'event': {'event': 'trigger', 'trigger_sim_us': t}}, {'event': {'event': 'network_phase', 'new_phase': 'congested', 'boundary_sim_us': t, 'new_bps': bps}}]
    if rec_after is not None:
        e.append({'event': {'event': 'network_phase', 'new_phase': 'recovery', 'boundary_sim_us': t + rec_after, 'new_bps': 3_000_000}})
    return {'network_events': e, 'goal_sim_s': 30.0, 'terminal_sim_s': 70.0}


check('config_ok maps B0-B3 to the frozen ledger names K0-K3', RA.config_ok({'bridge_config_line': 'J1_BRIDGE config=B3 link=QdiscLink', 'e5': {'ledger_config': 'K3'}}, 'B3')
      and not RA.config_ok({'bridge_config_line': 'J1_BRIDGE config=B3 link=QdiscLink', 'e5': {'ledger_config': 'B3'}}, 'B3')
      and RA.config_ok({'bridge_config_line': 'J1_BRIDGE config=MAKL link=J1Link', 'e5': {'ledger_config': 'MAKL'}}, 'MAKL'))
check('plan_ok accepts right / rejects wrong duration or capacity',
      RA.plan_ok(evs(42_000_000, 130_000, 8_000_000), 'LC', 8) and RA.plan_ok(evs(42_000_000, 180_000, 25_000_000), 'LB', 25)
      and not RA.plan_ok(evs(42_000_000, 130_000, 25_000_000), 'LC', 8) and not RA.plan_ok(evs(42_000_000, 180_000, 8_000_000), 'LC', 8)
      and not RA.plan_ok(evs(42_000_000, 130_000, None), 'LC', 8) and RA.plan_ok({'network_events': []}, 'C0', 25)
      and not RA.plan_ok(evs(42_000_000, 130_000, 8_000_000), 'C0', 25))
# ---------------- 5. metrics on a recorded E7 trial ----------------
import j1_metrics as JM
import e7_metrics as M7
s7 = json.loads((E7RES / 'STATE.json').read_text())
tid, r7 = next((k, v) for k, v in sorted(s7['done'].items()) if v['level'] == 'LC' and v.get('e7_config') == 'K3')
src_dir = E7RES / 'trials' / Path(r7['evidence_dir']).name
tmp = Path(tempfile.mkdtemp()) / src_dir.name; shutil.copytree(src_dir, tmp)
(tmp / 'J1_LEDGER.jsonl').write_text((tmp / 'E7_LEDGER.jsonl').read_text())
res = json.loads((tmp / 'TRIAL_RESULT.json').read_text()); res['crossing'] = res.get('crossing') or {}
a = M7.run_measures(tmp, res); b = JM.run_measures(tmp, res)
same = all(a[k] == b[k] for k in a if k.startswith('W_') or k.startswith('L_scan') or k in ('contact_class', 'arrival_error_s', 'ledger_config'))
(tmp / 'J1_DECISIONS.jsonl').write_text(json.dumps({'header': True, 'config': 'MAKL'}) + '\n' + '\n'.join(
    json.dumps(dict(t_us=int(res['trigger_sim_s'] * 1e6) + i * 100_000, choice=('camera' if i % 3 == 0 else 'scan'), astar=1.0, beta=0.9,
                    peak=0.5, c_est_bps=130000, v=0.15, d=0.8, q_scan=True, q_cam=(i % 2 == 0), kind='decision')) for i in range(30))
    + '\n' + '\n'.join(json.dumps({'cpu_ns': 10_000 + i}) for i in range(30)) + '\n')
with (tmp / 'c52_joint_bridge.log').open('a') as f:
    f.write('J1_STATE scans=500 odom=900 cmd=880 v_odom=0.150 v_cmd=0.150 d=0.800 decisions=30\n')
c = JM.run_measures(tmp, res)
check('j1_metrics: frozen E5/E7 measures unchanged; new fields filled', same and c['makl_decisions_W'] == 30 and c['makl_cam_chosen_W'] == 10
      and c['j1_inputs_odom'] == 900 and c['makl_cpu_med_us'] is not None and c['event_duration_measured_s'] == 25.0
      and c['a_nav_p95_clipped_s'] is not None and c['L_camera_payload_mean_B'] > 2000,
      f"viol {b.get('W_viol_anav_unom_b0.2_s')} (E7 run {r7.get('e5', {}).get('W_viol_anav_unom_b0.2_s')}), clipped {c['a_nav_p95_clipped_s']}, "
      f"event {c['event_duration_measured_s']} s, cam payload {c['L_camera_payload_mean_B']} B")
# ---------------- 6. analysis end to end on a synthetic STATE ----------------
import j1_analysis as JA
rnd = random.Random(7)
TRUE = {  # cell -> arm -> (viol, F, P1, clearance) : designed so that H1, H1b, H2a, H3 pass and H2b fails
    'C0-Q2': {a: (0.7, 1.56, 30.0, 0.40) for a in JA.ARMS}, 'C0-Q1': {a: (0.7, 4.6, 30.0, 0.40) for a in JA.ARMS}}
for c in JA.CONG:
    TRUE[c] = {'B0': (13.0, 0.2, 31, .4), 'B1': (13.0, .2, 31, .4), 'B2': (1.5, 0.0, 30.5, .4), 'B3': (3.0, 2.8, 30.5, .4),
               'B4': (4.5, 2.6, 30.5, .4), 'B5': (2.8, 3.6, 30.5, .4), 'B7': (1.8, 2.2, 30.5, .4), 'MAKL': (1.8, 2.9, 30.5, .4)}
    if c.startswith('LC'):
        TRUE[c]['MAKL'] = (1.8, 2.25, 30.5, .4)
done = {}
for row in RA.matrix():
    v, f_, p1, cl = TRUE[row['cell']][row['config']]
    e = {'W_viol_anav_unom_b0.2_s': round(v + rnd.uniform(-0.05, 0.05), 3), 'W_min_clearance_m': cl + rnd.uniform(-0.005, 0.005),
         'contact_class': None, 'a_nav_p95_clipped_s': 0.7, 'L_camera_payload_mean_B': 2700, 'ledger_config': row['config']}
    done[row['trial_id']] = {'evidence_dir': '/nonexistent', 'j1_config': row['config'], 'j1_cell': row['cell'], 'j1_rep': row['rep'],
                             'j1_delta_s': row['delta_s'], 'status': 'SUCCEEDED', 'order': len(done) + 1, 'e5': e,
                             '_F': f_ + rnd.uniform(-0.03, 0.03), '_P1': p1 + rnd.uniform(-0.2, 0.2)}
stubA = types.ModuleType('e3b_analysis')
stubA.per_run = lambda tid, r, td: {'F_fps': r['_F'], 'P1_s': r['_P1'], 'V': 0.1, 'S': 0.0, 'a_nav_p95_s': {'B0': 4.6, 'B1': 2.1, 'B2': 0.8, 'B3': 0.8}.get(r['j1_config'], 0.8) if r['j1_cell'] == 'LC-Q2-25s' else 0.7}
sys.modules['e3b_analysis'] = stubA
tdir = Path(tempfile.mkdtemp()); (tdir / 'STATE.json').write_text(json.dumps({'done': done}))
sys.argv = ['j1_analysis.py', str(tdir / 'STATE.json'), str(tdir / 'out')]
with contextlib.redirect_stdout(io.StringIO()):
    JA.main()
V = json.loads((tdir / 'out' / 'J1_VERDICTS.json').read_text())
check('analysis verdicts on designed synthetic data', V['H1_noninferior_violation_vs_B7_all_congested_cells'] and V['H1b_violation_below_B3_25s_cells']
      and V['H2a_camera_above_B7_Q1_cells'] and not V['H2b_camera_above_B7_Q2_cells'] and V['H3_violation_below_B4_and_B5_25s_cells']
      and V['H4a_no_robot_moving_contact_MAKL'] and V['H4b_P1_and_clearance_not_different_from_C0'] and V['R_anchor_B0_B3_reproduce_E6']
      and V['outcome'] == 'MAKL_SUPPORTED' and (tdir / 'out' / 'J1_CELLS.csv').is_file() and (tdir / 'out' / 'J1_PAIRED.csv').is_file(),
      json.dumps({k: V[k] for k in V if not k.endswith('_cells') and k not in ('R_detail', 'P_check_B7_bound', 'status_counts')}))
# NI must FAIL when MAKL is clearly worse than B7 by more than the margin in one cell
for tid, r in done.items():
    if r['j1_cell'] == 'LB-Q1-8s' and r['j1_config'] == 'MAKL':
        r['e5'] = dict(r['e5'], **{'W_viol_anav_unom_b0.2_s': r['e5']['W_viol_anav_unom_b0.2_s'] + 0.6})
(tdir / 'STATE.json').write_text(json.dumps({'done': done}))
with contextlib.redirect_stdout(io.StringIO()):
    JA.main()
V2 = json.loads((tdir / 'out' / 'J1_VERDICTS.json').read_text())
check('non-inferiority fails when MAKL is worse by more than the margin', not V2['H1_noninferior_violation_vs_B7_all_congested_cells']
      and V2['H1_cells']['LB-Q1-8s'] is False and V2['outcome'] == 'MAKL_PARTIALLY_SUPPORTED')
check('paired rule needs >= 4 pairs and all differences of one sign', not JA.clearly_greater({1: 2, 2: 2, 3: 2}, {1: 0, 2: 0, 3: 0}, 0.1)['clearly_greater']
      and not JA.clearly_greater({1: 2, 2: 2, 3: 2, 4: -1}, {1: 0, 2: 0, 3: 0, 4: 0}, 0.1)['clearly_greater']
      and JA.clearly_greater({1: 2, 2: 2, 3: 2, 4: 1}, {1: 0, 2: 0, 3: 0, 4: 0}, 0.1)['clearly_greater'])
# ---------------- 7. matrix ----------------
rows = RA.matrix(); blocks = [rows[i:i + 8] for i in range(0, 240, 8)]
check('matrix: 240 runs, 30 blocks of 8 arms, 8 rotations, delta per rep',
      len(rows) == 240 and len({r['trial_id'] for r in rows}) == 240 and len(blocks) == 30
      and all(len({(r['cell'], r['rep']) for r in b}) == 1 and sorted(r['config'] for r in b) == sorted(RA.ARMS) for b in blocks)
      and len({tuple(r['config'] for r in b) for b in blocks}) == 8 and all(r['delta_s'] == RA.DELTA[r['rep']] for r in rows)
      and {r['cell'] for r in rows} == set(JA.CELLS))
check('gates: MAKL at delta -0.4 (LC-Q2-25s) and +0.4 (LB-Q1-8s)', [(g['cell'], g['config'], g['delta_s']) for g in RA.GATES]
      == [('LC-Q2-25s', 'MAKL', -0.4), ('LB-Q1-8s', 'MAKL', 0.4)])
# ---------------- 8. compile ----------------
bad = []
for f in sorted(HERE.rglob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(Path(tempfile.mkdtemp()) / 'x.pyc'))
    except Exception as ex:
        bad.append((f.name, repr(ex)))
check('every package script compiles', not bad, bad)
print('J1_OFFLINE_TESTS', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
