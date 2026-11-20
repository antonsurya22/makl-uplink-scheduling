#!/usr/bin/env python3
"""J2 offline tests (no ROS, no Gazebo). Usage: test_j2_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. provenance: E5, E7, P0-J and J1 v2 packages match their freezes;
 1. j2_run_trial.py = J1 v2 j1_run_trial.py + declared edits (byte check);
 2. impairment model on synthetic periodic traffic, all 6 arms:
    'none' = the J1 link record by record; Bernoulli loss rate; Gilbert-Elliott loss rate (stationary pi_B * h);
    jitter keeps order, stays within +-J and never arrives before the end of transmission; square-wave capacity:
    serializer alternates hi/lo every 1 s from the congestion boundary with mean = nominal, while the bridge-facing
    capacity stays nominal; ledger conservation with channel_loss; same seed -> identical run;
 3. LIVE J2 BRIDGE (ROS stubbed; real bridge class chain + J1 launcher + J2 wrapper) with an E3-B C0 trace at 0.13 Mbit/s:
    equals the replay with the same impaired link (same seed) record by record for B3, B7 and MAKL under none, loss5,
    ge, jit50 and capsq; impairment line and J2_STATE lines printed; channel_loss reaches the ledger;
 4. run-all helpers: plan_ok, impair_ok, gate checks; matrix 252 runs, 42 blocks, one seed per block;
 5. j2_analysis end to end on a synthetic STATE: designed verdicts (ROBUST, then a dominated case -> NOT_ROBUST);
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
import j2_run_trial_edits as ED
check('j2_run_trial.py = J1 v2 j1_run_trial.py + declared edits', ED.derive((J1 / 'j1_run_trial.py').read_text()) == (HERE / 'j2_run_trial.py').read_text())

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
        self.header = Header(us); self.ranges = [float('inf'), 1.12, 3.0]; self.range_min = 0.12; self.range_max = 3.5


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
TOKEN = 'j2' + 'a' * 46
os.environ.update(SCI_SIM_ONLY='YES', SCI_D3_OWNED_EXCLUSIVE='YES', SCI_D3_SCOPE_TOKEN=TOKEN)
import e4_harness as H
import j1_links as JL
import j2_impair as I
import j_synth as SY
import j_replay as RP

# ---------------- 2. impairment model ----------------
mk = lambda cfg: JL.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000)
ARMS = ('B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
stf = (lambda t: (0.12, 0.15, 0.6 + abs(((t / 1e6) % 8) - 4) / 4))
same = True
for cfg in ARMS:
    for prof, c in (('Q2', 130_000), ('Q1', 180_000), ('Q1', 3_000_000)):
        a = SY.run(mk(cfg), profile=prof, cap_bps=c, probes=True, state_fn=stf if cfg == 'MAKL' else None)
        b = SY.run(I.impair(mk(cfg), I.CONDITIONS['none'], 7), profile=prof, cap_bps=c, probes=True, state_fn=stf if cfg == 'MAKL' else None)
        same &= [x[1:] for x in a] == [x[1:] for x in b]
check("'none' impairment = the J1 link record by record (6 arms x 3 cells)", same)


def big(cond, cfg='B7', c=3_000_000, horizon=200_000_000, seed=11):
    L = I.impair(mk(cfg), I.CONDITIONS[cond], seed)
    r = SY.run(L, profile='Q1', cap_bps=c, probes=False, horizon_us=horizon)
    return L, r


L, r = big('loss5'); s = L.j2_summary()
se = math.sqrt(0.05 * 0.95 / s['packets'])
check('Bernoulli 5 %: packet loss rate within 4 SE', abs(s['loss_rate_packets'] - 0.05) < 4 * se, f"{s['loss_rate_packets']} over {s['packets']} packets")
L, r = big('loss1'); s = L.j2_summary(); se = math.sqrt(0.01 * 0.99 / s['packets'])
check('Bernoulli 1 %: packet loss rate within 4 SE', abs(s['loss_rate_packets'] - 0.01) < 4 * se, f"{s['loss_rate_packets']}")
L, r = big('ge', horizon=600_000_000); s = L.j2_summary(); pb = 0.25 / 4.0
E = L.j2_ge_edges; bad_t = sum(E[i + 1] - E[i] for i in range(1, len(E) - 1, 2))
check('Gilbert-Elliott timeline: bad share of time near 6.25 %, mean bad period near 0.25 s',
      abs(bad_t / E[-1] - pb) < 0.02 and abs(bad_t / max(1, (len(E) - 1) // 2) / 1e6 - 0.25) < 0.06,
      f'{len(E)} edges, bad share {round(bad_t / E[-1], 4)}')
check('Gilbert-Elliott: loss rate near 6.25 % * 0.5 = 3.1 %, bad-state packet share near 6.25 %',
      abs(s['loss_rate_packets'] - pb * 0.5) < 0.008 and abs(s['bad_state_packets'] / s['packets'] - pb) < 0.015,
      f"loss {s['loss_rate_packets']}, bad share {round(s['bad_state_packets'] / s['packets'], 4)}, packets {s['packets']}")
lost_msgs = Counter(x[3] for x in r)['channel_loss']
lines = [x for x in L.ledger.lines if not x.get('header')]
check('channel_loss reaches the ledger; conservation (terminal + queued = offered)',
      lost_msgs > 0 and sum(1 for x in lines if x['status'] == 'channel_loss') == lost_msgs
      and len(lines) + sum(1 for x in r if x[3] == 'queued') == len(r), f'lost messages {lost_msgs}')
def exp_abs(J, d):
    """E|max(U(-J,J), -d)|: the offset is clamped so that a packet never arrives before its transmission ends."""
    return J / 2 if J <= d else (d * d / 2 + J * J / 2 + (J - d) * d) / (2 * J)


# jitter: order, bounds, mean (from the ledger: finish = arrival of the last packet; serialization at 3 Mbit/s)
for cond, J in (('jit10', 10_000), ('jit50', 50_000)):
    L = I.impair(mk('B7'), I.CONDITIONS[cond], 3)
    SY.run(L, profile='Q1', cap_bps=3_000_000, probes=False, horizon_us=60_000_000)
    s = L.j2_summary()
    led = sorted((x['tx_us'][-1], x['finish_us'], x['bytes']) for x in L.ledger.lines if not x.get('header') and x['status'] == 'delivered')
    inorder = all(led[i][1] <= led[i + 1][1] for i in range(len(led) - 1))
    lag = []
    for t_last, fin, b in led:
        w = min(1472, b - 1472 * ((max(1, b) - 1) // 1472)) + 28
        lag.append(fin - (t_last + math.ceil(8 * w * 1e6 / 3_000_000)))
    bounded = min(lag) >= 0 and max(lag) <= 20_000 + J
    check(f'{cond}: deliveries in order, delay within [0, 20 ms + J], mean |offset| = E|max(U(-J,J), -20 ms)|', inorder and bounded
          and abs(s['mean_abs_jitter_ms'] - exp_abs(J, 20_000) / 1000) < exp_abs(J, 20_000) / 1000 * 0.15,
          f"mean |jitter| {s['mean_abs_jitter_ms']} ms, delay range {min(lag) / 1000}-{max(lag) / 1000} ms, clamped {s['reordered_clamped']}")
# capacity square wave
L = I.impair(mk('B7'), I.CONDITIONS['capsq'], 5)
L.advance(1_000_000); L.capacity_bps = 130_000
eff = []
for t in range(1_000_000, 9_000_000, 100_000):
    L.advance(t); eff.append(L.j2_effective_capacity())
hi = int(round(130_000 * I.CONDITIONS['capsq']['capacity']['hi'])); lo = int(round(130_000 * I.CONDITIONS['capsq']['capacity']['lo']))
pattern = all(e == (hi if ((t - 1_000_000) // 1_000_000) % 2 == 0 else lo) for t, e in zip(range(1_000_000, 9_000_000, 100_000), eff))
check('capsq: serializer alternates 0.20 / 0.06 Mbit/s every 1 s from the boundary; mean = nominal; bridge sees nominal',
      pattern and abs(sum(eff) / len(eff) - 130_000) < 2 and L.capacity_bps == 130_000 and hi == 200_000 and lo == 60_000,
      f'hi {hi}, lo {lo}, mean {round(sum(eff) / len(eff))}')
L.capacity_bps = 3_000_000
check('capsq: no trace at 3 Mbit/s', L.j2_effective_capacity() == 3_000_000)
a = big('ge', cfg='MAKL', c=130_000, horizon=40_000_000, seed=99)[1]; b = big('ge', cfg='MAKL', c=130_000, horizon=40_000_000, seed=99)[1]
c_ = big('ge', cfg='MAKL', c=130_000, horizon=40_000_000, seed=98)[1]
check('same seed -> identical run; other seed -> different', a == b and a != c_)
# common random numbers: extra traffic in one arm does not change the draws of the other packets
lost = []
for pr in (False, True):
    Lx = I.impair(mk('B7'), I.CONDITIONS['loss5'], 21)
    SY.run(Lx, profile='Q1', cap_bps=3_000_000, probes=pr, horizon_us=60_000_000)
    lost.append({(x.stream, x.created) for x in Lx.records.values() if x.status == 'channel_loss' and x.stream != 'c52_probe_ack'})
check("common random numbers: the scan/camera messages lost are the same with or without extra probe traffic",
      lost[0] == lost[1] and len(lost[0]) > 5, f'{len(lost[0])} lost')

# ---------------- 3. live J2 bridge vs replay ----------------
import j2_bridge as J2B
M = J2B.JB.M
st = json.loads((E3BRES / 'STATE.json').read_text())
tid, d = next((k, v) for k, v in sorted(st['done'].items()) if v['level'] == 'C0' and v['arm'] == 'F-N2')
TR = H.load_trace(E3BRES / 'trials' / Path(d['evidence_dir']).name, dict(d, trial_id=tid))
TICK = H.TICK


def offers_at(tr, t, t_start):
    s = any(t - TICK < x <= t for x in tr['scans']); f = any(t - TICK < x <= t for x, _ in tr['frames'])
    return s or f or (t - TICK - t_start) % 500_000 == 0


def drive(cfg, cond, seed, cap=0.13):
    tr = TR
    t0 = (tr['goal_us'] // TICK) * TICK; t_start = t0 - 300_000
    tr = dict(tr, start_us=t_start, scans=[s for s in tr['scans'] if s > t0 - TICK], frames=[f for f in tr['frames'] if f[0] > t0 - TICK])
    T = ((tr['trigger_us'] + TICK - 1) // TICK) * TICK
    while offers_at(tr, T, t_start):
        T += TICK
    tr['trigger_us'] = T; t1 = T + 30_000_000
    RL = I.impair(J2B._ORIG(cfg, capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY), I.CONDITIONS[cond], seed)
    states = [(t0 - 10 * TICK, 0.12, 0.15, 1.12 - 0.22)]
    R = RP.replay(tr, RL, capacity_mbps=cap, duration_s=25, offers='live', states=states)
    tmp = Path(tempfile.mkdtemp())
    plan = {'normal_mbps': 3.0, 'congest_mbps': cap, 'recovery_mbps': 3.0, 'congest_after_trigger_s': 0.0, 'recover_after_trigger_s': 25.0}
    os.environ.update(SCI_D3_SCHEDULES_JSON=json.dumps([plan]), SCI_J1_CONFIG=cfg, SCI_J1_LEDGER=str(tmp / 'L.jsonl'),
                      SCI_J1_DECISIONS=str(tmp / 'D.jsonl'), SCI_J2_CONDITION=cond, SCI_J2_IMPAIR=json.dumps(I.CONDITIONS[cond]),
                      SCI_J2_SEED=str(seed))
    sys.argv = ['j2_bridge.py', '--mode', 'Q2', '--capacity-mbps', '3.0', '--queue-kb', '768', '--report-sim-s', '5']
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
    if not same and os.environ.get('J2_TEST_DEBUG'):
        print('DEBUG only-replay', [x for x in rep if x not in liv][:6]); print('DEBUG only-live', [x for x in liv if x not in rep][:6])
    log = out.getvalue()
    nloss = sum(1 for r in rows if r['status'] == 'channel_loss')
    good = same and len(rep) > 100 and f'J2_IMPAIR condition={cond} seed={seed}' in log and 'J2_STATE scans=' in log \
        and node.link.capacity_bps in (3_000_000, int(cap * 1e6)) and (nloss > 0 or cond in ('none', 'jit50', 'capsq'))
    check(f'live J2 bridge = impaired replay: {cfg} {cond}', good,
          f"msgs {len(liv)}/{len(rep)}, channel_loss {nloss}, summary {json.dumps(node.link.j2_summary())[:160]}")


for cfg in ('B3', 'B7', 'MAKL'):
    for cond in ('none', 'loss5', 'ge', 'jit50', 'capsq'):
        drive(cfg, cond, I.seed_for(cond, 1, 'Q2'))

# ---------------- 4. run-all helpers ----------------
src = (HERE / 'j2_run_all.py').read_text(); RA = types.ModuleType('j2ra'); RA.__file__ = str(HERE / 'j2_run_all.py')
exec(compile(src.split('def main():')[0], 'j2_run_all', 'exec'), RA.__dict__)
rows = RA.matrix(); blocks = [rows[i:i + 6] for i in range(0, 252, 6)]
check('matrix: 252 runs, 42 blocks of 6 arms, 6 rotations, one seed per block, deltas -0.4/0/+0.4',
      len(rows) == 252 and len(blocks) == 42 and all(len({(r['cell'], r['rep'], r['seed']) for r in b}) == 1 and sorted(r['config'] for r in b) == sorted(RA.ARMS) for b in blocks)
      and len({tuple(r['config'] for r in b) for b in blocks}) == 6 and {r['delta_s'] for r in rows} == {-0.4, 0.0, 0.4}
      and len({r['seed'] for r in rows}) == 42)


def evs(t, bps, rec):
    e = [{'event': {'event': 'trigger', 'trigger_sim_us': t}}, {'event': {'event': 'network_phase', 'new_phase': 'congested', 'boundary_sim_us': t, 'new_bps': bps}}]
    if rec:
        e.append({'event': {'event': 'network_phase', 'new_phase': 'recovery', 'boundary_sim_us': t + rec, 'new_bps': 3_000_000}})
    return {'network_events': e, 'goal_sim_s': 30.0, 'terminal_sim_s': 66.0}


g = RA.gates()[0]
good_r = dict(evs(42_000_000, 130_000, None), status='SUCCEEDED', a_nav_logged=True, bridge_config_line='J1_BRIDGE config=MAKL link=x',
              e5={'ledger_config': 'MAKL', 'arrival_error_s': 0.01, 'j1_inputs_odom': 5, 'j1_inputs_cmd': 5, 'makl_decisions_all': 50,
                  'makl_decisions_v_pos_all': 40, 'makl_decisions_W': 30, 'makl_scan_chosen_W': 20, 'makl_cam_chosen_W': 10,
                  'event_congested_bps': 130000, 'j2_condition_bridge': 'ge', 'j2_seed_bridge': g['seed'], 'j2_lost_packets': 9, 'L_all_channel_loss': 4})
bad_r = json.loads(json.dumps(good_r)); bad_r['e5']['j2_seed_bridge'] = 1
bad_r2 = json.loads(json.dumps(good_r)); bad_r2['e5']['j2_lost_packets'] = 0
check('gate checks: pass when all present; fail on wrong seed or inactive loss',
      all(RA.gate_checks(good_r, g).values()) and not all(RA.gate_checks(bad_r, g).values()) and not all(RA.gate_checks(bad_r2, g).values())
      and RA.config_ok({'bridge_config_line': 'J1_BRIDGE config=B2 link=ImpQdiscLink', 'e5': {'ledger_config': 'K2'}}, 'B2'))

# ---------------- 5. analysis on a synthetic STATE ----------------
import j2_analysis as JA
REF = {'LC-Q2': {'B2': (1.3, .02), 'B3': (2.4, 1.56), 'B4': (3.8, 1.57), 'B5': (2.5, 1.58), 'B7': (1.6, .41), 'MAKL': (1.9, .88)},
       'LB-Q1': {'B2': (1.2, .22), 'B3': (2.2, 2.97), 'B4': (2.4, 2.71), 'B5': (1.8, 3.57), 'B7': (1.3, 2.28), 'MAKL': (1.4, 3.22)}}
stubA = types.ModuleType('e3b_analysis'); stubA.per_run = lambda tid, r, td: {'F_fps': r['_F'], 'P1_s': 19.3, 'V': 0.1, 'S': 0.0, 'a_nav_p95_s': 0.7}
sys.modules['e3b_analysis'] = stubA
rnd = random.Random(3)


def synth(dominate=False):
    done = {}
    for row in rows:
        pc = row['cell'].split('|')[0]; v, f_ = REF[pc][row['config']]
        if dominate and row['condition'] == 'ge' and pc == 'LB-Q1' and row['config'] == 'B5':
            v, f_ = 0.3, 4.5
        done[row['trial_id']] = {'evidence_dir': '/nonexistent', 'j1_config': row['config'], 'j1_cell': row['cell'], 'j1_rep': row['rep'],
                                 'j1_delta_s': row['delta_s'], 'j2_condition': row['condition'], 'status': 'SUCCEEDED', 'order': len(done) + 1,
                                 '_F': f_ + rnd.uniform(-.02, .02),
                                 'e5': {'W_viol_anav_unom_b0.2_s': v + rnd.uniform(-.05, .05), 'W_min_clearance_m': 0.52, 'contact_class': None}}
    return done


for dom, want in ((False, 'ROBUST'), (True, 'NOT_ROBUST')):
    tdir = Path(tempfile.mkdtemp()); (tdir / 'STATE.json').write_text(json.dumps({'done': synth(dom)}))
    sys.argv = ['j2_analysis.py', str(tdir / 'STATE.json'), str(tdir / 'out')]
    with contextlib.redirect_stdout(io.StringIO()):
        JA.main()
    V = json.loads((tdir / 'out' / 'J2_VERDICTS.json').read_text())
    check(f'analysis on designed synthetic data -> {want}', V['outcome'] == want and V['R_none_reproduces_J1'] and V['n_runs'] == 252
          and (not dom or V['H3_by_cell']['LB-Q1|ge'] is False), json.dumps({k: V[k] for k in ('outcome', 'H2_cells_passed', 'R_none_reproduces_J1')}))
# ---------------- 6. compile ----------------
bad = []
for f in sorted(HERE.rglob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(Path(tempfile.mkdtemp()) / 'x.pyc'))
    except Exception as ex:
        bad.append((f.name, repr(ex)))
check('every package script compiles', not bad, bad)
print('J2_OFFLINE_TESTS', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
