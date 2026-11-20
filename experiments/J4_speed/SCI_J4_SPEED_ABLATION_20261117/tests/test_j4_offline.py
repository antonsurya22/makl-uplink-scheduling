#!/usr/bin/env python3
"""J4 offline tests (no ROS, no Gazebo). Usage: test_j4_offline.py [ROS2_WS]   (default ~/ros2_ws)
 0. provenance: E5, E7, P0-J, J1 v2 and J3 packages match their freezes;
 1. j4_run_trial.py = J3 j3_run_trial.py + declared edits (byte check);
 2. links: MAKL_cap is the frozen P0-J configuration (v_mode 'cap', v_cap 0.15); on synthetic periodic traffic MAKL_cap
    equals MAKL record by record when v_hat = 0.15 and differs when v_hat = 0.22 or 0.10 (decision log: the speed used
    is 0.15 for MAKL_cap and v_hat for MAKL; v_hat is logged for both); B7 and DSW_1.0 unchanged by the J4 factory;
 3. speed level: j4_stack on a generated movement config (DWB cap V, evidence file); mode gate and guard configure;
 4. J4 bridge chain imports with ROS stubbed; the J1 config list holds the four J4 arms; the bridge's factory is J4's;
 5. metrics on archived J1 runs (if present on this computer): phase A + B violation = frozen W violation; F_run;
    d_sw of MAKL; recomputation of choices; synthetic d_sw cases;
 6. run-all helpers: matrix 75 runs, 15 blocks, rotation, one C0 run per block, x_meet = 2.40 + V * delta; pilot
    checks, fallback rule, stack check;
 7. j4_analysis end to end on synthetic STATEs: designed outcomes;
 8. every script compiles."""
import contextlib, hashlib, io, json, math, os, py_compile, random, sys, tempfile, types
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
WS = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.home() / 'ros2_ws'
os.environ['HOME'] = str(WS.parent)
E5 = WS / 'SCI_E5_PED_SPEED_20261101'; E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'; P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'; J3 = WS / 'SCI_J3_SWITCH_ABLATION_20261116'; E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
J1RES = WS / 'results/SCI_J1_SCHEDULERS_V2_20261113/J1_MAIN/trials'
ok = True


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def check(name, good, detail=''):
    global ok
    ok &= bool(good)
    print(('PASS ' if good else 'FAIL ') + name, detail, flush=True)


# ---------------- 0. provenance ----------------
for P, name, pref in ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
                      (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2'), (J1, 'J1_FREEZE_HASHES.json', 'befd9065'),
                      (J3, 'J3_FREEZE_HASHES.json', 'b0ba0679')):
    z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if sha(P / f) != h]
    check(P.name + ' unchanged', not bad and sha(P / name).startswith(pref), bad)

# ---------------- 1. run_trial byte check ----------------
sys.path.insert(0, str(HERE / 'tests'))
import j4_run_trial_edits as ED
check('j4_run_trial.py = J3 j3_run_trial.py + declared edits', ED.derive((J3 / 'j3_run_trial.py').read_text()) == (HERE / 'j4_run_trial.py').read_text())

# ---------------- ROS stubs ----------------
class Any:
    def __init__(self, *a, **k): pass
    def __call__(self, *a, **k): return Any()
    def __getattr__(self, n): return Any()


def stub(name, **kw):
    m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m


class Obj:
    def __init__(self, **k): self.__dict__.update(k)


class Node:
    def __init__(self, name, **k): pass


POL = Obj(KEEP_LAST=1, BEST_EFFORT=1, RELIABLE=2, VOLATILE=1)
stub('rclpy', init=lambda *a, **k: None, ok=lambda: True, spin_once=lambda *a, **k: None, shutdown=lambda: None, spin=lambda *a: None)
stub('rclpy.node', Node=Node)
stub('rclpy.qos', QoSProfile=Any, ReliabilityPolicy=POL, DurabilityPolicy=POL, HistoryPolicy=POL, qos_profile_sensor_data='SENSOR')
stub('rclpy.serialization', serialize_message=lambda m: b'')
stub('sensor_msgs'); stub('sensor_msgs.msg', LaserScan=Any, Image=Any, CompressedImage=Any)
stub('std_msgs'); stub('std_msgs.msg', String=Any, Float32=Any, UInt8=Any)
stub('rosgraph_msgs'); stub('rosgraph_msgs.msg', Clock=Any)
stub('naw_interfaces'); stub('naw_interfaces.msg', NetworkState=Any)
stub('nav_msgs'); stub('nav_msgs.msg', Odometry='Odometry')
stub('geometry_msgs'); stub('geometry_msgs.msg', Twist=Any)
stub('cv2')
for p in (WS / 'GATE_B1_Link_Core', E7 / 'e4_frozen', E7, P0J / 'tests', P0J, J1, J3, E3B, HERE):
    sys.path.insert(0, str(p))
os.environ.update(SCI_SIM_ONLY='YES', SCI_D3_OWNED_EXCLUSIVE='YES', SCI_D3_SCOPE_TOKEN='j4' + 'a' * 46)
import j4_links as L4
L4.install()
JL = L4.JL
import j_synth as SY

# ---------------- 2. links ----------------
check('MAKL_cap = frozen P0-J configuration (v_mode cap, v_cap 0.15)',
      JL.J.CONFIGS['MAKL_cap'] == {'jdisc': 'makl', 'params': {'v_mode': 'cap'}} and JL.J.MAKL_DEFAULTS['v_cap'] == 0.15)
check('J1 config list holds the four J4 arms', all(a in JL.J1_CONFIGS for a in L4.J4_CONFIGS))
TMP = Path(tempfile.mkdtemp())


def mk(cfg, tag):
    return JL.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000,
                        ledger_path=str(TMP / f'L_{tag}.jsonl'), decisions_path=str(TMP / f'D_{tag}.jsonl'))


def ramp(v):
    """d falls from 2.0 m to 0.4 m over 30 s (synthetic approach)."""
    return lambda t: (v, v, max(0.4, 2.0 - 1.6 * t / 30_000_000))


def dlog(tag):
    return [json.loads(l) for l in (TMP / f'D_{tag}.jsonl').read_text().splitlines() if '"kind":"decision"' in l.replace(' ', '')]


same15 = True; diff22 = False; diff10 = False
for prof, c in (('Q1', 180_000), ('Q1', 130_000), ('Q2', 130_000)):
    a = SY.run(mk('MAKL', f'm15{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.15))
    b = SY.run(mk('MAKL_cap', f'f15{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.15))
    same15 &= [x[1:] for x in a] == [x[1:] for x in b]
    a = SY.run(mk('MAKL', f'm22{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.22))
    b = SY.run(mk('MAKL_cap', f'f22{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.22))
    diff22 |= [x[1:] for x in a] != [x[1:] for x in b]
    a = SY.run(mk('MAKL', f'm10{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.10))
    b = SY.run(mk('MAKL_cap', f'f10{prof}{c}'), profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.10))
    diff10 |= [x[1:] for x in a] != [x[1:] for x in b]
check('MAKL_cap = MAKL record by record when v_hat = 0.15 (3 cells)', same15)
check('MAKL_cap differs from MAKL when v_hat = 0.22 and when v_hat = 0.10', diff22 and diff10)
Dm = dlog('m22Q1180000'); Df = dlog('f22Q1180000')
check('decision log: MAKL uses v_hat, MAKL_cap uses 0.15; v_hat logged for both',
      Dm and Df and all(abs(x['v'] - x['v_hat']) < 1e-9 and abs(x['v_hat'] - 0.22) < 1e-9 for x in Dm)
      and all(x['v'] == 0.15 and abs(x['v_hat'] - 0.22) < 1e-9 for x in Df), f'{len(Dm)} / {len(Df)} decisions')
# MAKL_cap switches at the same d at every speed; MAKL switches farther out when faster (synthetic, Q1 0.18 Mbit/s)
import j4_metrics as M4


SGRID = [{'sim_s': round(0.1 * k, 1), 'clearance_m': max(0.4, 2.0 - 1.6 * 0.1 * k / 30) + 0.10} for k in range(300)]
fs = {}
for v in (0.10, 0.22):
    for cfg in ('MAKL', 'MAKL_cap'):
        tag = f'sw{cfg}{v}'
        SY.run(mk(cfg, tag), profile='Q1', cap_bps=180_000, probes=True, state_fn=ramp(v))
        fs[(cfg, v)] = M4.d_switch(dlog(tag), SGRID, 0.0, 30.0)['d_sw_m']
check('synthetic approach (d_sw rule): MAKL switching clearance rises with speed by > 0.3 m; MAKL_cap within 0.10 m',
      None not in fs.values() and fs[('MAKL', 0.22)] - fs[('MAKL', 0.10)] > 0.3
      and abs(fs[('MAKL_cap', 0.22)] - fs[('MAKL_cap', 0.10)]) <= 0.10, fs)
D = dlog('m22Q1180000')
C = [x for x in D if x['q_scan'] and x['q_cam'] and isinstance(x.get('peak'), (int, float)) and math.isfinite(x['peak'])]
check('recomputed choice with the logged speed = logged choice (MAKL, contested decisions)',
      C and sum(M4.recompute(x, x['v']) == x['choice'] for x in C) / len(C) >= 0.99, len(C))
same = True
for prof, c in (('Q1', 180_000), ('Q2', 130_000)):
    for cfg in ('B7', 'DSW_1.0'):
        a = SY.run(JL.J.make_jlink(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000) if cfg == 'B7'
                   else L4.L3.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000),
                   profile=prof, cap_bps=c, probes=True, state_fn=ramp(0.15))
        b = SY.run(JL.make_link(cfg, capacity_bps=3_000_000, queue_limit_bytes=768_000, delay_us=20_000), profile=prof,
                   cap_bps=c, probes=True, state_fn=ramp(0.15))
        same &= [x[1:] for x in a] == [x[1:] for x in b]
check('B7 and DSW_1.0 through the J4 factory = the J1 / J3 classes record by record', same)

# ---------------- 3. speed level ----------------
import j4_stack as S4
import generate_movement_config as GM
ok3 = True
for v in (0.10, 0.15, 0.22):
    out = Path(tempfile.mkdtemp()); mv = GM.generate(out, edge=True); m = S4.apply_speed(mv, out, v)
    import yaml
    d = yaml.safe_load(Path(m['controller']).read_text())['controller_server']['ros__parameters']['FollowPath']
    ev = json.loads((out / 'J4_SPEED.json').read_text())
    ok3 &= d['max_vel_x'] == v and d['max_speed_xy'] == v and ev['dwb_max_vel_x'] == v and ev['guard_max_lin'] == max(0.18, v)
    base = yaml.safe_load(Path(mv['controller']).read_text())
    nd = yaml.safe_load(Path(m['controller']).read_text())
    base['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = v
    base['controller_server']['ros__parameters']['FollowPath']['max_speed_xy'] = v
    ok3 &= base == nd
check('apply_speed: DWB max_vel_x = max_speed_xy = V, nothing else changed (0.10, 0.15, 0.22)', ok3)
try:
    S4.apply_speed(mv, out, 0.30); bad = False
except RuntimeError:
    bad = True
check('apply_speed refuses a level outside the set', bad)
import j4_mode_gate as MG, j4_guard as GG
G = types.SimpleNamespace(LIN_CAP=(0.15, 0.08, 0.0)); MG.configure(G, 0.22)
H = types.SimpleNamespace(MAX_LIN=0.18); GG.configure(H, 0.22); H2 = types.SimpleNamespace(MAX_LIN=0.18); GG.configure(H2, 0.10)
check('mode gate NORMAL cap = V (SLOW, STOP unchanged); guard MAX_LIN = max(0.18, V)',
      G.LIN_CAP == (0.22, 0.08, 0.0) and H.MAX_LIN == 0.22 and H2.MAX_LIN == 0.18)
src = (E3B / 'e2_mode_gate.py').read_text(); src2 = (E3B / 'safety_chain.py').read_text()
check('frozen gate and guard hold the names that J4 sets', 'LIN_CAP=(0.15,0.08,0.0)' in src and 'MAX_LIN=0.18' in src2)

# ---------------- 4. bridge chain ----------------
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    import j4_bridge as J4B
check('J4 bridge imports the J1 launcher with the J4 factory', J4B.JB.JL.make_link is L4.make_link and
      all(a in J4B.JB.JL.J1_CONFIGS for a in L4.J4_CONFIGS) and J4B.JB.JL.is_makl(mk('MAKL_cap', 'br')))

# ---------------- 5. metrics ----------------
runs = sorted(J1RES.glob('J1_R*_LB-Q1-25s_MAKL_*'))[:3] + sorted(J1RES.glob('J1_R*_LB-Q1-25s_B7_*'))[:2] if J1RES.is_dir() else []
if runs:
    good = True; dsw = []
    for d in runs:
        res = json.loads((d / 'TRIAL_RESULT.json').read_text())
        cfg = 'MAKL' if '_MAKL_' in d.name else 'B7'
        m = M4.run_measures(d, res, 0.15, cfg)
        good &= abs(m['viol_A'] + m['viol_B'] - m['W_viol_anav_unom_b0.2_s']) < 1e-6 and m['F_run'] > 0 and 6 < m['t_ca_after_trigger_s'] < 10
        if cfg == 'MAKL':
            dsw.append(m['d_sw_m'])
    check('archived J1 runs: viol_A + viol_B = frozen W violation; F_run; closest approach 6-10 s after the trigger', good)
    check('archived J1 MAKL (Q1 0.18 Mbit/s, v_hat about 0.17): d_sw in 1.0-1.2 m (model: 1.09 m)',
          dsw and all(x is not None and 1.0 <= x <= 1.2 for x in dsw), dsw)
else:
    print('SKIP archived J1 runs not found at', J1RES, '(metrics on archived runs not checked here)', flush=True)
grid = [{'sim_s': round(10 + 0.1 * k, 1), 'clearance_m': 2.0 - 0.02 * k} for k in range(100)]
mkq = lambda t, d, ch: {'t_us': int(t * 1e6), 'd': d, 'choice': ch, 'q_scan': True, 'q_cam': True}
Dsyn = [mkq(10 + 0.1 * k, 2.0 - 0.02 * k - 0.10, 'camera' if 2.0 - 0.02 * k - 0.10 > 1.0 else 'scan') for k in range(0, 90, 2)]
r = M4.d_switch(Dsyn, grid, 10.0, 19.9)
check('synthetic d_sw: clean switch at 1.0 m found', r['d_sw_m'] is not None and 0.95 <= r['d_sw_m'] <= 1.0, r)
Dsyn2 = Dsyn + [mkq(10.05, 1.85, 'scan')]          # one isolated early scan-first
r2 = M4.d_switch(Dsyn2, grid, 10.0, 19.9)
check('synthetic d_sw: an isolated early scan-first decision does not move d_sw', r2['d_sw_m'] == r['d_sw_m'] and r2['d_first_scan_m'] == 1.85)
r3 = M4.d_switch(Dsyn[:4], grid, 10.0, 19.9)
check('synthetic d_sw: missing with fewer than 6 qualifying decisions', r3['d_sw_m'] is None)
Dw = [dict(x, d=x['d'] - 0.5) for x in Dsyn]                 # d set by a wall, not the pedestrian
check('synthetic d_sw: decisions with d set by a wall do not qualify', M4.d_switch(Dw, grid, 10.0, 19.9)['dsw_n_qualifying'] == 0)

# A1 safety detector: one-tick lag between gated and final command is not a safety action; two zero ticks are
G1 = [{'sim_s': round(10 + 0.1 * k, 1), 'mode': 0, 'guard_latched': False, 'cmd_raw_v': 0.10 if k % 2 else 0.074,
       'cmd_final_v': 0.074 if k % 2 else 0.10, 'raw_scan_min_m': 1.5, 'clearance_m': 1.6} for k in range(20)]
G2 = [dict(r) for r in G1]
for r in G2[10:12]:
    r['cmd_raw_v'] = 0.10; r['cmd_final_v'] = 0.0
G3 = [dict(r) for r in G1]; G3[15]['mode'] = 2; G3[15]['raw_scan_min_m'] = 0.75; G3[15]['clearance_m'] = 0.6
a1, a2, a3 = M4.safety(G1, 10.0, 0.0), M4.safety(G2, 10.0, 0.0), M4.safety(G3, 10.0, 0.0)
check('A1 safety ticks: DWB sample lag ignored; zero output on 2 ticks counted once; mask STOP counted with its raw range',
      a1['safety_ticks'] == 0 and a2['safety_ticks'] == 1 and a2['first_safety_sim_s'] == 11.1 and a3['safety_ticks'] == 1
      and a3['first_safety_raw_m'] == 0.75 and a3['wall_safety_ticks'] == 0, (a1, a2, a3))

# ---------------- 6. run-all helpers ----------------
import j4_run_all as RA
rows = RA.matrix((0.10, 0.15, 0.22))
blocks = {}
for x in rows:
    blocks.setdefault(x['block'], []).append(x)
rot = [tuple(r['config'] for r in b if r['level'] == 'LB') for b in blocks.values()]
check('matrix: 75 runs, 15 blocks of 4 congested arms + 1 B7 C0 run, all 4 rotations used, deltas as J1',
      len(rows) == 75 and len(blocks) == 15 and all(len(b) == 5 and b[-1]['level'] == 'C0' and b[-1]['config'] == 'B7' for b in blocks.values())
      and len(set(rot)) == 4 and sorted({x['delta_s'] for x in rows}) == [-0.4, -0.2, 0.0, 0.2, 0.4])
check('x_meet = 2.40 + V * delta', RA.cfg_for(-0.4, 0.22)['x_meet'] == round(2.4 - 0.088, 4) and RA.cfg_for(0.4, 0.10)['x_meet'] == 2.44)
row = {'v': 0.22, 'config': 'MAKL', 'level': 'C0'}
good_r = {'status': 'SUCCEEDED', 'bridge_config_line': 'J1_BRIDGE config=MAKL link=x', 'j4_stack_lines': {
    'gate': 'J4_MODE_GATE LIN_CAP=(0.22, 0.08, 0.0)', 'guard': 'J4_GUARD V=0.22 SAFE_STOP=0.7 EMERGENCY=0.39 SCAN_MAX_AGE=0.5 MAX_LIN=0.22',
    'speed_json': {'dwb_max_vel_x': 0.22}},
    'e5': {'ledger_config': 'MAKL', 'robot_x_at_arrival': 2.30, 'speedwin_share_within_0.02': 0.65, 'speedwin_ticks': 30,
           'speedwin_v_median': 0.209, 'speedwin_v_min': 0.193,
           'speedwin_cmd_policy_max': 0.22, 'first_safety_raw_m': 0.95, 'wall_safety_ticks': 0, 't_ca_after_trigger_s': 5.5,
           'cmd_policy_max': 0.22}}
check('pilot checks pass on a good run', all(RA.pilot_checks(good_r, row).values()), RA.pilot_checks(good_r, row))
bad_r = json.loads(json.dumps(good_r)); bad_r['e5']['wall_safety_ticks'] = 3
check('pilot P5 fails on a wall safety tick, and P5 is a fallback-eligible failure',
      not RA.pilot_checks(bad_r, row)['P5_no_wall_safety_tick'] and 'P5_no_wall_safety_tick' in RA.FALLBACK_OK)
bad3 = json.loads(json.dumps(good_r)); bad3['e5']['speedwin_v_median'] = 0.19
bad4 = json.loads(json.dumps(good_r)); bad4['e5']['speedwin_v_min'] = 0.10
check('A1 P3: median within 0.02 of V and no tick below 0.5 V (per-tick share no longer used)',
      RA.pilot_checks(good_r, row)['P3_speed_reaches_switching_window'] and not RA.pilot_checks(bad3, row)['P3_speed_reaches_switching_window']
      and not RA.pilot_checks(bad4, row)['P3_speed_reaches_switching_window'])
st0 = {'halted': {'why': '2 robot-moving contacts'}}
check('A1 re-evaluation runs only for a pilot halt (other halts and running states untouched)',
      RA.amend_a1(dict(st0)) == st0 and RA.amend_a1({'halted': None}) == {'halted': None})
bad2 = json.loads(json.dumps(good_r)); bad2['e5']['cmd_policy_max'] = 0.25
check('stack check fails when the planner commanded more than V + 0.01', not RA.stack_ok(bad2, 0.22) and RA.stack_ok(good_r, 0.22))


def fake_resolve(fail_at):
    st = {'done': {}}
    saved = RA.save; RA.save = lambda s: None
    calls = []

    def rps(s, v):
        calls.append(v)
        ch = {k: True for k in ('P1_run_succeeded', 'P2_meeting_0.75_1.05_m_before_crossing', 'P3_speed_reaches_switching_window',
                                'P4_first_safety_after_switching_window', 'P5_no_wall_safety_tick', 'P6_closest_approach_within_20s')}
        for k in fail_at.get(v, ()):
            ch[k] = False
        return [{'checks': ch}, {'checks': ch}]
    orig = RA.run_pilot_set; RA.run_pilot_set = rps
    try:
        lv = RA.resolve_levels(st); out = ('levels', lv, calls)
    except SystemExit as e:
        out = ('halt', str(e), calls)
    RA.run_pilot_set = orig; RA.save = saved
    return out


a = fake_resolve({}); b = fake_resolve({0.22: ['P5_no_wall_safety_tick']}); c = fake_resolve({0.22: ['P2_meeting_0.75_1.05_m_before_crossing']})
d = fake_resolve({0.22: ['P5_no_wall_safety_tick'], 0.20: ['P3_speed_reaches_switching_window']}); e = fake_resolve({0.10: ['P6_closest_approach_within_20s']})
check('pilot rule: all pass -> 0.10/0.15/0.22; 0.22 wall-only -> 0.20 fallback; other 0.22 failure, failing fallback, or a 0.10 failure -> halt',
      a[:2] == ('levels', (0.10, 0.15, 0.22)) and b[:2] == ('levels', (0.10, 0.15, 0.20)) and b[2][-1] == 0.20
      and c[0] == 'halt' and d[0] == 'halt' and e[0] == 'halt', [a[:2], b[:2], c[0], d[0], e[0]])

# ---------------- 7. analysis end to end ----------------
import j4_analysis as AN


def synth_state(kind, seed=1):
    rnd = random.Random(seed); done = {}
    levels = (0.10, 0.15, 0.22)
    for rw in RA.matrix(levels):
        v, a, rep = rw['v'], rw['config'], rw['rep']
        n = lambda s: rnd.uniform(-s, s)
        base_va = {0.10: 1.0, 0.15: 1.1, 0.22: 1.2}[v]
        dsw = {'MAKL': {0.10: 0.80, 0.15: 1.00, 0.22: 1.30}[v], 'MAKL_cap': 1.00, 'DSW_1.0': 1.00}.get(a)
        if kind in ('not_shown', 'invalid') and a == 'MAKL':
            dsw = 1.00
        va = base_va; fr = {'B7': 2.4, 'MAKL': 3.2, 'MAKL_cap': 3.2, 'DSW_1.0': 3.2}[a]
        if kind == 'supported':
            if v == 0.22 and a == 'MAKL_cap':
                va += 1.2 + 0.1 * rep
            if v == 0.10 and a in ('MAKL_cap', 'DSW_1.0'):
                fr -= 0.8
        if kind == 'mechanism_only':
            if v == 0.22 and a in ('MAKL_cap', 'DSW_1.0'):
                va += 0.2 + 0.02 * rep
            if v == 0.10 and a in ('MAKL_cap', 'DSW_1.0'):
                fr -= 0.1
        if rw['level'] == 'C0':
            va = 0.5 + n(0.1); fr = 4.7 + n(0.05); dsw = None
        e5 = {'W_viol_anav_unom_b0.2_s': round(va + 0.3 + n(0.05), 3), 'viol_A': round(va + n(0.05), 3), 'F_run': round(fr + n(0.05), 3),
              'F_A': fr, 'F_B': fr, 'F_fps': fr * 0.7, 'd_sw_m': None if dsw is None else round(dsw + n(0.02), 3), 'contact_class': None,
              'impl_n': 50 if a in ('MAKL', 'MAKL_cap') else 0, 'impl_ok': (kind != 'invalid') if a in ('MAKL', 'MAKL_cap') else None,
              'sanity_share_W': 1.0 if a in ('MAKL', 'MAKL_cap') else None, 'agree_share_W': 0.97 if a in ('MAKL', 'MAKL_cap') else None}
        done[rw['trial_id']] = {'status': 'SUCCEEDED', 'evidence_dir': '/nonexistent', 'j1_config': a, 'j1_cell': rw['cell'],
                                'j1_rep': rep, 'j1_delta_s': rw['delta_s'], 'j4_v': v, 'e5': e5, 'order': len(done) + 1}
    return {'done': done, 'levels': list(levels), 'top_level_reason': 'planned'}


res = {}
for kind in ('supported', 'mechanism_only', 'not_shown', 'invalid'):
    p = TMP / f'S_{kind}.json'; p.write_text(json.dumps(synth_state(kind)))
    o = TMP / f'O_{kind}'; sys.argv = ['j4_analysis.py', str(p), str(o)]
    with contextlib.redirect_stdout(io.StringIO()):
        AN.main()
    V = json.loads((o / 'J4_VERDICTS.json').read_text()); res[kind] = (V['speed_term'], V['distance_switch'])
check('analysis designed outcomes', res == {'supported': ('SPEED_TERM_SUPPORTED', 'DSW_ADVANTAGE'),
                                            'mechanism_only': ('SPEED_TERM_MECHANISM_ONLY', 'DSW_SIGN_ONLY'),
                                            'not_shown': ('SPEED_TERM_NOT_SHOWN', 'DSW_NOT_SHOWN'),
                                            'invalid': ('INVALID_SPEED_IMPLEMENTATION', 'DSW_NOT_SHOWN')}, res)
V = json.loads((TMP / 'O_supported' / 'J4_VERDICTS.json').read_text())
check('J per speed = max(J1 value, C0 range) and >= the J1 values', all(V['J_per_speed'][k]['viol_A'] >= 0.8 and V['J_per_speed'][k]['F_run'] >= 0.4
                                                                         for k in V['J_per_speed']))

# ---------------- 8. compile ----------------
cc = True
for f in sorted(HERE.rglob('*.py')):
    try:
        py_compile.compile(str(f), doraise=True, cfile=str(TMP / (f.name + 'c')))
    except Exception as ex:
        cc = False; print(ex)
check('every script compiles', cc)
print('J4_OFFLINE_TESTS', 'PASS' if ok else 'FAIL', flush=True)
sys.exit(0 if ok else 1)
