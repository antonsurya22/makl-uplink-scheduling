#!/usr/bin/env python3
"""J4 batch (protocol J4_PROTOCOL_FREEZE.md). Simulation only. Resumable; finished runs are never repeated.
  0. Checks before every run: J4 freeze hashes; J3, J1 v2, P0-J, E7 (article E6) and E5 packages unchanged.
  1. LOCK in STATE before any pilot, gate or counted run.
  2. PILOTS (not counted): MAKL, no congestion (C0), Q1, at each speed level, start shifts -0.4 and +0.4 s.
     Checks per run (protocol 4 as amended by A1): P1 run succeeded; P2 the pedestrian reaches the path when the robot is
     0.75-1.05 m before the crossing; P3 speed reaches the switching window (from the pedestrian's start until d < D_LOW[V]:
     median robot speed within 0.02 m/s of V, no tick below 0.5 V, >= 5 ticks; planner command <= V + 0.01; A1 replaced
     the v0 test '>= 80 % of ticks within 0.02 m/s', which measured DWB's velocity sampling); P4 first safety action after
     the switching window (raw range at
     the first safety tick < D_LOW[V] + 0.22 m); P5 no safety tick caused by a wall; P6 closest approach <= 20 s after
     the trigger. Pre-declared fallback: if a 0.22 m/s pilot fails ONLY P3/P4/P5, the top level becomes 0.20 m/s and its
     two pilots are run; if those pass, 0.20 m/s replaces 0.22 m/s everywhere. Any other pilot failure halts J4
     (an amendment is needed before any counted run).
  3. GATES (3 runs, not counted, all must pass; Q1 at 0.18 Mbit/s, 25 s): G1 MAKL at 0.10 m/s, delta -0.4 s;
     G2 MAKL_cap at the top level, delta +0.4 s; G3 DSW_1.0 at the top level, delta 0.
     Checks: the J3 gate checks + P3 + speed implementation (MAKL: speed in a* = v_hat; MAKL_cap: = 0.15).
  4. MATRIX: blocks (rep, V): the 4 arms (B7 MAKL MAKL_cap DSW_1.0) with congestion (LB-Q1: Q1 at 0.18 Mbit/s, 25 s from
     the trigger), arm order rotated per block, then one B7 run without congestion (C0-Q1). 5 reps x 3 levels x 5 = 75
     counted runs. Pedestrian 0.50 m/s, E5/J1 'meet' start with x_meet = 2.40 + V * delta (delta -0.4 ... +0.4 s).
Halts: 2 robot-moving contacts; hard-guard death; bridge/ledger configuration mismatch; capacity plan mismatch; onboard
inputs missing; speed level not applied (planner command > V + 0.01, or the gate/guard line missing); speed
implementation check failed in a MAKL or MAKL_cap run that moved.
AMENDMENT A1 (J4_PROTOCOL_AMENDMENT_A1.md): if the STATE is halted by the pilot rule and A1 has not been applied, the six
pilots already run are re-evaluated from their evidence with the A1 measures and checks (no pilot is re-run); the result
and the old halt are kept in STATE['amendment_A1']; the run continues only if the pilot rule passes under A1."""
import datetime, hashlib, json, os, secrets, subprocess, sys, time, zipfile
from pathlib import Path
PJ = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
E5 = WS / 'SCI_E5_PED_SPEED_20261101'
E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'
P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
J3 = WS / 'SCI_J3_SWITCH_ABLATION_20261116'
BASE = WS / 'results/SCI_J4_SPEED_ABLATION_20261117'
MAIN = BASE / 'J4_MAIN'
ARMS = ('B7', 'MAKL', 'MAKL_cap', 'DSW_1.0')
ONBOARD = ('MAKL', 'MAKL_cap', 'DSW_1.0')
LEVELS_PLANNED = (0.10, 0.15, 0.22)
TOP_FALLBACK = 0.20
DUR = 25
DELTA = {1: -0.4, 2: -0.2, 3: 0.0, 4: 0.2, 5: 0.4}
SPEED = 0.50
NEW = dict(x=3.2, y0=-1.5, y1=1.40)
X_MEET = 2.4
CROSS_X = 3.2
PINS = ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
        (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2'), (J1, 'J1_FREEZE_HASHES.json', 'befd9065'),
        (J3, 'J3_FREEZE_HASHES.json', 'b0ba0679'))
sys.path.insert(0, str(PJ)); sys.path.insert(0, str(J1)); sys.path.insert(0, str(E5)); sys.path.insert(0, str(E3B))
import j4_metrics as M4


def vtag(v):
    return 'V%03d' % int(round(v * 100))


def policy():
    return 'F-N1'                                   # NORMAL mode, camera Q1 (as J1-J3 Q1 cells)


def cfg_for(delta, v):
    return dict(NEW, speed=SPEED, start_mode='meet', x_meet=round(X_MEET + v * delta, 4))


def plan_for(level):
    import e3b_policy_logic as L3
    p = dict(L3.plan(level))
    if level != 'C0':
        p['recover_after_trigger_s'] = float(DUR)
    return p


def matrix(levels):
    rows = []; b = 0
    for rep in (1, 2, 3, 4, 5):
        for v in levels:
            k = list(ARMS); r = b % len(ARMS); k = k[r:] + k[:r]; b += 1
            for a in k:
                rows.append({'trial_id': f'J4_R{rep}_{vtag(v)}_LB-Q1_{a}', 'rep': rep, 'v': v, 'cell': f'{vtag(v)}|LB-Q1',
                             'level': 'LB', 'config': a, 'delta_s': DELTA[rep], 'block': b})
            rows.append({'trial_id': f'J4_R{rep}_{vtag(v)}_C0-Q1_B7', 'rep': rep, 'v': v, 'cell': f'{vtag(v)}|C0-Q1',
                         'level': 'C0', 'config': 'B7', 'delta_s': DELTA[rep], 'block': b})
    assert len(rows) == 75 and len({r['trial_id'] for r in rows}) == 75
    return rows


def pilots(v):
    return [{'trial_id': f'PILOT_J4_{vtag(v)}_C0-Q1_MAKL_d{"m" if d < 0 else "p"}04', 'rep': 'P', 'v': v,
             'cell': f'{vtag(v)}|C0-Q1', 'level': 'C0', 'config': 'MAKL', 'delta_s': d, 'block': 0} for d in (-0.4, 0.4)]


def gates(top):
    return ({'trial_id': f'GATE_J4_G1_{vtag(0.10)}_LB-Q1_MAKL', 'rep': 'G1', 'v': 0.10, 'cell': f'{vtag(0.10)}|LB-Q1',
             'level': 'LB', 'config': 'MAKL', 'delta_s': -0.4, 'block': 0},
            {'trial_id': f'GATE_J4_G2_{vtag(top)}_LB-Q1_MAKL_cap', 'rep': 'G2', 'v': top, 'cell': f'{vtag(top)}|LB-Q1',
             'level': 'LB', 'config': 'MAKL_cap', 'delta_s': 0.4, 'block': 0},
            {'trial_id': f'GATE_J4_G3_{vtag(top)}_LB-Q1_DSW_1.0', 'rep': 'G3', 'v': top, 'cell': f'{vtag(top)}|LB-Q1',
             'level': 'LB', 'config': 'DSW_1.0', 'delta_s': 0.0, 'block': 0})


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def say(*a):
    print(*a, flush=True)


def save(s):
    t = MAIN / 'STATE.json.new'; t.write_text(json.dumps(s, indent=2, default=str) + '\n'); os.replace(t, MAIN / 'STATE.json')


def verify_freeze():
    fz = json.loads((PJ / 'J4_FREEZE_HASHES.json').read_text())
    bad = [f for f, h in fz['sha256'].items() if not (PJ / f).is_file() or sha(PJ / f) != h]
    if bad:
        raise SystemExit('J4_FROZEN_FILES_CHANGED ' + repr(bad))
    for P, name, pref in PINS:
        z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if not (P / f).is_file() or sha(P / f) != h]
        if bad or not sha(P / name).startswith(pref):
            raise SystemExit('PACKAGE_CHANGED ' + P.name + ' ' + repr(bad))
    return fz


def run_one(row, attempt, kind):
    e5cfg = cfg_for(row['delta_s'], row['v'])
    plan = plan_for(row['level'])
    cell = row['cell'].split('|')[1]
    spec = dict(trial_id=row['trial_id'], level=row['level'], arm=policy(), rep=str(row['rep']), seed='',
                block=str(row['rep']), e5=e5cfg,
                j1={'config': row['config'], 'plan': plan, 'cell': cell, 'duration_s': DUR, 'delta_s': row['delta_s']},
                j4={'v_level': row['v']},
                scope_token=secrets.token_hex(24), actor_trace_sha256=sha(E5 / 'e5_crossing.py'))
    tag = f"{row['trial_id']}_a{attempt}_{now()}"; out = MAIN / 'trials' / tag; t0 = time.monotonic()
    p = subprocess.run(['/usr/bin/python3', '-u', str(PJ / 'j4_run_trial.py'), json.dumps(spec), str(out)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=2400)
    if out.is_dir():
        (out / 'RUN_TRIAL_CONSOLE.log').write_text(p.stdout)
    res = json.loads((out / 'TRIAL_RESULT.json').read_text()) if (out / 'TRIAL_RESULT.json').is_file() else \
        {'status': 'PRE_MOTION_BLOCKED', 'error': p.stdout[-800:]}
    res.update(wall_total_s=round(time.monotonic() - t0, 1), attempts=attempt)
    return post(out, row, res, kind)


def post(out, row, res, kind):
    """Measures of one finished run (also used by the A1 re-evaluation of the pilots)."""
    res.update(evidence_dir=str(out), j1_kind=kind,
               j1_config=row['config'], j1_cell=row['cell'], j1_level=row['level'], j1_profile='Q1',
               j1_duration_s=DUR, j1_rep=row['rep'], j1_delta_s=row['delta_s'], j1_plan=plan_for(row['level']),
               e5_cfg=cfg_for(row['delta_s'], row['v']), speed=SPEED, geometry='new', j4_v=row['v'])
    log = (out / 'c52_joint_bridge.log').read_text(errors='replace') if (out / 'c52_joint_bridge.log').is_file() else ''
    res['bridge_config_line'] = next((l for l in log.splitlines() if l.startswith('J1_BRIDGE config=')), None)
    res['j4_stack_lines'] = stack_lines(out)
    try:
        res['e5'] = M4.run_measures(out, res, row['v'], row['config'])
    except Exception as ex:
        res['e5'] = {'error': repr(ex)}
    if row['config'] == 'DSW_1.0':
        res['j3_dsw'] = dsw_summary(out, res)
    return res


def dsw_summary(out, res):
    """As J3: threshold in the decision-file header, decisions in W, share of W decisions with d < threshold."""
    f = out / 'J1_DECISIONS.jsonl'
    if not f.is_file():
        return {'error': 'no decision file'}
    hdr = None; D = []
    for l in f.read_text().splitlines():
        try:
            x = json.loads(l)
        except Exception:
            continue
        if x.get('header'):
            hdr = x
        elif x.get('kind') == 'decision':
            D.append(x)
    ev = [e.get('event', {}) for e in (res.get('network_events') or [])]
    t0 = next((e.get('trigger_sim_us') for e in ev if e.get('event') == 'trigger'), None)
    W = [x for x in D if t0 is not None and t0 <= x['t_us'] < t0 + DUR * 1_000_000]
    th = ((hdr or {}).get('effective_params') or {}).get('d_th_m')
    return {'d_th_m': th, 'decisions_W': len(W),
            'scan_first_share_W': round(sum(x['d'] < th for x in W) / len(W), 4) if W and th else None,
            'cam_chosen_W': sum(x['choice'] == 'camera' for x in W), 'scan_chosen_W': sum(x['choice'] == 'scan' for x in W)}


def stack_lines(out):
    g = (out / 'e2_mode_gate.log').read_text(errors='replace') if (out / 'e2_mode_gate.log').is_file() else ''
    h = (out / 'final_hard_guard.log').read_text(errors='replace') if (out / 'final_hard_guard.log').is_file() else ''
    s = json.loads((out / 'J4_SPEED.json').read_text()) if (out / 'J4_SPEED.json').is_file() else None
    return {'gate': next((l for l in g.splitlines() if l.startswith('J4_MODE_GATE ')), None),
            'guard': next((l for l in h.splitlines() if l.startswith('J4_GUARD ')), None), 'speed_json': s}


def started(r):
    return bool(r.get('bridge_config_line'))


def config_ok(r, cfg):
    if r.get('status') == 'PRE_MOTION_BLOCKED' and not started(r):
        return True
    m = r.get('e5') or {}
    return (r.get('bridge_config_line') or '').startswith(f'J1_BRIDGE config={cfg} ') and m.get('ledger_config') == cfg


def stack_ok(r, v):
    """The speed level was applied: DWB YAML, gate and guard lines, and the planner never commanded more than V + 0.01."""
    if r.get('status') == 'PRE_MOTION_BLOCKED' and not started(r):
        return True
    s = r.get('j4_stack_lines') or {}; m = r.get('e5') or {}
    sj = s.get('speed_json') or {}
    gate = s.get('gate') or ''; guard = s.get('guard') or ''
    ok = abs((sj.get('dwb_max_vel_x') or -1) - v) < 1e-9 and gate.startswith('J4_MODE_GATE LIN_CAP=(%s,' % float(v)) \
        and ('MAX_LIN=%s' % max(0.18, float(v))) in guard
    cp = m.get('cmd_policy_max')
    return bool(ok and (cp is None or cp <= v + 0.01))


def plan_ok(r, level, dur=DUR):
    import e3b_policy_logic as L
    ev = [e.get('event', {}) for e in (r.get('network_events') or [])]
    trig = [e for e in ev if e.get('event') == 'trigger']
    ph = {e.get('new_phase'): e for e in ev if e.get('event') == 'network_phase'}
    if level == 'C0':
        return 'congested' not in ph
    if r.get('goal_sim_s') is None:
        return True
    t = trig[0].get('trigger_sim_us') if trig else None
    c = ph.get('congested', {}); rc = ph.get('recovery', {})
    ok = bool(t) and len(trig) == 1 and c.get('boundary_sim_us') == t and c.get('new_bps') == int(round(L.LEVELS[level] * 1_000_000))
    if ok and rc:
        ok = rc.get('boundary_sim_us') == t + int(round(dur * 1e6))
    elif ok and r.get('terminal_sim_s') is not None and r['terminal_sim_s'] * 1e6 > t + dur * 1e6 + 1e6:
        ok = False
    return ok


def inputs_ok(r, cfg):
    if cfg not in ONBOARD or r.get('goal_sim_s') is None:
        return True
    m = r.get('e5') or {}
    ok = (m.get('j1_inputs_odom') or 0) > 0 and (m.get('j1_inputs_cmd') or 0) > 0 and (m.get('j1_inputs_scans') or 0) > 0 \
        and (m.get('makl_decisions_all') or 0) > 0
    if cfg == 'MAKL':
        ok = ok and (m.get('makl_decisions_v_pos_all') or 0) > 0
    if cfg == 'DSW_1.0':
        ok = ok and (r.get('j3_dsw') or {}).get('d_th_m') == 1.0
    return ok


def impl_ok(r, cfg):
    """MAKL: speed in a* = v_hat in every decision; MAKL_cap: = 0.15 in every decision (and v_hat logged)."""
    if cfg not in ('MAKL', 'MAKL_cap') or r.get('goal_sim_s') is None:
        return True
    m = r.get('e5') or {}
    return m.get('impl_ok') is True and (m.get('impl_n') or 0) > 0


def hard_guard_death(r):
    return 'OWNED_PROCESS_DIED_final_hard_guard' in str(r.get('error') or '')


def with_retries(row, kind):
    for a in (1, 2, 3):
        r = run_one(row, a, kind); m = r.get('e5') if isinstance(r.get('e5'), dict) else {}
        say(f"   {row['trial_id']} attempt {a}: {r.get('status')} config_ok={config_ok(r, row['config'])} "
            f"stack_ok={stack_ok(r, row['v'])} plan_ok={plan_ok(r, row['level'])} inputs_ok={inputs_ok(r, row['config'])} "
            f"impl_ok={impl_ok(r, row['config'])} contact={m.get('contact_class')} viol={m.get('W_viol_anav_unom_b0.2_s')} "
            f"viol_A={m.get('viol_A')} F_run={m.get('F_run')} d_sw={m.get('d_sw_m')} t_ca={m.get('t_ca_after_trigger_s')} "
            f"speedwin={m.get('speedwin_share_within_0.02')} cmdmax={m.get('cmd_policy_max')} "
            f"safety_raw={m.get('first_safety_raw_m')} wall={m.get('wall_safety_ticks')} "
            f"meet={m.get('robot_x_at_arrival')} {str(r.get('error') or '')[:100]}")
        if r.get('status') != 'PRE_MOTION_BLOCKED' or hard_guard_death(r):
            return r
        time.sleep(5)
    return r


def pilot_checks(r, row):
    m = r.get('e5') or {}; v = row['v']
    xa = m.get('robot_x_at_arrival'); fs = m.get('first_safety_raw_m')
    tca = m.get('t_ca_after_trigger_s')
    return {'P1_run_succeeded': r.get('status') == 'SUCCEEDED' and stack_ok(r, v) and config_ok(r, row['config']),
            'P2_meeting_0.75_1.05_m_before_crossing': xa is not None and 0.75 <= CROSS_X - xa <= 1.05,
            'P3_speed_reaches_switching_window': m.get('speedwin_v_median') is not None and abs(m['speedwin_v_median'] - v) <= 0.02
            and (m.get('speedwin_v_min') or 0) >= 0.5 * v and (m.get('speedwin_ticks') or 0) >= 5
            and (m.get('speedwin_cmd_policy_max') or 9) <= v + 0.01,
            'P4_first_safety_after_switching_window': fs is None or fs < M4.D_LOW[v] + 0.22,
            'P5_no_wall_safety_tick': (m.get('wall_safety_ticks') or 0) == 0,
            'P6_closest_approach_within_20s': tca is not None and tca <= 20.0}


FALLBACK_OK = {'P3_speed_reaches_switching_window', 'P4_first_safety_after_switching_window', 'P5_no_wall_safety_tick'}


def gate_checks(r, row):
    m = r.get('e5') or {}
    cfg = row['config']
    ch = {'run_succeeded': r.get('status') == 'SUCCEEDED',
          'capacity_plan_nominal': plan_ok(r, row['level']) and m.get('event_congested_bps') is not None,
          'a_nav_logged': bool(r.get('a_nav_logged')),
          'arrival_within_0.15s': m.get('arrival_error_s') is not None and abs(m['arrival_error_s']) <= 0.15,
          'bridge_and_ledger_config': config_ok(r, cfg) and started(r),
          'speed_level_applied': stack_ok(r, row['v']),
          'speed_reaches_switching_window': pilot_checks(r, row)['P3_speed_reaches_switching_window'],
          'onboard_inputs_received': inputs_ok(r, cfg) and r.get('goal_sim_s') is not None,
          'decisions_W_ge_20_both_streams': (m.get('makl_decisions_W') or 0) >= 20 and (m.get('makl_scan_chosen_W') or 0) >= 1
          and (m.get('makl_cam_chosen_W') or 0) >= 1}
    if cfg in ('MAKL', 'MAKL_cap'):
        ch['speed_used_in_astar'] = impl_ok(r, cfg) and (m.get('sanity_share_W') or 0) >= 0.99
    if cfg == 'DSW_1.0':
        ch['dsw_threshold_logged'] = (r.get('j3_dsw') or {}).get('d_th_m') == 1.0
    return ch


def halt(st, why, **k):
    st['halted'] = dict(why=why, at=now(), **k); save(st); raise SystemExit('J4_HALT ' + why)


def finish(st):
    an = MAIN / 'final'; an.mkdir(exist_ok=True)
    p = subprocess.run(['/usr/bin/python3', str(PJ / 'j4_analysis.py'), str(MAIN / 'STATE.json'), str(an)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (an / 'ANALYSIS_CONSOLE.log').write_text(p.stdout); say(p.stdout[-4000:])
    z = Path.home() / 'Downloads' / f'SCI_J4_RESULTS_{now()}.zip'
    keep = ('TRIAL_RESULT.json', 'TRIAL_SPEC.json', 'GRAPH_ASSERT.json', 'c52_joint_bridge.log', 'RUN_TRIAL_CONSOLE.log',
            'E2_CROSSING_GROUND_TRUTH.jsonl', 'controller_server.log', 'ERROR.txt', 'E5_AMCL_TOL.json', 'J1_LEDGER.jsonl',
            'J1_DECISIONS.jsonl', 'J4_SPEED.json', 'e2_mode_gate.log', 'final_hard_guard.log')
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zz:
        zz.write(MAIN / 'STATE.json', 'STATE.json')
        for f in sorted(an.rglob('*')):
            if f.is_file():
                zz.write(f, 'final/' + f.name)
        dirs = [r['evidence_dir'] for r in st['done'].values()] + \
               [h['evidence_dir'] for h in st.get('gate_history', []) + st.get('pilot_history', []) if h.get('evidence_dir')]
        for d in dirs:
            d = Path(d)
            for f in sorted(d.glob('*')) if d.is_dir() else []:
                if f.name in keep or f.name.startswith('E3B_'):
                    zz.write(f, 'trials/' + d.name + '/' + f.name)
        for f in ('J4_PROTOCOL_FREEZE.md', 'J4_FREEZE_HASHES.json'):
            zz.write(PJ / f, 'SOURCE/' + f)
    say('J4_RESULTS_ZIP', z, sha(z))


def run_pilot_set(st, v):
    P = st.setdefault('pilots', {})
    for row in pilots(v):
        if row['trial_id'] in P:
            continue
        say(f"=== PILOT {row['trial_id']} (not counted, V {v}, delta {row['delta_s']:+.1f} s) ==="); verify_freeze()
        r = with_retries(row, 'pilot'); ch = pilot_checks(r, row); m = r.get('e5') or {}
        rec = {'v': v, 'checks': ch, 'passed': all(ch.values()), 'status': r.get('status'), 'evidence_dir': r.get('evidence_dir'),
               'measures': {k: m.get(k) for k in ('robot_x_at_arrival', 'speedwin_share_within_0.02', 'speedwin_ticks',
                                                  'speedwin_v_median', 'speedwin_v_min',
                                                  'speedwin_cmd_policy_max', 'first_safety_raw_m', 'first_safety_sim_s',
                                                  'wall_safety_ticks', 'safety_ticks', 't_ca_after_trigger_s', 'clear_ca_m',
                                                  't_run_after_trigger_s', 'cmd_policy_max', 'v_hat_med_W')}}
        P[row['trial_id']] = rec; st.setdefault('pilot_history', []).append(dict(rec, trial_id=row['trial_id'])); save(st)
        say('J4_PILOT', row['trial_id'], json.dumps(rec, default=str))
    return [P[r['trial_id']] for r in pilots(v)]


def amend_a1(st):
    """A1: re-evaluate the pilots already run (evidence on disk) with the A1 measures and checks; clear the pilot halt."""
    if st.get('amendment_A1') or not st.get('halted'):
        return st
    if not str(st['halted'].get('why', '')).startswith('pilot failed'):
        return st
    old = {'halted': st['halted'], 'pilots_v0': st.get('pilots')}
    newp = {}
    for v in LEVELS_PLANNED:
        for row in pilots(v):
            rec = (st.get('pilots') or {}).get(row['trial_id'])
            if not rec or not rec.get('evidence_dir') or not Path(rec['evidence_dir']).is_dir():
                raise SystemExit('J4_A1_PILOT_EVIDENCE_MISSING ' + row['trial_id'])
            out = Path(rec['evidence_dir'])
            res = json.loads((out / 'TRIAL_RESULT.json').read_text())
            r = post(out, row, res, 'pilot'); ch = pilot_checks(r, row); m = r.get('e5') or {}
            newp[row['trial_id']] = {'v': v, 'checks': ch, 'passed': all(ch.values()), 'status': r.get('status'),
                                     'evidence_dir': str(out), 'reevaluated_under': 'A1',
                                     'measures': {k: m.get(k) for k in ('robot_x_at_arrival', 'speedwin_share_within_0.02',
                                                                       'speedwin_v_median', 'speedwin_v_min', 'speedwin_ticks',
                                                                       'speedwin_cmd_policy_max', 'first_safety_raw_m',
                                                                       'first_safety_sim_s', 'wall_safety_ticks', 'safety_ticks',
                                                                       't_ca_after_trigger_s', 'clear_ca_m', 'v_hat_med_W')}}
            say('J4_A1_PILOT', row['trial_id'], json.dumps(newp[row['trial_id']], default=str))
    st['amendment_A1'] = dict(old, at=now(), freeze_file_sha256=sha(PJ / 'J4_FREEZE_HASHES.json'))
    st['pilots'] = newp; st['halted'] = None; st.pop('levels', None)
    save(st)
    return st


def resolve_levels(st):
    """Pilots; returns the three counted levels or halts."""
    if st.get('levels'):
        return tuple(st['levels'])
    fails = {}
    for v in LEVELS_PLANNED:
        recs = run_pilot_set(st, v)
        fails[v] = sorted({k for rec in recs for k, ok in rec['checks'].items() if not ok})
    top = 0.22
    if fails[0.10] or fails[0.15]:
        halt(st, 'pilot failed (amendment required)', fails={str(k): x for k, x in fails.items()})
    if fails[0.22]:
        if not set(fails[0.22]) <= FALLBACK_OK:
            halt(st, 'pilot failed at 0.22 m/s (amendment required)', fails={str(k): x for k, x in fails.items()})
        recs = run_pilot_set(st, TOP_FALLBACK)
        f20 = sorted({k for rec in recs for k, ok in rec['checks'].items() if not ok})
        if f20:
            halt(st, 'pilot failed at 0.22 and at the fallback 0.20 m/s (amendment required)', fails_022=fails[0.22], fails_020=f20)
        top = TOP_FALLBACK
        say('J4_FALLBACK top level 0.20 m/s (0.22 failed only %s)' % fails[0.22])
    st['levels'] = [0.10, 0.15, top]; st['top_level_reason'] = 'planned' if top == 0.22 else 'pre-declared fallback'
    save(st); say('J4_LEVELS', st['levels'])
    return tuple(st['levels'])


def main():
    if not (os.getenv('SCI_SIM_ONLY') == 'YES' and os.getenv('SCI_E2_NO_PHYSICAL_ROBOT') == 'YES'):
        raise SystemExit('J4_SIM_ONLY_ACKS_REQUIRED')
    os.environ['SCI_E3B_EDGE_NAV'] = 'YES'
    fz = verify_freeze(); MAIN.mkdir(parents=True, exist_ok=True); (MAIN / 'trials').mkdir(exist_ok=True)
    st = json.loads((MAIN / 'STATE.json').read_text()) if (MAIN / 'STATE.json').is_file() else \
        {'created': now(), 'freeze': fz, 'done': {}, 'halted': None}
    st = amend_a1(st)
    if st.get('halted'):
        raise SystemExit('J4_HALTED_REVIEW_REQUIRED ' + json.dumps(st['halted']))
    if 'locked' not in st:
        st['locked'] = {'at': now(), 'freeze_file_sha256': sha(PJ / 'J4_FREEZE_HASHES.json'), 'x_meet': X_MEET,
                        'delta_s': DELTA, 'levels_planned': LEVELS_PLANNED, 'top_fallback': TOP_FALLBACK, 'arms': ARMS,
                        'plans': {lv: plan_for(lv) for lv in ('LB', 'C0')}, 'd_low': M4.D_LOW}
        save(st); say('J4_LOCKED', json.dumps(st['locked'], default=str))
    levels = resolve_levels(st)
    g = st.setdefault('gate', {'runs': {}})
    for row in gates(levels[-1]):
        if row['trial_id'] in g['runs'] and g['runs'][row['trial_id']].get('passed'):
            continue
        if row['trial_id'] in g['runs']:
            halt(st, 'gate failed earlier', gate=row['trial_id'])
        say(f"=== GATE {row['trial_id']} (not counted, delta {row['delta_s']:+.1f} s) ==="); verify_freeze()
        r = with_retries(row, 'gate'); ch = gate_checks(r, row); m = r.get('e5') or {}
        rec = {'checks': ch, 'passed': all(ch.values()), 'status': r.get('status'), 'evidence_dir': r.get('evidence_dir'),
               'dsw': r.get('j3_dsw'), 'decisions': {k: m.get(k) for k in m if k.startswith('makl_') or k.startswith('j1_inputs')},
               'speed': {k: m.get(k) for k in ('impl_ok', 'impl_n', 'sanity_share_W', 'agree_share_W', 'v_hat_med_W', 'd_sw_m',
                                               'speedwin_share_within_0.02', 'cmd_policy_max')}}
        g['runs'][row['trial_id']] = rec; st.setdefault('gate_history', []).append(dict(rec, trial_id=row['trial_id'])); save(st)
        say('J4_GATE', row['trial_id'], json.dumps(rec, default=str))
        if not rec['passed']:
            halt(st, 'gate failed', gate=row['trial_id'])
    g['passed'] = True; save(st)
    for row in matrix(levels):
        if row['trial_id'] in st['done']:
            continue
        verify_freeze(); say(f"=== {row['trial_id']} (block {row['block']}, delta {row['delta_s']:+.1f} s) ===")
        r = with_retries(row, 'counted')
        r.update(order=len(st['done']) + 1, block=row['block']); st['done'][row['trial_id']] = r; save(st)
        if not config_ok(r, row['config']):
            halt(st, 'bridge/ledger configuration mismatch', trial=row['trial_id'])
        if not stack_ok(r, row['v']):
            halt(st, 'speed level not applied', trial=row['trial_id'])
        if not plan_ok(r, row['level']):
            halt(st, 'capacity plan mismatch', trial=row['trial_id'])
        if not inputs_ok(r, row['config']):
            halt(st, 'onboard inputs missing', trial=row['trial_id'])
        if not impl_ok(r, row['config']):
            halt(st, 'speed implementation check failed', trial=row['trial_id'])
        if hard_guard_death(r):
            halt(st, 'hard-guard death', trial=row['trial_id'])
        rm = sum(1 for x in st['done'].values() if (x.get('e5') or {}).get('contact_class') == 'robot_moving')
        if rm >= 2:
            halt(st, '2 robot-moving contacts')
    st['completed'] = now(); save(st); finish(st)


if __name__ == '__main__':
    main()
