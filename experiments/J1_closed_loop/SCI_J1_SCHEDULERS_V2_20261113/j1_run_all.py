#!/usr/bin/env python3
"""J1 v2 batch (protocol J1_PROTOCOL_FREEZE.md, amendment A1). v2 changes only the gate check of the capacity plan
(v1 demanded a recovery boundary even when the run ended before it) and adds the thread CPU time per MAKL decision.
J1 batch (protocol J1_PROTOCOL_FREEZE.md). Simulation only. Resumable; finished runs are never repeated.
  0. Checks before every run: J1 freeze hashes; P0-J, E7 (article E6) and E5 packages unchanged (freeze files + hashes).
  1. LOCK in STATE (freeze-file hash, time, calibration, deltas, cells) before any gate or counted run.
  2. GATE (2 runs, not counted, both must pass; delta extremes, E8 lesson):
       G1 MAKL, LC (0.13 Mbit/s) Q2 25 s, delta -0.4 s;   G2 MAKL, LB (0.18 Mbit/s) Q1 8 s, delta +0.4 s.
     Checks: run SUCCEEDED; congestion boundary at the trigger with the cell's capacity; recovery boundary = trigger +
     the cell's duration; a_nav logged; pedestrian arrival within 0.15 s; bridge line + ledger header = MAKL; MAKL inputs
     received (odometry > 0, Nav2 command > 0, decisions with v > 0); >= 20 MAKL decisions in W with both streams chosen
     at least once; CPU time logged.
  3. MATRIX: 8 arms x 6 cells x 5 paired start shifts (delta -0.4..+0.4 s) = 240 counted runs, pedestrian 0.50 m/s.
     Cells: C0-Q2, C0-Q1, LC-Q2-25s, LC-Q2-8s, LB-Q1-25s, LB-Q1-8s. Block = one delta x cell with all 8 arms; arm order
     rotated by one position per block.
Rules (as E5-X3): a run that fails after goal acceptance is DATA (never retried); pre-motion: up to 3 attempts; contacts and
emergency stops are OUTCOMES. Halts: 2 robot-moving contacts; hard-guard death; bridge/ledger configuration mismatch;
capacity plan mismatch; MAKL inputs missing in a MAKL run that moved."""
import datetime, hashlib, json, os, secrets, subprocess, sys, time, zipfile
from pathlib import Path
PJ = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
E5 = WS / 'SCI_E5_PED_SPEED_20261101'
E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'
P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
BASE = WS / 'results/SCI_J1_SCHEDULERS_V2_20261113'
V1_MAIN = WS / 'results/SCI_J1_SCHEDULERS_20261112/J1_MAIN'      # v1: halted at its first gate (gate-check error); archived in the ZIP
MAIN = BASE / 'J1_MAIN'
ARMS = ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
CELLS = (('C0', 'Q2', 25), ('C0', 'Q1', 25), ('LC', 'Q2', 25), ('LC', 'Q2', 8), ('LB', 'Q1', 25), ('LB', 'Q1', 8))
DELTA = {1: -0.4, 2: -0.2, 3: 0.0, 4: 0.2, 5: 0.4}
V_NOM = 0.15
SPEED = 0.50
NEW = dict(x=3.2, y0=-1.5, y1=1.40)
E5_LOCK = {'0.5': {'x_meet': 2.4}, '0.08': {'x_meet': 2.2, 'preroll_s': 9.01}}
PINS = ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
        (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2'))


def cell_name(c):
    lv, prof, dur = c
    return f'{lv}-{prof}' if lv == 'C0' else f'{lv}-{prof}-{dur}s'


def policy(prof):
    return 'F-N1' if prof == 'Q1' else 'F-N2'


def cfg_for(delta):
    """E5-locked 'meet' start (0.50 m/s) shifted by delta seconds: X_MEET + 0.15*delta (as X1-X3)."""
    return dict(NEW, speed=SPEED, start_mode='meet', x_meet=round(E5_LOCK['0.5']['x_meet'] + V_NOM * delta, 4))


def plan_for(level, dur):
    import e3b_policy_logic as L3
    p = dict(L3.plan(level))
    if level != 'C0':
        p['recover_after_trigger_s'] = float(dur)
    return p


def matrix():
    rows = []; b = 0
    for rep in (1, 2, 3, 4, 5):
        for c in CELLS:
            k = list(ARMS); r = b % len(ARMS); k = k[r:] + k[:r]; b += 1
            rows += [{'trial_id': f'J1_R{rep}_{cell_name(c)}_{a}', 'rep': rep, 'cell': cell_name(c), 'level': c[0],
                      'profile': c[1], 'duration_s': c[2], 'config': a, 'delta_s': DELTA[rep], 'block': b} for a in k]
    assert len(rows) == 240 and len({r['trial_id'] for r in rows}) == 240
    return rows


# ---- runtime imports ----
sys.path.insert(0, str(PJ)); sys.path.insert(0, str(E5)); sys.path.insert(0, str(E3B))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def say(*a):
    print(*a, flush=True)


def save(s):
    t = MAIN / 'STATE.json.new'; t.write_text(json.dumps(s, indent=2, default=str) + '\n'); os.replace(t, MAIN / 'STATE.json')


def verify_freeze():
    fz = json.loads((PJ / 'J1_FREEZE_HASHES.json').read_text())
    bad = [f for f, h in fz['sha256'].items() if not (PJ / f).is_file() or sha(PJ / f) != h]
    if bad:
        raise SystemExit('J1_FROZEN_FILES_CHANGED ' + repr(bad))
    for P, name, pref in PINS:
        z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if not (P / f).is_file() or sha(P / f) != h]
        if bad or not sha(P / name).startswith(pref):
            raise SystemExit('PACKAGE_CHANGED ' + P.name + ' ' + repr(bad))
    return fz


def run_one(row, attempt, kind):
    import j1_metrics as M
    e5cfg = cfg_for(row['delta_s'])
    plan = plan_for(row['level'], row['duration_s'])
    spec = dict(trial_id=row['trial_id'], level=row['level'], arm=policy(row['profile']), rep=str(row['rep']), seed='',
                block=str(row['rep']), e5=e5cfg,
                j1={'config': row['config'], 'plan': plan, 'cell': row['cell'], 'duration_s': row['duration_s'],
                    'delta_s': row['delta_s']},
                scope_token=secrets.token_hex(24), actor_trace_sha256=sha(E5 / 'e5_crossing.py'))
    tag = f"{row['trial_id']}_a{attempt}_{now()}"; out = MAIN / 'trials' / tag; t0 = time.monotonic()
    p = subprocess.run(['/usr/bin/python3', '-u', str(PJ / 'j1_run_trial.py'), json.dumps(spec), str(out)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=2400)
    if out.is_dir():
        (out / 'RUN_TRIAL_CONSOLE.log').write_text(p.stdout)
    res = json.loads((out / 'TRIAL_RESULT.json').read_text()) if (out / 'TRIAL_RESULT.json').is_file() else \
        {'status': 'PRE_MOTION_BLOCKED', 'error': p.stdout[-800:]}
    res.update(wall_total_s=round(time.monotonic() - t0, 1), attempts=attempt, evidence_dir=str(out), j1_kind=kind,
               j1_config=row['config'], j1_cell=row['cell'], j1_level=row['level'], j1_profile=row['profile'],
               j1_duration_s=row['duration_s'], j1_rep=row['rep'], j1_delta_s=row['delta_s'], j1_plan=plan,
               e5_cfg=e5cfg, speed=SPEED, geometry='new')
    log = (out / 'c52_joint_bridge.log').read_text(errors='replace') if (out / 'c52_joint_bridge.log').is_file() else ''
    res['bridge_config_line'] = next((l for l in log.splitlines() if l.startswith('J1_BRIDGE config=')), None)
    try:
        res['e5'] = M.run_measures(out, res)
    except Exception as ex:
        res['e5'] = {'error': repr(ex)}
    return res


def started(r):
    return bool(r.get('bridge_config_line'))


LEDGER_NAME = {'B0': 'K0', 'B1': 'K1', 'B2': 'K2', 'B3': 'K3'}     # B0-B3 are the frozen E6 classes; their ledger header says K0-K3


def config_ok(r, cfg):
    if r.get('status') == 'PRE_MOTION_BLOCKED' and not started(r):
        return True
    m = r.get('e5') or {}
    return (r.get('bridge_config_line') or '').startswith(f'J1_BRIDGE config={cfg} ') and m.get('ledger_config') == LEDGER_NAME.get(cfg, cfg)


def plan_ok(r, level, dur):
    """Congestion at the trigger with the level's capacity; recovery exactly dur seconds later. C0: no congestion."""
    import e3b_policy_logic as L
    ev = [e.get('event', {}) for e in (r.get('network_events') or [])]
    trig = [e for e in ev if e.get('event') == 'trigger']
    ph = {e.get('new_phase'): e for e in ev if e.get('event') == 'network_phase'}
    if level == 'C0':
        return 'congested' not in ph
    if r.get('goal_sim_s') is None:
        return True                                   # never moved: nothing to check
    t = trig[0].get('trigger_sim_us') if trig else None
    c = ph.get('congested', {}); rc = ph.get('recovery', {})
    ok = bool(t) and len(trig) == 1 and c.get('boundary_sim_us') == t and c.get('new_bps') == int(round(L.LEVELS[level] * 1_000_000))
    if ok and rc:
        ok = rc.get('boundary_sim_us') == t + int(round(dur * 1e6))
    elif ok and r.get('terminal_sim_s') is not None and r['terminal_sim_s'] * 1e6 > t + dur * 1e6 + 1e6:
        ok = False                                    # ran past the planned recovery without a recovery boundary
    return ok


def makl_inputs_ok(r, cfg):
    if cfg != 'MAKL' or r.get('goal_sim_s') is None:
        return True
    m = r.get('e5') or {}
    return (m.get('j1_inputs_odom') or 0) > 0 and (m.get('j1_inputs_cmd') or 0) > 0 and (m.get('makl_decisions_all') or 0) > 0 \
        and (m.get('makl_decisions_v_pos_all') or 0) > 0


def hard_guard_death(r):
    return 'OWNED_PROCESS_DIED_final_hard_guard' in str(r.get('error') or '')


def with_retries(row, kind):
    for a in (1, 2, 3):
        r = run_one(row, a, kind); m = r.get('e5') if isinstance(r.get('e5'), dict) else {}
        say(f"   {row['trial_id']} attempt {a}: {r.get('status')} config_ok={config_ok(r, row['config'])} "
            f"plan_ok={plan_ok(r, row['level'], row['duration_s'])} contact={m.get('contact_class')} "
            f"viol={m.get('W_viol_anav_unom_b0.2_s')} anav95c={m.get('a_nav_p95_clipped_s')} "
            f"makl={m.get('makl_decisions_W')}/{m.get('makl_cam_share_W')} {str(r.get('error') or '')[:110]}")
        if r.get('status') != 'PRE_MOTION_BLOCKED' or hard_guard_death(r):
            return r
        time.sleep(5)
    return r


def gate_checks(r, level, dur):
    m = r.get('e5') or {}
    return {'run_succeeded': r.get('status') == 'SUCCEEDED',
            # v2 (amendment A1): same rule as for counted runs (plan_ok): congestion at the trigger with the cell's
            # capacity; a recovery boundary, if the run lasted past it, exactly dur s later. For the 8 s gate the recovery
            # must be observed (the run always lasts longer than 8 s after the trigger).
            'capacity_plan_at_trigger_and_recovery': plan_ok(r, level, dur) and (m.get('event_congested_bps') is not None)
            and (m.get('event_duration_measured_s') == float(dur) if dur <= 8 else m.get('event_duration_measured_s') in (None, float(dur))),
            'a_nav_logged': bool(r.get('a_nav_logged')),
            'arrival_within_0.15s': m.get('arrival_error_s') is not None and abs(m['arrival_error_s']) <= 0.15,
            'bridge_and_ledger_MAKL': config_ok(r, 'MAKL') and m.get('ledger_config') == 'MAKL' and started(r),
            'makl_inputs_received': makl_inputs_ok(r, 'MAKL') and r.get('goal_sim_s') is not None,
            'makl_decisions_W_ge_20_both_streams': (m.get('makl_decisions_W') or 0) >= 20 and (m.get('makl_scan_chosen_W') or 0) >= 1
            and (m.get('makl_cam_chosen_W') or 0) >= 1,
            'cpu_logged': m.get('makl_cpu_med_us') is not None and m.get('makl_cputhread_med_us') is not None}


def halt(st, why, **k):
    st['halted'] = dict(why=why, at=now(), **k); save(st); raise SystemExit('J1_HALT ' + why)


def finish(st):
    an = MAIN / 'final'; an.mkdir(exist_ok=True)
    p = subprocess.run(['/usr/bin/python3', str(PJ / 'j1_analysis.py'), str(MAIN / 'STATE.json'), str(an)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (an / 'ANALYSIS_CONSOLE.log').write_text(p.stdout); say(p.stdout[-4000:])
    z = Path.home() / 'Downloads' / f'SCI_J1_V2_RESULTS_{now()}.zip'
    keep = ('TRIAL_RESULT.json', 'TRIAL_SPEC.json', 'GRAPH_ASSERT.json', 'c52_joint_bridge.log', 'RUN_TRIAL_CONSOLE.log',
            'E2_CROSSING_GROUND_TRUTH.jsonl', 'controller_server.log', 'ERROR.txt', 'E5_AMCL_TOL.json', 'J1_LEDGER.jsonl',
            'J1_DECISIONS.jsonl')
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zz:
        zz.write(MAIN / 'STATE.json', 'STATE.json')
        for f in sorted(an.rglob('*')):
            if f.is_file():
                zz.write(f, 'final/' + f.name)
        dirs = [r['evidence_dir'] for r in st['done'].values()] + [h['evidence_dir'] for h in st.get('gate_history', []) if h.get('evidence_dir')]
        for d in dirs:
            d = Path(d)
            for f in sorted(d.glob('*')) if d.is_dir() else []:
                if f.name in keep or f.name.startswith('E3B_'):
                    zz.write(f, 'trials/' + d.name + '/' + f.name)
        for f in ('J1_PROTOCOL_FREEZE.md', 'J1_FREEZE_HASHES.json'):
            zz.write(PJ / f, 'SOURCE/' + f)
        if (V1_MAIN / 'STATE.json').is_file():                  # v1 (halted at the gate) is archived, not used
            zz.write(V1_MAIN / 'STATE.json', 'V1_HALTED/STATE.json')
            for d in sorted((V1_MAIN / 'trials').glob('*')):
                for f in sorted(d.glob('*')):
                    if f.name in keep or f.name.startswith('E3B_'):
                        zz.write(f, 'V1_HALTED/trials/' + d.name + '/' + f.name)
    say('J1_RESULTS_ZIP', z, sha(z))


GATES = (({'trial_id': 'GATE_J1_G1_LC-Q2-25s_MAKL', 'rep': 'G1', 'cell': 'LC-Q2-25s', 'level': 'LC', 'profile': 'Q2',
           'duration_s': 25, 'config': 'MAKL', 'delta_s': -0.4, 'block': 0}),
         ({'trial_id': 'GATE_J1_G2_LB-Q1-8s_MAKL', 'rep': 'G2', 'cell': 'LB-Q1-8s', 'level': 'LB', 'profile': 'Q1',
           'duration_s': 8, 'config': 'MAKL', 'delta_s': 0.4, 'block': 0}))


def main():
    if not (os.getenv('SCI_SIM_ONLY') == 'YES' and os.getenv('SCI_E2_NO_PHYSICAL_ROBOT') == 'YES'):
        raise SystemExit('J1_SIM_ONLY_ACKS_REQUIRED')
    os.environ['SCI_E3B_EDGE_NAV'] = 'YES'
    fz = verify_freeze(); MAIN.mkdir(parents=True, exist_ok=True); (MAIN / 'trials').mkdir(exist_ok=True)
    st = json.loads((MAIN / 'STATE.json').read_text()) if (MAIN / 'STATE.json').is_file() else \
        {'created': now(), 'freeze': fz, 'done': {}, 'halted': None}
    if st.get('halted'):
        raise SystemExit('J1_HALTED_REVIEW_REQUIRED ' + json.dumps(st['halted']))
    if 'locked' not in st:
        st['locked'] = {'at': now(), 'freeze_file_sha256': sha(PJ / 'J1_FREEZE_HASHES.json'), 'e5_calibration_used': E5_LOCK,
                        'delta_s': DELTA, 'cells': [cell_name(c) for c in CELLS], 'arms': ARMS,
                        'plans': {cell_name(c): plan_for(c[0], c[2]) for c in CELLS}}
        save(st); say('J1_LOCKED', json.dumps(st['locked']))
    g = st.setdefault('gate', {'runs': {}})
    for row in GATES:
        if row['trial_id'] in g['runs'] and g['runs'][row['trial_id']].get('passed'):
            continue
        if row['trial_id'] in g['runs']:
            halt(st, 'gate failed earlier', gate=row['trial_id'])
        say(f"=== GATE {row['trial_id']} (not counted, delta {row['delta_s']:+.1f} s) ==="); verify_freeze()
        r = with_retries(row, 'gate'); ch = gate_checks(r, row['level'], row['duration_s']); m = r.get('e5') or {}
        rec = {'checks': ch, 'passed': all(ch.values()), 'status': r.get('status'), 'evidence_dir': r.get('evidence_dir'),
               'makl': {k: m.get(k) for k in m if k.startswith('makl_') or k.startswith('j1_')},
               'event_duration_measured_s': m.get('event_duration_measured_s')}
        g['runs'][row['trial_id']] = rec; st.setdefault('gate_history', []).append(dict(rec, trial_id=row['trial_id'])); save(st)
        say('J1_GATE', row['trial_id'], json.dumps(rec, default=str))
        if not rec['passed']:
            halt(st, 'gate failed', gate=row['trial_id'])
    g['passed'] = True; save(st)
    for row in matrix():
        if row['trial_id'] in st['done']:
            continue
        verify_freeze(); say(f"=== {row['trial_id']} (block {row['block']}, delta {row['delta_s']:+.1f} s) ===")
        r = with_retries(row, 'counted')
        r.update(order=len(st['done']) + 1, block=row['block']); st['done'][row['trial_id']] = r; save(st)
        if not config_ok(r, row['config']):
            halt(st, 'bridge/ledger configuration mismatch', trial=row['trial_id'])
        if not plan_ok(r, row['level'], row['duration_s']):
            halt(st, 'capacity plan mismatch', trial=row['trial_id'])
        if not makl_inputs_ok(r, row['config']):
            halt(st, 'MAKL inputs missing', trial=row['trial_id'])
        if hard_guard_death(r):
            halt(st, 'hard-guard death', trial=row['trial_id'])
        rm = sum(1 for x in st['done'].values() if (x.get('e5') or {}).get('contact_class') == 'robot_moving')
        if rm >= 2:
            halt(st, '2 robot-moving contacts')
    st['completed'] = now(); save(st); finish(st)


if __name__ == '__main__':
    main()
