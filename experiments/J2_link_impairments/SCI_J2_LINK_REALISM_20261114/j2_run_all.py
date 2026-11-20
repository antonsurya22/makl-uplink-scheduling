#!/usr/bin/env python3
"""J2 batch (protocol J2_PROTOCOL_FREEZE.md). Simulation only. Resumable; finished runs are never repeated.
  0. Checks before every run: J2 freeze hashes; J1 v2, P0-J, E7 (article E6) and E5 packages unchanged.
  1. LOCK in STATE before any gate or counted run.
  2. GATE (2 runs, not counted, both must pass):
       G1 MAKL, Q2 at 0.13 Mbit/s (LC), Gilbert-Elliott loss, delta -0.4 s;
       G2 MAKL, Q1 at 0.18 Mbit/s (LB), square-wave capacity, delta +0.4 s.
     Checks: SUCCEEDED; capacity plan (plan_ok, nominal value); a_nav logged; arrival within 0.15 s; bridge + ledger =
     MAKL; MAKL inputs received; >= 20 MAKL decisions in W with both streams chosen; impairment line = condition and seed;
     impairment active (G1: lost packets > 0 and channel_loss in the ledger; G2: capacity changes >= 2).
  3. MATRIX: 6 arms (B2 B3 B4 B5 B7 MAKL) x 7 conditions (none jit10 jit50 loss1 loss5 ge capsq) x 2 cells (LC-Q2,
     LB-Q1; 25 s events) x 3 paired start shifts (delta -0.4, 0, +0.4 s) = 252 counted runs, pedestrian 0.50 m/s.
     Block = one delta x cell x condition with all 6 arms (same impairment seed); arm order rotated per block.
Rules as J1. Halts: 2 robot-moving contacts; hard-guard death; bridge/ledger configuration mismatch; capacity plan
mismatch; MAKL inputs missing in a MAKL run that moved; impairment line missing or not matching the condition/seed."""
import datetime, hashlib, json, os, secrets, subprocess, sys, time, zipfile
from pathlib import Path
PJ = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
E5 = WS / 'SCI_E5_PED_SPEED_20261101'
E7 = WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102'
P0J = WS / 'SCI_P0J_SCHEDULERS_20261111'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
BASE = WS / 'results/SCI_J2_LINK_REALISM_20261114'
MAIN = BASE / 'J2_MAIN'
ARMS = ('B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
CONDS = ('none', 'jit10', 'jit50', 'loss1', 'loss5', 'ge', 'capsq')
CELLS = (('LC', 'Q2'), ('LB', 'Q1'))
DUR = 25
DELTA = {1: -0.4, 2: 0.0, 3: 0.4}
V_NOM = 0.15
SPEED = 0.50
NEW = dict(x=3.2, y0=-1.5, y1=1.40)
E5_LOCK = {'0.5': {'x_meet': 2.4}}
PINS = ((E5, 'E5_FREEZE_HASHES.json', '358daffd'), (E7, 'E7_FREEZE_HASHES.json', 'a0cc9ef7'),
        (P0J, 'P0J_FREEZE_HASHES.json', 'e5bccfc2'), (J1, 'J1_FREEZE_HASHES.json', 'befd9065'))
LEDGER_NAME = {'B2': 'K2', 'B3': 'K3'}
sys.path.insert(0, str(PJ)); sys.path.insert(0, str(E5)); sys.path.insert(0, str(E3B))


def cell_name(lv, prof, cond):
    return f'{lv}-{prof}|{cond}'


def policy(prof):
    return 'F-N1' if prof == 'Q1' else 'F-N2'


def cfg_for(delta):
    return dict(NEW, speed=SPEED, start_mode='meet', x_meet=round(E5_LOCK['0.5']['x_meet'] + V_NOM * delta, 4))


def plan_for(level):
    import e3b_policy_logic as L3
    p = dict(L3.plan(level)); p['recover_after_trigger_s'] = float(DUR)
    return p


def matrix():
    import j2_impair as I
    rows = []; b = 0
    for rep in (1, 2, 3):
        for lv, prof in CELLS:
            for cond in CONDS:
                k = list(ARMS); r = b % len(ARMS); k = k[r:] + k[:r]; b += 1
                for a in k:
                    rows.append({'trial_id': f'J2_R{rep}_{lv}-{prof}_{cond}_{a}', 'rep': rep, 'cell': cell_name(lv, prof, cond),
                                 'level': lv, 'profile': prof, 'condition': cond, 'config': a, 'delta_s': DELTA[rep], 'block': b,
                                 'seed': I.seed_for(cond, rep, prof)})
    assert len(rows) == 252 and len({r['trial_id'] for r in rows}) == 252
    return rows


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def say(*a):
    print(*a, flush=True)


def save(s):
    t = MAIN / 'STATE.json.new'; t.write_text(json.dumps(s, indent=2, default=str) + '\n'); os.replace(t, MAIN / 'STATE.json')


def verify_freeze():
    fz = json.loads((PJ / 'J2_FREEZE_HASHES.json').read_text())
    bad = [f for f, h in fz['sha256'].items() if not (PJ / f).is_file() or sha(PJ / f) != h]
    if bad:
        raise SystemExit('J2_FROZEN_FILES_CHANGED ' + repr(bad))
    for P, name, pref in PINS:
        z = json.loads((P / name).read_text()); bad = [f for f, h in z['sha256'].items() if not (P / f).is_file() or sha(P / f) != h]
        if bad or not sha(P / name).startswith(pref):
            raise SystemExit('PACKAGE_CHANGED ' + P.name + ' ' + repr(bad))
    return fz


def run_one(row, attempt, kind):
    import j2_metrics as M
    import j2_impair as I
    e5cfg = cfg_for(row['delta_s'])
    plan = plan_for(row['level'])
    spec = dict(trial_id=row['trial_id'], level=row['level'], arm=policy(row['profile']), rep=str(row['rep']), seed='',
                block=str(row['rep']), e5=e5cfg,
                j1={'config': row['config'], 'plan': plan, 'cell': row['cell'], 'duration_s': DUR, 'delta_s': row['delta_s']},
                j2={'condition': row['condition'], 'impair': I.CONDITIONS[row['condition']], 'seed': row['seed']},
                scope_token=secrets.token_hex(24), actor_trace_sha256=sha(E5 / 'e5_crossing.py'))
    tag = f"{row['trial_id']}_a{attempt}_{now()}"; out = MAIN / 'trials' / tag; t0 = time.monotonic()
    p = subprocess.run(['/usr/bin/python3', '-u', str(PJ / 'j2_run_trial.py'), json.dumps(spec), str(out)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=2400)
    if out.is_dir():
        (out / 'RUN_TRIAL_CONSOLE.log').write_text(p.stdout)
    res = json.loads((out / 'TRIAL_RESULT.json').read_text()) if (out / 'TRIAL_RESULT.json').is_file() else \
        {'status': 'PRE_MOTION_BLOCKED', 'error': p.stdout[-800:]}
    res.update(wall_total_s=round(time.monotonic() - t0, 1), attempts=attempt, evidence_dir=str(out), j1_kind=kind,
               j1_config=row['config'], j1_cell=row['cell'], j1_level=row['level'], j1_profile=row['profile'],
               j1_duration_s=DUR, j1_rep=row['rep'], j1_delta_s=row['delta_s'], j1_plan=plan, j2_condition=row['condition'],
               j2_seed=row['seed'], e5_cfg=e5cfg, speed=SPEED, geometry='new')
    log = (out / 'c52_joint_bridge.log').read_text(errors='replace') if (out / 'c52_joint_bridge.log').is_file() else ''
    res['bridge_config_line'] = next((l for l in log.splitlines() if l.startswith('J1_BRIDGE config=')), None)
    try:
        res['e5'] = M.run_measures(out, res)
    except Exception as ex:
        res['e5'] = {'error': repr(ex)}
    return res


def started(r):
    return bool(r.get('bridge_config_line'))


def config_ok(r, cfg):
    if r.get('status') == 'PRE_MOTION_BLOCKED' and not started(r):
        return True
    m = r.get('e5') or {}
    return (r.get('bridge_config_line') or '').startswith(f'J1_BRIDGE config={cfg} ') and m.get('ledger_config') == LEDGER_NAME.get(cfg, cfg)


def impair_ok(r, cond, seed):
    if r.get('status') == 'PRE_MOTION_BLOCKED' and not started(r):
        return True
    m = r.get('e5') or {}
    return m.get('j2_condition_bridge') == cond and m.get('j2_seed_bridge') == seed


def plan_ok(r, level, dur=DUR):
    import e3b_policy_logic as L
    ev = [e.get('event', {}) for e in (r.get('network_events') or [])]
    trig = [e for e in ev if e.get('event') == 'trigger']
    ph = {e.get('new_phase'): e for e in ev if e.get('event') == 'network_phase'}
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
            f"plan_ok={plan_ok(r, row['level'])} impair_ok={impair_ok(r, row['condition'], row['seed'])} "
            f"contact={m.get('contact_class')} viol={m.get('W_viol_anav_unom_b0.2_s')} anav95c={m.get('a_nav_p95_clipped_s')} "
            f"loss={m.get('j2_loss_rate_packets')} capch={m.get('j2_cap_changes')} chl={m.get('L_all_channel_loss')} "
            f"makl={m.get('makl_decisions_W')}/{m.get('makl_cam_share_W')} {str(r.get('error') or '')[:100]}")
        if r.get('status') != 'PRE_MOTION_BLOCKED' or hard_guard_death(r):
            return r
        time.sleep(5)
    return r


def gate_checks(r, row):
    m = r.get('e5') or {}
    ch = {'run_succeeded': r.get('status') == 'SUCCEEDED',
          'capacity_plan_nominal': plan_ok(r, row['level']) and m.get('event_congested_bps') is not None,
          'a_nav_logged': bool(r.get('a_nav_logged')),
          'arrival_within_0.15s': m.get('arrival_error_s') is not None and abs(m['arrival_error_s']) <= 0.15,
          'bridge_and_ledger_MAKL': config_ok(r, 'MAKL') and started(r),
          'makl_inputs_received': makl_inputs_ok(r, 'MAKL') and r.get('goal_sim_s') is not None,
          'makl_decisions_W_ge_20_both_streams': (m.get('makl_decisions_W') or 0) >= 20 and (m.get('makl_scan_chosen_W') or 0) >= 1
          and (m.get('makl_cam_chosen_W') or 0) >= 1,
          'impairment_line_matches': impair_ok(r, row['condition'], row['seed']) and started(r)}
    if row['condition'] == 'ge':
        ch['impairment_active'] = (m.get('j2_lost_packets') or 0) > 0 and (m.get('L_all_channel_loss') or 0) > 0
    elif row['condition'] == 'capsq':
        ch['impairment_active'] = (m.get('j2_cap_changes') or 0) >= 2
    return ch


def halt(st, why, **k):
    st['halted'] = dict(why=why, at=now(), **k); save(st); raise SystemExit('J2_HALT ' + why)


def finish(st):
    an = MAIN / 'final'; an.mkdir(exist_ok=True)
    p = subprocess.run(['/usr/bin/python3', str(PJ / 'j2_analysis.py'), str(MAIN / 'STATE.json'), str(an)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (an / 'ANALYSIS_CONSOLE.log').write_text(p.stdout); say(p.stdout[-4000:])
    z = Path.home() / 'Downloads' / f'SCI_J2_RESULTS_{now()}.zip'
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
        for f in ('J2_PROTOCOL_FREEZE.md', 'J2_FREEZE_HASHES.json'):
            zz.write(PJ / f, 'SOURCE/' + f)
    say('J2_RESULTS_ZIP', z, sha(z))


def gates():
    import j2_impair as I
    return ({'trial_id': 'GATE_J2_G1_LC-Q2_ge_MAKL', 'rep': 'G1', 'cell': 'LC-Q2|ge', 'level': 'LC', 'profile': 'Q2',
             'condition': 'ge', 'config': 'MAKL', 'delta_s': -0.4, 'block': 0, 'seed': I.seed_for('ge', 'G1', 'Q2')},
            {'trial_id': 'GATE_J2_G2_LB-Q1_capsq_MAKL', 'rep': 'G2', 'cell': 'LB-Q1|capsq', 'level': 'LB', 'profile': 'Q1',
             'condition': 'capsq', 'config': 'MAKL', 'delta_s': 0.4, 'block': 0, 'seed': I.seed_for('capsq', 'G2', 'Q1')})


def main():
    if not (os.getenv('SCI_SIM_ONLY') == 'YES' and os.getenv('SCI_E2_NO_PHYSICAL_ROBOT') == 'YES'):
        raise SystemExit('J2_SIM_ONLY_ACKS_REQUIRED')
    os.environ['SCI_E3B_EDGE_NAV'] = 'YES'
    fz = verify_freeze(); MAIN.mkdir(parents=True, exist_ok=True); (MAIN / 'trials').mkdir(exist_ok=True)
    st = json.loads((MAIN / 'STATE.json').read_text()) if (MAIN / 'STATE.json').is_file() else \
        {'created': now(), 'freeze': fz, 'done': {}, 'halted': None}
    if st.get('halted'):
        raise SystemExit('J2_HALTED_REVIEW_REQUIRED ' + json.dumps(st['halted']))
    if 'locked' not in st:
        import j2_impair as I
        st['locked'] = {'at': now(), 'freeze_file_sha256': sha(PJ / 'J2_FREEZE_HASHES.json'), 'e5_calibration_used': E5_LOCK,
                        'delta_s': DELTA, 'cells': [f'{a}-{b}' for a, b in CELLS], 'conditions': {c: I.CONDITIONS[c] for c in CONDS},
                        'arms': ARMS, 'plans': {lv: plan_for(lv) for lv, _ in CELLS}}
        save(st); say('J2_LOCKED', json.dumps(st['locked']))
    g = st.setdefault('gate', {'runs': {}})
    for row in gates():
        if row['trial_id'] in g['runs'] and g['runs'][row['trial_id']].get('passed'):
            continue
        if row['trial_id'] in g['runs']:
            halt(st, 'gate failed earlier', gate=row['trial_id'])
        say(f"=== GATE {row['trial_id']} (not counted, delta {row['delta_s']:+.1f} s) ==="); verify_freeze()
        r = with_retries(row, 'gate'); ch = gate_checks(r, row); m = r.get('e5') or {}
        rec = {'checks': ch, 'passed': all(ch.values()), 'status': r.get('status'), 'evidence_dir': r.get('evidence_dir'),
               'j2': {k: m.get(k) for k in m if k.startswith('j2_') or k.startswith('L_all')},
               'makl': {k: m.get(k) for k in m if k.startswith('makl_')}}
        g['runs'][row['trial_id']] = rec; st.setdefault('gate_history', []).append(dict(rec, trial_id=row['trial_id'])); save(st)
        say('J2_GATE', row['trial_id'], json.dumps(rec, default=str))
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
        if not impair_ok(r, row['condition'], row['seed']):
            halt(st, 'impairment mismatch', trial=row['trial_id'])
        if not plan_ok(r, row['level']):
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
