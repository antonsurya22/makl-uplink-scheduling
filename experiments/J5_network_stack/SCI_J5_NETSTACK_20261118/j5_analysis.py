#!/usr/bin/env python3
"""J5 pre-specified analysis (J5_PROTOCOL_FREEZE.md sections 5 and 8). Usage: j5_analysis.py RESULTS_DIR OUT_DIR
RESULTS_DIR holds edge/<rid>.jsonl (edge receive log), sender/<rid>/* (files uploaded by the sender) and
STATE_SENDER.json. Only counted replays of valid blocks enter the tests; pairs are the traces (rep 1-5).
Times: trace microseconds; a wall time w (ns) maps to t0 + (w - wall0)/1000 with the sender's wall0 of that replay.
  peak age at delivery k (k >= 1) = t_k - (newest capture among deliveries before k); W = [T, T + 25 s)
  a_nav uses the edge receive time; a_link uses the end of transmission in the link model (sender)
  F = camera frames received in W within 1.0 s of capture / 25 s
  violation = frozen J0 open-loop violation (j_replay.violation) with the edge deliveries and the C0 trajectory
Outputs: J5_REPLAYS.csv, J5_CELLS.csv, J5_TESTS.csv, J5_VERDICTS.json, J5_PLATFORM.json."""
import csv, json, math, statistics as st, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5_common as C

W_US = int(C.DUR_S * 1e6)


def pct(xs, q):
    xs = sorted(x for x in xs if x is not None and math.isfinite(x))
    if not xs:
        return None
    k = (len(xs) - 1) * q; f = math.floor(k); c = math.ceil(k)
    return xs[f] if f == c else xs[f] + (xs[c] - xs[f]) * (k - f)


def jl(p):
    p = Path(p)
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.is_file() else []


def peaks(deliv, T):
    """deliv: [(t_deliver_us, capture_us)] -> peak ages (s) at deliveries inside W."""
    out = []; newest = None
    for t, cap in sorted(deliv):
        if newest is not None and T <= t < T + W_US:
            out.append((t - newest) / 1e6)
        newest = cap if newest is None else max(newest, cap)
    return out


def replay_measures(base, rid):
    S = base / 'sender' / rid
    meta = json.loads((S / 'meta.json').read_text())
    t0, T, w0 = meta['t0_us'], meta['T_us'], meta['wall0_ns']
    to_us = lambda ns: t0 + (ns - w0) // 1000
    edge = jl(base / 'edge' / f'{rid}.jsonl'); sent = jl(S / 'sent.jsonl')
    tr = C.load_trace(meta['cell'], meta['rep'])
    sc = [(to_us(r['recv_ns']), to_us(r['cap_ns'])) for r in edge if r['stream'] == 'scan']
    an = peaks(sc, T)
    al = peaks([(r['tx_us'], r['cap_us']) for r in sent if r['stream'] == 'scan'], T)
    cam = [(to_us(r['recv_ns']), to_us(r['cap_ns'])) for r in edge if r['stream'] == 'camera']
    F = sum(1 for t, c in cam if T <= t < T + W_US and t - c <= 1_000_000) / C.DUR_S
    R = {'recv': [('scan', t, c, None, None) for t, c in sc], 'T': T}
    v = C.JR.violation(R, tr['grid'])
    ow = [(r['recv_ns'] - r['pub_ns']) / 1e6 for r in edge]
    txmap = {r['mid']: r for r in sent}
    nav_link = [(to_us(r['recv_ns']) - txmap[r['mid']]['tx_us']) / 1e3 for r in edge if r['mid'] in txmap]
    late = [(r['pub_us'] - r['tx_us']) / 1e3 for r in sent]
    eps_ce = [(r['off_us'] - r['cap_us']) / 1e3 for r in sent if r['stream'] == 'scan']
    tc = jl(S / 'tc.jsonl')
    wW0 = w0 + (T - t0) * 1000; wW1 = wW0 + W_US * 1000
    bl = [x.get('backlog_bytes', 0) for x in tc if wW0 <= x['wall_ns'] < wW1]
    m = {'rid': rid, 'rep': meta['rep'], 'cell': meta['cell'], 'mode': meta['mode'], 'arm': meta['arm'], 'tag': meta.get('tag', ''),
         'valid': meta['valid'], 'n_edge': len(edge), 'n_sent': len(sent),
         'anav_p95_s': pct(an, .95), 'anav_max_s': max(an) if an else None, 'alink_p95_s': pct(al, .95),
         'alink_max_s': max(al) if al else None, 'F_fps': round(F, 4), 'viol_s': v['viol_s'],
         'ow_p50_ms': pct(ow, .5), 'ow_p99_ms': pct(ow, .99), 'nav_minus_link_p50_ms': pct(nav_link, .5),
         'nav_minus_link_p99_ms': pct(nav_link, .99), 'pub_late_p50_ms': pct(late, .5), 'pub_late_p99_ms': pct(late, .99),
         'eps_ce_p50_ms': pct(eps_ce, .5), 'backlog_share_gt_3000B': (sum(b > 3000 for b in bl) / len(bl)) if bl else None,
         'backlog_med_B': pct(bl, .5), 'backlog_max_B': max(bl) if bl else None, 'tc_samples_W': len(bl)}
    if meta['arm'] == 'MAKL':
        d = jl(S / 'DECISIONS.jsonl')
        thr = [x['cputhread_ns'] / 1e6 for x in d if 'cputhread_ns' in x]
        m.update({'makl_dec_thread_p50_ms': pct(thr, .5), 'makl_dec_thread_p99_ms': pct(thr, .99),
                  'makl_dec_thread_max_ms': max(thr) if thr else None})
    return m


def clearly_greater(a, b, j):
    """J1 rule: >= 4 of 5 pairs, every paired difference a - b > 0 and the mean > j."""
    reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
    ev = len(reps) >= 4
    return {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d], 'mean_diff': round(st.mean(d), 4) if d else None,
            'J': j, 'evaluable': ev, 'clearly_greater': bool(ev and all(x > 0 for x in d) and st.mean(d) > j)}


def main():
    base = Path(sys.argv[1]); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    S = json.loads((base / 'STATE_SENDER.json').read_text())
    counted = []
    for b, B in sorted(S.get('blocks', {}).items(), key=lambda kv: int(kv[0])):
        if B.get('status') != 'valid':
            continue
        att = [a for a in B['attempts'] if a.get('valid')][-1]
        counted += att['replays']
    R = [replay_measures(base, rid) for rid in counted]
    keys = list(R[0].keys()) if R else []
    for r in R:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(out / 'J5_REPLAYS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(R)
    by = lambda cell, mode, arm, k: {r['rep']: r[k] for r in R if r['cell'] == cell and r['mode'] == mode and r['arm'] == arm
                                     and isinstance(r.get(k), (int, float)) and math.isfinite(r[k])}
    cells = []
    for cell in C.CELLS:
        for mode in C.MODES:
            for arm in C.ARMS:
                rr = [r for r in R if r['cell'] == cell and r['mode'] == mode and r['arm'] == arm]
                if not rr:
                    continue
                x = {'cell': cell, 'mode': mode, 'arm': arm, 'n': len(rr)}
                for k in keys[7:]:
                    z = [r[k] for r in rr if isinstance(r.get(k), (int, float)) and not isinstance(r.get(k), bool)
                         and math.isfinite(r[k])]
                    if z:
                        x[k] = round(st.mean(z), 4); x[k + '_sd'] = round(st.stdev(z), 4) if len(z) > 1 else None
                cells.append(x)
    ck = []
    for x in cells:
        for k in x:
            if k not in ck:
                ck.append(k)
    with open(out / 'J5_CELLS.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=ck); w.writeheader(); w.writerows(cells)
    T = {}; V = {'n_counted': len(R), 'blocks': {b: B.get('status') for b, B in S.get('blocks', {}).items()}}
    Jage = {}
    for cell in C.CELLS:
        z = list(by(cell, 'A', 'B7', 'anav_p95_s').values())
        Jage[cell] = round(max(0.1, (max(z) - min(z)) if len(z) >= 2 else 0.0), 4)
    V['J_age'] = Jage
    t1 = t2 = n1 = p1 = p2 = True
    for cell, cfg in C.CELLS.items():
        for x in ('B3', 'B7', 'MAKL'):
            t = T[f'T1_{cell}_A_B0_gt_{x}_anav'] = clearly_greater(by(cell, 'A', 'B0', 'anav_p95_s'), by(cell, 'A', x, 'anav_p95_s'), Jage[cell])
            t1 &= t['clearly_greater']
        if cell == 'Q1':
            t = T['T1_Q1_A_MAKL_gt_B7_F'] = clearly_greater(by(cell, 'A', 'MAKL', 'F_fps'), by(cell, 'A', 'B7', 'F_fps'), cfg['J_F'])
            t1 &= t['clearly_greater']
        t = T[f'T2_{cell}_A_MAKL_gt_B7_viol'] = clearly_greater(by(cell, 'A', 'MAKL', 'viol_s'), by(cell, 'A', 'B7', 'viol_s'), cfg['J_viol'])
        t2 &= t['evaluable'] and not t['clearly_greater']
        t = T[f'N1_{cell}_N_B0_gt_MAKL_anav'] = clearly_greater(by(cell, 'N', 'B0', 'anav_p95_s'), by(cell, 'N', 'MAKL', 'anav_p95_s'), Jage[cell])
        n1 &= t['evaluable'] and not t['clearly_greater']
        t = T[f'P1_{cell}_MAKL_N_gt_NP_anav'] = clearly_greater(by(cell, 'N', 'MAKL', 'anav_p95_s'), by(cell, 'NP', 'MAKL', 'anav_p95_s'), Jage[cell])
        p1 &= t['clearly_greater']
        a, b = by(cell, 'NP', 'MAKL', 'anav_p95_s'), by(cell, 'A', 'MAKL', 'anav_p95_s')
        reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
        T[f'P2_{cell}_MAKL_NP_minus_A_anav'] = {'n_pairs': len(reps), 'diffs': [round(x, 4) for x in d],
                                                'mean_diff': round(st.mean(d), 4) if d else None, 'J': Jage[cell],
                                                'predicted_s': round(8 * (3000 + (3170 if cell == 'Q1' else 2819)) / cfg['c_bps'] * (1 / C.PACE - 1), 4)}
        p2 &= len(reps) >= 4 and st.mean(d) <= Jage[cell]
        for arm in C.ARMS:
            a, b = by(cell, 'NP', arm, 'F_fps'), by(cell, 'A', arm, 'F_fps')
            reps = sorted(set(a) & set(b)); d = [a[k] - b[k] for k in reps]
            T[f'P3_{cell}_{arm}_F_NP_minus_A'] = {'n_pairs': len(reps), 'mean_diff': round(st.mean(d), 4) if d else None,
                                                  'A_mean': round(st.mean(b.values()), 4) if b else None}
    n2 = True
    for cell in C.CELLS:
        for arm in C.ARMS:
            z = [r['backlog_share_gt_3000B'] for r in R if r['cell'] == cell and r['mode'] == 'N' and r['arm'] == arm
                 and r.get('backlog_share_gt_3000B') is not None]
            T[f'N2_{cell}_{arm}_backlog_share'] = {'mean': round(st.mean(z), 4) if z else None, 'n': len(z)}
            n2 &= bool(z) and st.mean(z) > 0.5
    V.update({'T1': t1, 'T2': t2, 'N1_demonstration': n1, 'N2': n2, 'P1': p1, 'P2': p2,
              'TRANSFERS': t1 and t2, 'pacing': 'CONTROL_NEEDS_PACING' if p1 else 'CONTROL_NOT_RECOVERED',
              'unexpected_N1_failed': not n1})
    allr = [r for r in R]
    plat = {k: pct([r[k] for r in allr if r.get(k) is not None], q) for k, q in
            (('ow_p50_ms', .5), ('ow_p99_ms', .5), ('nav_minus_link_p50_ms', .5), ('nav_minus_link_p99_ms', .5),
             ('pub_late_p99_ms', .5), ('eps_ce_p50_ms', .5))}
    mk = [r for r in allr if r['arm'] == 'MAKL']
    plat['makl_decision_thread_p50_ms'] = pct([r.get('makl_dec_thread_p50_ms') for r in mk], .5)
    plat['makl_decision_thread_p99_ms_max_over_replays'] = max((r.get('makl_dec_thread_p99_ms') or 0) for r in mk) if mk else None
    plat['note'] = 'medians over replays of the per-replay statistics; ow = publish -> edge callback (includes the clock offset, <= 2 ms)'
    (out / 'J5_PLATFORM.json').write_text(json.dumps(plat, indent=1) + '\n')
    (out / 'J5_VERDICTS.json').write_text(json.dumps(V, indent=1) + '\n')
    with open(out / 'J5_TESTS.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['test', 'result_json'])
        for k, v in T.items():
            w.writerow([k, json.dumps(v)])
    if S.get('pilots'):
        (out / 'J5_PILOTS.json').write_text(json.dumps({k: v for k, v in S['pilots'].items() if k in ('checks', 'decision_time', 'iperf', 'passed')},
                                                       indent=1, default=str) + '\n')
    print('J5_VERDICTS', json.dumps({k: v for k, v in V.items() if k != 'blocks'}))
    print('J5_PLATFORM', json.dumps(plat))


if __name__ == '__main__':
    main()
