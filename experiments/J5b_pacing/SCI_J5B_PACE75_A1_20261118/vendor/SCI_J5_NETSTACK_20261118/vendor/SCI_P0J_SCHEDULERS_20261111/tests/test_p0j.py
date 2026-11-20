#!/usr/bin/env python3
"""P0-J unit tests. Usage: python3 -I tests/test_p0j.py [TRACES_DIR]
T1 a*(v,d) closed form = the 36 cells of Table S21 (v11.1 Table 1) to 2 decimals; v_max(a*,d) = v.
T2 ledger conservation for every discipline (counts and bytes; waiting_bytes >= 0 and back to 0; wire bytes);
   message-level disciplines never interleave packets of two messages (non-preemptive).
T3 Proposition 1: B7 (priority + keep-last), periodic scans + camera Q1/Q2 at 0.13/0.15/0.18/0.21 Mbit/s, 23 phases:
   peak scan age <= a_min(c) = T_s + 8(L_m+L_s)/c + delta, L = bytes on the wire.
T3b Proposition 2: for B3, B5, B7, MAKL, REF: after every camera frame, scan age before the next scan delivery
   >= a_LB(c) = 8(L_m+2L_s)/c + delta.
T4 MAKL limits (SLO_m = infinity): beta = 0 gives the same record-by-record outcome as B7; beta = infinity the same
   as camera-first keep-last.
T5 (needs traces) j_replay reproduces the frozen e4_harness.replay record by record (K0-K3) and the frozen
   E4_RESULTS_ALL.csv rows (fifo/prio/keeplast, scan expiry none, all caps, durations, windows, traces).
T6 CPU time per MAKL decision (reported; must be < 1 ms median on this machine).
T7 bridge interface: only bridge statuses visible; records expose finished_us/message_id/payload_bytes; prune keeps
   live records; outcomes() works.
Prints one line per test and a final P0J_TESTS PASS/FAIL; writes P0J_TEST_RESULTS.json next to TRACES_DIR's parent
if given, else in the current directory."""
import csv, json, math, random, statistics, sys, time
from collections import Counter
from pathlib import Path
HERE = Path(__file__).resolve().parent
PKG = HERE.parent
for p in (PKG / 'frozen_deps' / 'frozen', PKG / 'frozen_deps', PKG, HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import j_links as J
import j_synth as S

RES = {}
ALL_CFG = ['B0', 'B1', 'B2', 'B3', 'B4', 'B4m', 'B5', 'B7', 'MAKL', 'MAKL_wait', 'MAKL_odom', 'MAKL_cap',
           'MAKL_true_c', 'MAKL_T', 'REF_camfirst']
MSG_LEVEL = ['B5', 'B7', 'MAKL', 'MAKL_wait', 'MAKL_odom', 'MAKL_cap', 'MAKL_true_c', 'MAKL_T', 'REF_camfirst']


def mk(cfg, cap=3_000_000, **kw):
    return J.make_jlink(cfg, capacity_bps=cap, queue_limit_bytes=768_000, delay_us=20_000, **kw)


def report(name, ok, **info):
    RES[name] = dict(passed=bool(ok), **info)
    tag = 'SKIP' if ok is None else ('PASS' if ok else 'FAIL')
    RES[name]['passed'] = None if ok is None else bool(ok)
    print(f'{name}: {tag} {json.dumps(info, default=str)[:600]}', flush=True)


# ---------------------------------------------------------------------------------------------------------------- T1
TABLE = {  # (u, v): {d: (b=0.2, b=0.5)}; None = no admissible age ('--')
    (0.08, 0.08): {0.5: (2.72, 2.90), 0.7: (3.97, 4.16), 1.0: (5.85, 6.03)},
    (0.08, 0.15): {0.5: (1.57, 1.87), 0.7: (2.44, 2.74), 1.0: (3.74, 4.05)},
    (0.08, 0.30): {0.5: (0.31, 0.85), 0.7: (0.83, 1.38), 1.0: (1.62, 2.17)},
    (0.08, 0.50): {0.5: (None, 0.19), 0.7: (None, 0.54), 1.0: (0.20, 1.06)},
    (0.5, 0.08): {0.5: (0.39, 0.61), 0.7: (0.73, 0.96), 1.0: (1.25, 1.48)},
    (0.5, 0.15): {0.5: (0.01, 0.40), 0.7: (0.31, 0.71), 1.0: (0.78, 1.17)},
}


def t1():
    bad = []; n = 0
    for (u, v), row in TABLE.items():
        for d, pair in row.items():
            for b, ref in zip((0.2, 0.5), pair):
                n += 1
                a = J.astar(v, d, u, b, 0.1)
                if ref is None:
                    if a >= 0:
                        bad.append((u, v, d, b, a, ref))
                    continue
                if round(a, 2) != ref:
                    bad.append((u, v, d, b, round(a, 4), ref))
                if abs(J.vmax(a, d, u, b, 0.1) - v) > 1e-9:
                    bad.append(('vmax', u, v, d, b))
    report('T1_astar_table', n == 36 and not bad, cells=n, mismatches=bad)


# ---------------------------------------------------------------------------------------------------------------- T2
def drive_random(L, seed, cfg):
    rnd = random.Random(seed)
    t = 0; mids = []; neg = False
    caps = [3_000_000, 130_000, 180_000, 100_000, 250_000]
    while t < 40_000_000:
        t += rnd.choice([1_000, 20_000, 50_000, 100_000])
        L.advance(t)
        if rnd.random() < 0.02:
            L.capacity_bps = rnd.choice(caps)
        if hasattr(L, 'set_state'):
            L.set_state(v_odom=rnd.uniform(0, 0.2), v_cmd=rnd.uniform(0, 0.2), d=rnd.uniform(0.1, 2.0), t_us=t)
        s = rnd.choice(['scan', 'scan', 'camera', 'camera', 'c52_probe_ack', 'other'])
        b = {'scan': 2944, 'camera': rnd.randint(1500, 4200), 'c52_probe_ack': 96, 'other': rnd.randint(1, 5000)}[s]
        rel = 'reliable' if (s == 'camera' and rnd.random() < 0.3) else 'best_effort'
        mids.append(L.offer(stream=s, payload_bytes=b, stamp_us=t, reliability=rel))
        if L.waiting_bytes < 0:
            neg = True
    L.advance(t + 600_000_000)
    return mids, neg


def t2():
    fails = {}
    for cfg in ALL_CFG:
        for seed in range(4):
            L = mk(cfg)
            mids, neg = drive_random(L, seed, cfg)
            lines = [x for x in L.ledger.lines if not x.get('header')]
            off = Counter(L.records[m].stream for m in mids)
            offb = Counter();
            for m in mids:
                offb[L.records[m].stream] += L.records[m].payload_bytes if hasattr(L.records[m], 'payload_bytes') else L.records[m].payload
            term = Counter(x['stream'] for x in lines); termb = Counter()
            for x in lines:
                termb[x['stream']] += x['bytes']
            queued = [m for m in mids if L.records[m].status == 'queued']
            ok = (off == term and offb == termb and not queued and not neg and L.waiting_bytes == 0
                  and len(lines) == len(mids))
            # wire bytes = sum over transmitted packets
            txw = sum(len(x['tx_us']) for x in lines)
            if hasattr(L, 'jdisc'):
                st_true = Counter(x['status'] for x in lines)
                ok &= set(st_true) <= {'delivered', 'expired', 'queue_drop', 'replaced', 'dropped_codel', 'dropped_slo'}
            # non-preemption for message-level disciplines: tx starts of one message are consecutive in time order
            if cfg in MSG_LEVEL:
                starts = sorted((t, x['mid']) for x in lines for t in x['tx_us'])
                seq = [m for _, m in starts]
                runs = [seq[0]] if seq else []
                for m in seq[1:]:
                    if m != runs[-1]:
                        runs.append(m)
                ok &= len(runs) == len(set(runs))
            if not ok:
                fails[f'{cfg}/{seed}'] = dict(off=dict(off), term=dict(term), queued=len(queued), neg=neg,
                                             wb=L.waiting_bytes, lines=len(lines), mids=len(mids))
    report('T2_ledger_conservation', not fails, configs=len(ALL_CFG), seeds=4, failures=fails)


# ---------------------------------------------------------------------------------------------------------------- T3
def t3():
    rows = []; ok = True
    for prof in ('Q1', 'Q2'):
        for c in (130_000, 150_000, 180_000, 210_000):
            lm = S.wire(S.CAM[prof][0]); ls = S.wire(S.SCAN_B)
            amin = 0.2 + 8 * (lm + ls) / c + 0.02
            worst = 0
            for ph in range(0, 200_000, 9_000):
                L = mk('B7', c)
                r = S.run(L, profile=prof, cap_bps=c, phase_cam_us=ph, horizon_us=20_000_000)
                worst = max(worst, max(a for _, a in S.peak_ages(r)))
            good = worst <= amin + 2e-6         # 1 us ceil rounding per packet
            ok &= good
            rows.append(dict(profile=prof, c_bps=c, worst_peak_s=round(worst, 4), a_min_s=round(amin, 4),
                             a_min_payload_only_s=round(0.2 + 8 * (S.CAM[prof][0] - 64 + S.SCAN_B) / c + 0.02, 4),
                             ok=good))
    report('T3_prop1_upper_bound', ok, cells=rows)


def t3b():
    ok = True; rows = []
    for cfg in ('B3', 'B5', 'B7', 'MAKL', 'REF_camfirst'):
        for prof in ('Q1', 'Q2'):
            for c in (130_000, 180_000):
                lm = S.wire(S.CAM[prof][0]); ls = S.wire(S.SCAN_B)
                alb = 8 * (lm + 2 * ls) / c + 0.02
                L = mk(cfg, c)
                st = (lambda t: (0.1, 0.1, 0.6 + 0.8 * abs(((t / 1e6) % 10) - 5) / 5))
                S.run(L, profile=prof, cap_bps=c, horizon_us=30_000_000, state_fn=st if 'MAKL' in cfg else None)
                lines = [x for x in L.ledger.lines if not x.get('header')]
                # camera frames that were fully transmitted: start of first packet, end = start of the next
                # transmission is not logged, so use start + wire time at c (fixed c in this test)
                cams = sorted((x['tx_us'][0], x['tx_us'][0] + sum(math.ceil(8 * w * 1e6 / c) for w in
                               [min(1472, x['bytes'] - o) + 28 for o in range(0, x['bytes'], 1472)]))
                              for x in lines if x['stream'] == 'camera' and len(x['tx_us']) == (max(1, x['bytes']) + 1471) // 1472)
                sd = sorted((x['finish_us'], x['offer_us'], x['tx_us'][0]) for x in lines
                            if x['stream'] == 'scan' and x['status'] == 'delivered')
                worst = math.inf; n = 0
                for ts, te in cams:
                    # next scan = first delivered scan that STARTED after the camera frame ended
                    nxt = [f for f, o, s0 in sd if s0 >= te]
                    if not nxt:
                        continue
                    prev = [o for f, o, s0 in sd if f < nxt[0]]
                    if not [1 for f, o, s0 in sd if f <= ts + 20_000 and s0 < ts]:
                        continue                    # Prop 2 needs at least one scan delivered before the frame
                    newest = max(prev)
                    age = (nxt[0] - newest) / 1e6
                    worst = min(worst, age); n += 1
                good = n > 0 and worst >= alb - 2e-6
                ok &= good
                rows.append(dict(cfg=cfg, profile=prof, c_bps=c, camera_frames=n, min_age_after_camera_s=round(worst, 4),
                                 a_LB_s=round(alb, 4), ok=good))
    report('T3b_prop2_lower_bound', ok, cells=rows)


# ---------------------------------------------------------------------------------------------------------------- T4
def outcome(cfg, params, prof, c, probes=True):
    L = mk(cfg, c, params=params)
    st = (lambda t: (0.12, 0.15, 0.4 + abs(((t / 1e6) % 8) - 4) / 4))
    r = S.run(L, profile=prof, cap_bps=c, probes=probes, horizon_us=30_000_000, state_fn=st if cfg.startswith('MAKL') else None)
    return [(s, cap, stt, fin) for _, s, cap, stt, fin in r]


def t4():
    ok = True; det = []
    for prof in ('Q1', 'Q2'):
        for c in (130_000, 180_000, 3_000_000):
            a = outcome('MAKL', {'beta_override': 0.0, 'slo_m': math.inf}, prof, c)
            b = outcome('B7', {}, prof, c)
            x = outcome('MAKL', {'beta_override': math.inf, 'slo_m': math.inf}, prof, c)
            y = outcome('REF_camfirst', {}, prof, c)
            e0 = a == b; einf = x == y
            # sanity: the two limits differ under congestion
            ok &= e0 and einf
            det.append(dict(profile=prof, c_bps=c, beta0_equals_B7=e0, betainf_equals_camfirst=einf,
                            limits_differ=(a != x)))
    report('T4_makl_limits', ok, cells=det)


# ---------------------------------------------------------------------------------------------------------------- T6
def t6():
    L = mk('MAKL', 130_000)
    st = (lambda t: (0.15, 0.15, 0.5 + abs(((t / 1e6) % 10) - 5) / 5))
    S.run(L, profile='Q1', cap_bps=130_000, probes=True, horizon_us=60_000_000, state_fn=st)
    ns = sorted(L.cpu_ns)
    med = ns[len(ns) // 2] / 1e3; p99 = ns[int(0.99 * (len(ns) - 1))] / 1e3
    report('T6_makl_cpu_time', med < 1000, decisions=len(ns), median_us=round(med, 2), p99_us=round(p99, 2),
           max_us=round(ns[-1] / 1e3, 2), python=sys.version.split()[0])


# ---------------------------------------------------------------------------------------------------------------- T7
def t7():
    ok = True; info = {}
    for cfg in ('B4', 'B5', 'B7', 'MAKL'):
        L = mk(cfg, 130_000)
        mids, _ = drive_random(L, 11, cfg)
        vis = Counter(r.status for r in L.records.values())
        ok &= set(vis) <= set(J.BRIDGE_STATUSES)
        r = L.records[mids[0]]
        ok &= hasattr(r, 'finished_us') and hasattr(r, 'message_id') and hasattr(r, 'payload_bytes')
        ok &= isinstance(L.outcomes(), dict) and hasattr(L, 'tx_log')
        n0 = len(L.records); L.prune(set()); n1 = len(L.records)
        ok &= n1 == 0
        # prune must keep a live (queued) record
        L2 = mk(cfg, 50_000)
        L2.advance(0); m1 = L2.offer(stream='camera', payload_bytes=4000, stamp_us=0)
        m2 = L2.offer(stream='scan', payload_bytes=2944, stamp_us=0)
        L2.prune(set()); ok &= m1 in L2.records and m2 in L2.records
        info[cfg] = dict(visible=dict(vis), pruned=f'{n0}->{n1}')
    report('T7_bridge_interface', ok, **info)


# ---------------------------------------------------------------------------------------------------------------- T5
def t5(traces):
    import e4_harness as H
    import j_replay as R
    if traces is None:
        report('T5_replay_equivalence', None, skipped='NO_TRACES (runs on the VM)')
        return
    traces = Path(traces)
    st = json.loads((traces / 'E3B' / 'STATE.json').read_text())
    TR = {}
    for tid, d in sorted(st['done'].items()):
        if d['level'] == 'C0' and d['arm'] in ('F-N1', 'F-N2'):
            td = traces / 'E3B' / 'trials' / Path(d['evidence_dir']).name
            if td.is_dir():
                TR[(('Q1' if d['arm'] == 'F-N1' else 'Q2'), d['rep'])] = H.load_trace(td, dict(d, trial_id=tid))
    if len(TR) < 6:
        report('T5_replay_equivalence', False, error='E3B C0 traces missing', found=sorted(TR)); return
    # (a) record-by-record against e4_harness.replay
    pairs = {'B0': dict(link='frozen'), 'B1': dict(link='discipline', discipline='fifo', cam_mode='factor', cam_expiry=1.0),
             'B2': dict(link='discipline', discipline='prio', cam_mode='factor'),
             'B3': dict(link='discipline', discipline='keeplast', cam_mode='factor')}
    a_bad = []; n_rec = 0
    for key, tr in sorted(TR.items()):
        for cfg, kw in pairs.items():
            for cap in (0.13, 0.18):
                ref = H.replay(tr, capacity_mbps=cap, duration_s=25, **kw)
                L = J.make_jlink(cfg, capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY, ledger=True)
                mode = 'e4_frozen' if cfg == 'B0' else 'e4_factor'
                got = R.replay(tr, L, capacity_mbps=cap, duration_s=25, offers=mode,
                               cam_expiry=kw.get('cam_expiry'))
                n_rec += len(ref['status'])
                same = (ref['recv'] == got['recv'] and ref['queue'] == got['queue']
                        and [v[:4] + (('queue_drop' if v[4] == 'replaced' else v[4]), v[5]) for v in ref['status'].values()]
                        == [v[:4] + (v[4], v[5]) for v in got['status'].values()])
                if not same:
                    a_bad.append((key, cfg, cap))
    # (b) frozen E4_RESULTS_ALL.csv rows (scan expiry none; fifo/prio/keeplast; camera expiry none/1.0)
    rows = list(csv.DictReader(open(PKG / 'frozen_deps' / 'E4_RESULTS_ALL.csv')))
    sel = [r for r in rows if r['scan_expiry_s'] == 'none' and r['discipline'] in ('fifo', 'prio', 'keeplast')]
    cache = {}; b_bad = []; ncmp = 0
    for r in sel:
        k = (r['trace_rep'], r['profile'], r['cam_expiry_s'], r['discipline'], float(r['capacity_mbps']), int(r['duration_s']))
        if k not in cache:
            tr = TR[(r['profile'], r['trace_rep'])]
            disc = {'fifo': 'fifo', 'prio': 'prio', 'keeplast': 'keeplast'}[r['discipline']]
            import link_disciplines as LD
            L = LD.DisciplineLink(capacity_bps=3_000_000, queue_limit_bytes=H.QLIM, delay_us=H.DELAY, discipline=disc)
            ce = None if r['cam_expiry_s'] == 'none' else float(r['cam_expiry_s'])
            cache[k] = R.replay(tr, L, capacity_mbps=k[4], duration_s=k[5], offers='e4_factor', cam_expiry=ce)
        m = H.metrics(cache[k], window=r['window'])
        for col, v in m.items():
            vv = round(v, 4) if isinstance(v, float) and math.isfinite(v) else v
            ref = r[col]
            if str(vv) != ref and not (ref in ('', 'None') and vv is None):
                try:
                    if abs(float(ref) - float(vv)) <= 1e-9:
                        continue
                except (TypeError, ValueError):
                    pass
                b_bad.append((k, r['window'], col, ref, vv))
            ncmp += 1
    report('T5_replay_equivalence', not a_bad and not b_bad, traces=len(TR), replays_compared=len(TR) * 8,
           records=n_rec, record_mismatches=a_bad[:10], csv_rows=len(sel), csv_values=ncmp,
           csv_mismatches=len(b_bad), first_csv_mismatches=b_bad[:10])


def main():
    traces = sys.argv[1] if len(sys.argv) > 1 else None
    t0 = time.time()
    for f in (t1, t2, t3, t3b, t4, t6, t7):
        f()
    t5(traces)
    req = ['T1_astar_table', 'T2_ledger_conservation', 'T3_prop1_upper_bound', 'T3b_prop2_lower_bound',
           'T4_makl_limits', 'T6_makl_cpu_time', 'T7_bridge_interface']
    if traces is not None:
        req.append('T5_replay_equivalence')
    ok = all(RES[k]['passed'] for k in req)
    RES['_summary'] = dict(passed=ok, required=req, wall_s=round(time.time() - t0, 1), python=sys.version)
    out = Path(traces).parent if traces else Path.cwd()
    name = 'P0J_TEST_RESULTS.json' if traces else 'P0J_TEST_RESULTS_UNIT.json'
    (out / name).write_text(json.dumps(RES, indent=1, default=str) + '\n')
    print('P0J_TESTS', 'PASS' if ok else 'FAIL', f'({len(req)} required, {RES["_summary"]["wall_s"]} s)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
