#!/usr/bin/env python3
"""J2 per-run measures = the frozen J1 v2 measures (j1_metrics.run_measures, unchanged) + J2 items:
  ledger: channel_loss counts per stream in W (scan, camera, probe ACK);
  impairment evidence: the last 'J2_IMPAIR' and 'J2_STATE' lines of the bridge log (packets, lost packets, loss rate,
  mean |jitter|, capacity changes)."""
import json, re, sys
from pathlib import Path
WS = Path.home() / 'ros2_ws'
J1 = WS / 'SCI_J1_SCHEDULERS_V2_20261113'
if str(J1) not in sys.path:
    sys.path.insert(0, str(J1))
import j1_metrics as M1


def bridge_j2(td):
    p = Path(td) / 'c52_joint_bridge.log'
    out = {'j2_impair_line': None, 'j2_state': None}
    if not p.is_file():
        return out
    for l in p.read_text(errors='replace').splitlines():
        if l.startswith('J2_IMPAIR '):
            out['j2_impair_line'] = l
        elif l.startswith('J2_STATE '):
            out['j2_state'] = l
    if out['j2_state']:
        try:
            s = json.loads(out['j2_state'].split(' ', 2)[2])
            out.update({'j2_' + k: v for k, v in s.items()})
        except Exception as e:
            out['j2_state_error'] = repr(e)
    if out['j2_impair_line']:
        m = re.search(r'condition=(\S+) seed=(\d+)', out['j2_impair_line'])
        if m:
            out['j2_condition_bridge'] = m.group(1); out['j2_seed_bridge'] = int(m.group(2))
    return out


def run_measures(td, res):
    m = M1.run_measures(td, res)
    hdr, rows = M1.M7.read_ledger(Path(td) / 'J1_LEDGER.jsonl')
    t0 = res.get('trigger_sim_s')
    if rows and t0 is not None:
        a = int(round(t0 * 1e6)); b = a + 25_000_000
        W = [r for r in rows if a <= r['offer_us'] < b]
        for s in ('scan', 'camera', 'c52_probe_ack'):
            m[f'L_{s}_channel_loss'] = sum(1 for r in W if r['stream'] == s and r['status'] == 'channel_loss')
        m['L_all_channel_loss'] = sum(1 for r in rows if r['status'] == 'channel_loss')
    m.update(bridge_j2(td))
    return m
