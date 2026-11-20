#!/usr/bin/env python3
"""Builds the J5 replay traces from the archived J1 runs without congestion (C0), B7, rep 1-5, camera Q1 and Q2
(as J0 did: every offered frame was delivered, and the trajectory is the C0 one).
Usage: make_traces.py J1_TRIALS_DIR OUT_DIR   ->  J5_TRACE_<Q1|Q2>_R<rep>.json
Uses only frozen P0-J code: e4_harness.load_trace (offers) and j_replay.state_series (robot state for MAKL).
JALANKAN_J5_EDGE.sh runs it on the VM: the result must be byte-identical to the traces shipped in the package."""
import glob, hashlib, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5_common as C

SRC = ('TRIAL_RESULT.json', 'E3B_FRAMES.jsonl', 'E3B_EDGE_SCAN.jsonl', 'c52_joint_bridge.log', 'E3B_GRID.jsonl')


def build(trials, out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True); made = []
    for cell in ('Q1', 'Q2'):
        for rep in C.REPS:
            ds = sorted(glob.glob(str(Path(trials) / f'J1_R{rep}_C0-{cell}_B7_a*')))
            if len(ds) != 1:
                raise SystemExit(f'J5_TRACE_SOURCE_NOT_UNIQUE {cell} R{rep} {ds}')
            d = Path(ds[0]); res = json.loads((d / 'TRIAL_RESULT.json').read_text())
            if res.get('status') != 'SUCCEEDED':
                raise SystemExit('J5_TRACE_SOURCE_NOT_SUCCEEDED ' + d.name)
            tr = C.H.load_trace(d, res)
            if tr['profile'] != cell:
                raise SystemExit('J5_TRACE_PROFILE_MISMATCH ' + d.name)
            grid = C.JR.load_grid(d)
            tr['states'] = C.JR.state_series(grid)
            tr['grid'] = [{'sim_s': g['sim_s'], 'v_mps': g.get('v_mps'), 'clearance_m': g.get('clearance_m')}
                          for g in grid if g.get('sim_s') is not None]
            tr['source_dir'] = d.name
            tr['source_sha256'] = {f: hashlib.sha256((d / f).read_bytes()).hexdigest() for f in SRC}
            p = out / f'J5_TRACE_{cell}_R{rep}.json'
            p.write_text(json.dumps(tr, sort_keys=True, separators=(',', ':')) + '\n'); made.append(p)
    return made


if __name__ == '__main__':
    for p in build(sys.argv[1], sys.argv[2]):
        print('J5_TRACE', p.name, hashlib.sha256(p.read_bytes()).hexdigest()[:12])
