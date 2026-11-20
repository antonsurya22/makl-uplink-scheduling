#!/usr/bin/env python3
"""Collect the recorded per-run traces that J0 and test T5 need, READ-ONLY, into one new folder.
Usage: export_j0_traces.py OUT_DIR
Studies (frozen results, nothing is changed):
  E3B  ~/ros2_ws/results/SCI_E3B_EDGE_NAV_20261031/E3B_MAIN          (T5 equivalence: C0 F-N1/F-N2 traces of E4)
  E5   ~/ros2_ws/results/SCI_E5_PED_SPEED_20261101/E5_MAIN           (J0: offered Q1/Q2 load + v(t), d(t))
  E7   ~/ros2_ws/results/SCI_E7_QUEUE_DISCIPLINE_20261102/E7_MAIN    (= article E6; J0 calibration, ledger)
For each run in STATE.json['done'] the files below are copied from its evidence folder; if that folder is gone, from
the study's results ZIP in ~/Downloads (newest first). The bridge log is filtered to the lines J0 uses.
Writes OUT_DIR/<study>/STATE.json, OUT_DIR/<study>/trials/<run>/..., and OUT_DIR/TRACE_INDEX.json."""
import json, sys, zipfile
from pathlib import Path
WS = Path.home() / 'ros2_ws'
DL = Path.home() / 'Downloads'
STUDIES = {
    'E3B': (WS / 'results/SCI_E3B_EDGE_NAV_20261031/E3B_MAIN', 'SCI_E3B_EDGE_NAV_RESULTS_*.zip'),
    'E5': (WS / 'results/SCI_E5_PED_SPEED_20261101/E5_MAIN', 'SCI_E5_PED_SPEED_RESULTS_*.zip'),
    'E7': (WS / 'results/SCI_E7_QUEUE_DISCIPLINE_20261102/E7_MAIN', 'SCI_E7_QUEUE_DISCIPLINE_RESULTS_*.zip'),
}
FILES = ('E3B_GRID.jsonl', 'E3B_FRAMES.jsonl', 'E3B_EDGE_SCAN.jsonl', 'E3B_LINK.jsonl', 'E7_LEDGER.jsonl',
         'E2_CROSSING_GROUND_TRUTH.jsonl', 'TRIAL_RESULT.json', 'TRIAL_SPEC.json')
LOG = 'c52_joint_bridge.log'
LOG_KEYS = ('C2_BRIDGE', 'network_phase', 'E7_BRIDGE', 'episode_', 'trigger')


def filt(text):
    return ''.join(l + '\n' for l in text.splitlines() if any(k in l for k in LOG_KEYS))


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=False)
    index = {}
    for study, (main_dir, pat) in STUDIES.items():
        st_path = main_dir / 'STATE.json'
        zips = sorted(DL.glob(pat), key=lambda p: p.stat().st_mtime, reverse=True)
        zf = None
        if not st_path.is_file():
            if not zips:
                index[study] = {'error': 'NO_STATE_AND_NO_ZIP', 'looked': str(st_path)}
                print('TRACE', study, 'MISSING', flush=True)
                continue
            zf = zipfile.ZipFile(zips[0])
            state = json.loads(zf.read('STATE.json'))
        else:
            state = json.loads(st_path.read_text())
        so = out / study
        (so / 'trials').mkdir(parents=True)
        (so / 'STATE.json').write_text(json.dumps(state))
        n_ok = n_zip = n_missing = 0
        missing = []
        for tid, d in sorted(state.get('done', {}).items()):
            ev = Path(d.get('evidence_dir') or '')
            name = ev.name
            td = so / 'trials' / name
            td.mkdir()
            got = 0
            if ev.is_dir():
                for f in FILES:
                    if (ev / f).is_file():
                        (td / f).write_bytes((ev / f).read_bytes()); got += 1
                if (ev / LOG).is_file():
                    (td / LOG).write_text(filt((ev / LOG).read_text(errors='replace'))); got += 1
                n_ok += 1
            else:
                if zf is None and zips:
                    zf = zipfile.ZipFile(zips[0])
                if zf is not None:
                    names = set(zf.namelist())
                    for f in FILES:
                        k = f'trials/{name}/{f}'
                        if k in names:
                            (td / f).write_bytes(zf.read(k)); got += 1
                    k = f'trials/{name}/{LOG}'
                    if k in names:
                        (td / LOG).write_text(filt(zf.read(k).decode(errors='replace'))); got += 1
                if got:
                    n_zip += 1
                else:
                    n_missing += 1; missing.append(tid)
        index[study] = {'runs': len(state.get('done', {})), 'from_folder': n_ok, 'from_zip': n_zip,
                        'missing': n_missing, 'missing_ids': missing[:20],
                        'zip_used': str(zips[0]) if (zf is not None and zips) else None}
        print('TRACE', study, json.dumps(index[study]), flush=True)
    (out / 'TRACE_INDEX.json').write_text(json.dumps(index, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
