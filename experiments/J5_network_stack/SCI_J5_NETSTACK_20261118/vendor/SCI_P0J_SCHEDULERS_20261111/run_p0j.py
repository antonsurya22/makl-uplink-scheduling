#!/usr/bin/env python3
"""P0-J batch (offline only; no ROS, no Gazebo, no robot). Called by JALANKAN_P0J.sh.
 1. verify this package against P0J_FREEZE_HASHES.json and the vendored frozen files against their original freezes
    (and, if present on the VM, against the frozen packages in ~/ros2_ws);
 2. unit tests T1-T4, T6, T7 (no traces);
 3. export the recorded traces (read-only) of E3-B, E5 and E7 (= article E6);
 4. test T5 (replay equivalence with the frozen E4 harness and the frozen E4_RESULTS_ALL.csv);
 5. J0 exploratory replay (only if 1-4 passed);
 6. results ZIP in ~/Downloads.
Stops at a failed step 1-3 and still writes the ZIP with what exists; a T5 failure is reported and J0 still runs
(exploratory, flagged as not trusted)."""
import datetime, hashlib, json, os, subprocess, sys, zipfile
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
PY = sys.executable
UTC = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
OUT = WS / 'results' / 'SCI_P0J_SCHEDULERS_20261111' / UTC
ORIGINAL = {  # vendored file -> (original freeze, SHA256 recorded there)
    'frozen_deps/link_disciplines.py': ('E4_FREEZE_HASHES_V3 / E7_FREEZE_HASHES', 'edebf71ac9d8d7e6768dff4af377962194b4e3e661ffe9d6c8724be331d2684e'),
    'frozen_deps/e4_harness.py': ('E4_FREEZE_HASHES_V3 / E7_FREEZE_HASHES', '4857643f10d2079d9c322240144d0281fbd5c7a07b51ef16e470bb14877f62f4'),
    'frozen_deps/frozen/shared_link.py': ('GATE_B1 / E4 / E7', '04f290705241f30300fe643717a8728d52570e572d5c5dbf35098ef1d8d9acab'),
    'frozen_deps/e7_link.py': ('E7_FREEZE_HASHES', 'afbcfd980afea8aed63d5f593d195aaf98b94867b489d09b9c72daa7929ea23d'),
    'frozen_deps/e3b_policy_logic.py': ('E3B SHA256.json', '5e83eaaf61ddbaaffbca1364afa55dc1cc24697aa4edcd68cd83d15fc0c87311'),
    'frozen_deps/e3_policy_logic.py': ('E3B SHA256.json', 'c737c9cd6f6541962276fed617b797f73376992cb0e1d515c48218af3f3df2be'),
    'frozen_deps/e2_policy_logic.py': ('E3B SHA256.json', 'ddacb29855def9570453769b7537e0392c3f7e213ce45e2cb9dc6242b05836d4'),
    'frozen_deps/e2_event_logic.py': ('E3B SHA256.json', '7913d5f0ab09eded8143beb97f60369bc6649ee8c9b03a51eeb7cc3a383c4739'),
    'frozen_deps/e5_metrics.py': ('E5_FREEZE_HASHES', 'df8e80b154931b002b96e763773ba21acb3a2c2b305adf385d760919cda6e296'),
}
ON_VM = {  # vendored file -> copy in the frozen package on the VM (compared if it exists)
    'frozen_deps/link_disciplines.py': WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102/e4_frozen/link_disciplines.py',
    'frozen_deps/e4_harness.py': WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102/e4_frozen/e4_harness.py',
    'frozen_deps/frozen/shared_link.py': WS / 'GATE_B1_Link_Core/shared_link.py',
    'frozen_deps/e7_link.py': WS / 'SCI_E7_QUEUE_DISCIPLINE_20261102/e7_link.py',
    'frozen_deps/e3b_policy_logic.py': WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031/e3b_policy_logic.py',
    'frozen_deps/e5_metrics.py': WS / 'SCI_E5_PED_SPEED_20261101/e5_metrics.py',
}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def say(*a):
    print('P0J', *a, flush=True)


def step(name, cmd):
    say('STEP', name, '...')
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(OUT))
    (OUT / f'{name}.log').write_text(p.stdout)
    print(p.stdout[-6000:], flush=True)
    say('STEP', name, 'exit', p.returncode)
    return p.returncode == 0


def finish(status):
    (OUT / 'P0J_STATUS.json').write_text(json.dumps(status, indent=1) + '\n')
    z = Path.home() / 'Downloads' / f'SCI_P0J_RESULTS_{UTC}.zip'
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zz:
        for f in sorted(OUT.rglob('*')):
            if f.is_file():
                zz.write(f, str(f.relative_to(OUT)))
        for f in ('P0J_FREEZE_HASHES.json', 'README.md'):
            if (HERE / f).is_file():
                zz.write(HERE / f, 'SOURCE/' + f)
    say('RESULTS_ZIP', z, sha(z))
    say('STATUS', json.dumps(status))
    say('Upload this ZIP and the console log to Claude.')


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    say('OUT', OUT, 'python', sys.version.split()[0])
    status = {'utc': UTC, 'python': sys.version}
    # 1. hashes
    fz = json.loads((HERE / 'P0J_FREEZE_HASHES.json').read_text())
    bad = [f for f, h in fz['sha256'].items() if not (HERE / f).is_file() or sha(HERE / f) != h]
    orig_bad = [f for f, (_, h) in ORIGINAL.items() if sha(HERE / f) != h]
    vm = {f: (None if not p.is_file() else sha(p) == sha(HERE / f)) for f, p in ON_VM.items()}
    status['hashes'] = dict(package_changed=bad, vendored_not_equal_original=orig_bad, equal_to_vm_copy=
                            {f: v for f, v in vm.items()}, freeze_utc=fz.get('frozen_utc'))
    say('HASHES', json.dumps(status['hashes']))
    if bad or orig_bad or any(v is False for v in vm.values()):
        status['result'] = 'HALT_HASHES'; finish(status); return 2
    # 2. unit tests without traces
    ok = step('T_unit', [PY, '-I', str(HERE / 'tests/test_p0j.py')])
    status['unit_tests'] = ok
    if not ok:
        status['result'] = 'HALT_UNIT_TESTS'; finish(status); return 3
    # 3. traces
    tr = OUT / 'traces'
    ok = step('export_traces', [PY, '-I', str(HERE / 'export_j0_traces.py'), str(tr)])
    status['traces'] = json.loads((tr / 'TRACE_INDEX.json').read_text()) if (tr / 'TRACE_INDEX.json').is_file() else None
    if not ok:
        status['result'] = 'HALT_TRACES'; finish(status); return 4
    # 4. T5 with traces (writes P0J_TEST_RESULTS.json into OUT)
    ok = step('T5_with_traces', [PY, '-I', str(HERE / 'tests/test_p0j.py'), str(tr)])
    status['all_tests_with_traces'] = ok
    if not ok:      # J0 is exploratory: run it anyway, but it is not trusted until T5 is diagnosed
        say('WARNING T5 FAILED: J0 still runs (exploratory) but is NOT trusted until the T5 failure is diagnosed')
    # 5. J0
    ok = step('J0_explore', [PY, '-I', str(HERE / 'j0_explore.py'), str(tr), str(OUT / 'J0')])
    status['j0_ran'] = ok
    if (OUT / 'J0' / 'J0_SUMMARY.json').is_file():
        status['j0_gate'] = json.loads((OUT / 'J0' / 'J0_SUMMARY.json').read_text()).get('gate_Q1_018_25s')
    status['result'] = ('DONE' if status['all_tests_with_traces'] else 'DONE_BUT_T5_FAILED') if ok else 'J0_ERROR'
    finish(status)
    return 0 if (ok and status['all_tests_with_traces']) else 6


if __name__ == '__main__':
    sys.exit(main())
