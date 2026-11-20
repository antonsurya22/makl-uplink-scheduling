# SCI_J4_SPEED_ABLATION_20261117 — J4: does MAKL's speed term matter? (75 counted runs)

Simulation only. The protocol is `J4_PROTOCOL_FREEZE.md` plus amendment `J4_PROTOCOL_AMENDMENT_A1.md` (pilot checks P3–P5;
written after the pilots and before any gate or counted run). Hashes are in `J4_FREEZE_HASHES.json`.

The VM must hold these packages (all are hash-checked and none is edited):
- `SCI_J3_SWITCH_ABLATION_20261116`
- `SCI_J1_SCHEDULERS_V2_20261113`
- `SCI_P0J_SCHEDULERS_20261111`
- `SCI_E7_QUEUE_DISCIPLINE_20261102`
- `SCI_E5_PED_SPEED_20261101`
- `SCI_E3B_EDGE_NAV_RUNNER_20261031`

## Run (about 4 h, resumable)
First run, or after amendment A1 (`-o` overwrites the package files; the results folder is not touched):
```bash
cd ~/ros2_ws && unzip -o ~/Downloads/SCI_J4_SPEED_ABLATION_20261117_A1.zip -d ~/ros2_ws && bash SCI_J4_SPEED_ABLATION_20261117/JALANKAN_J4.sh
```
With amendment A1 and the halted STATE from the first attempt, the runner re-evaluates the six stored pilots (no re-run).
It then continues with the 3 gates and the 75 counted runs, which take about 3.5 h.
The run goes in this order:
1. offline tests;
2. 6 pilot runs (C0, MAKL, 2 per speed; not counted);
3. 3 gate runs (not counted);
4. 75 counted runs;
5. analysis and the results ZIP.

If a pilot fails, J4 stops before any counted run. The only exception is the pre-declared fallback from 0.22 to 0.20 m/s,
which runs on its own.

Upload `~/Downloads/SCI_J4_RESULTS_<UTC>.zip` and `~/Downloads/SCI_J4_CONSOLE_<UTC>.log`. If the run stops (a halt, a pilot
failure or a power cut), upload the console log first, and do not delete `~/ros2_ws/results/SCI_J4_SPEED_ABLATION_20261117`.

## Files
| File | Content |
|---|---|
| `j4_links.py` | J3 links + `MAKL_cap` (= MAKL-FixedV, the frozen P0-J configuration with v = 0.15 in a\*) + v̂ in the decision log |
| `j4_bridge.py` | J1 bridge, unchanged, with the J4 link factory |
| `j4_stack.py`, `j4_mode_gate.py`, `j4_guard.py` | speed level V: DWB cap = V, mode-gate NORMAL cap = V, guard MAX_LIN = max(0.18, V) (same mechanism as E8) |
| `j4_run_trial.py` | J3 `j3_run_trial.py` + declared edits (`tests/j4_run_trial_edits.py`) |
| `j4_metrics.py` | phases A/B, F_run, viol_A/B, d_sw, speed implementation, agreement, safety ticks, pilot measures |
| `j4_analysis.py` | J per speed, J-rule, sign rule, mechanism rule; M1, M2, S0, S1, S2, D1, D2, H3–H5, R; outcomes |
| `j4_run_all.py` | lock → pilots (with fallback) → gates → 75 runs → analysis → ZIP |
| `tests/test_j4_offline.py` | provenance; byte check; FixedV = MAKL at 0.15 and differs at 0.10 / 0.22; speed edits; bridge chain; metrics on archived J1 runs; matrix; pilot rule; analysis outcomes |
