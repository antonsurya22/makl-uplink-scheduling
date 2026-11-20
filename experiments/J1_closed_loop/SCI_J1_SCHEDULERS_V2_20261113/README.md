# SCI_J1_SCHEDULERS_V2_20261113 — J1 v2: MAKL vs seven baselines in the closed loop (240 counted runs)

Simulation only. Protocol: `J1_PROTOCOL_FREEZE.md` (frozen; hashes in `J1_FREEZE_HASHES.json`).
Needs on the VM (all hash-checked, none edited): `SCI_P0J_SCHEDULERS_20261111`, `SCI_E7_QUEUE_DISCIPLINE_20261102`,
`SCI_E5_PED_SPEED_20261101`, `SCI_E3B_EDGE_NAV_RUNNER_20261031`.

## Run (about 9.5-10 h, resumable)
```bash
cd ~/ros2_ws && unzip -n ~/Downloads/SCI_J1_SCHEDULERS_V2_20261113.zip -d ~/ros2_ws && bash SCI_J1_SCHEDULERS_V2_20261113/JALANKAN_J1.sh
```
Upload `~/Downloads/SCI_J1_V2_RESULTS_<UTC>.zip` and `~/Downloads/SCI_J1_V2_CONSOLE_<UTC>.log`.
If it stops (halt or power cut), upload the console log first; do not delete `~/ros2_ws/results/SCI_J1_SCHEDULERS_V2_20261113` or the halted v1 folder `~/ros2_ws/results/SCI_J1_SCHEDULERS_20261112` (archived in the v2 ZIP).

v2 = v1 with amendment A1 (gate check of the capacity plan; thread CPU time). See the protocol, section 12.

## Files
| File | Content |
|---|---|
| `j1_links.py` | P0-J schedulers + capture stamps + MAKL decision log |
| `j1_bridge.py` | E7 launcher pattern; MAKL inputs (raw scan d, /odom, /e2_nav_cmd_policy) |
| `j1_run_trial.py` | E7 `e7_run_trial.py` + declared edits (`tests/j1_run_trial_edits.py`) |
| `j1_metrics.py`, `j1_analysis.py` | frozen E5/E7 measures + J1 items; frozen paired tests and verdicts |
| `j1_run_all.py` | lock -> 2 gates -> 240 runs -> analysis -> ZIP |
| `tests/test_j1_offline.py` | live bridge (ROS stubbed) = replay for all 8 arms, MAKL inputs, plans, metrics, analysis, matrix |
