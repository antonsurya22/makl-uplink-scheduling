# SCI_J3_SWITCH_ABLATION_20261116 — J3: does MAKL need a*(v,d), or is a distance switch enough? (75 counted runs)

Simulation only. Protocol: `J3_PROTOCOL_FREEZE.md` (frozen; hashes in `J3_FREEZE_HASHES.json`).
Needs on the VM (all hash-checked, none edited): `SCI_J1_SCHEDULERS_V2_20261113`, `SCI_P0J_SCHEDULERS_20261111`,
`SCI_E7_QUEUE_DISCIPLINE_20261102`, `SCI_E5_PED_SPEED_20261101`, `SCI_E3B_EDGE_NAV_RUNNER_20261031`.

## Run (about 3 h, resumable)
```bash
cd ~/ros2_ws && unzip -n ~/Downloads/SCI_J3_SWITCH_ABLATION_20261116.zip -d ~/ros2_ws && bash SCI_J3_SWITCH_ABLATION_20261116/JALANKAN_J3.sh
```
Upload `~/Downloads/SCI_J3_RESULTS_<UTC>.zip` and `~/Downloads/SCI_J3_CONSOLE_<UTC>.log`.
If it stops (halt or power cut), upload the console log first; do not delete `~/ros2_ws/results/SCI_J3_SWITCH_ABLATION_20261116`.

## Files
| File | Content |
|---|---|
| `j3_links.py` | distance switch DSW_0.8 / 1.0 / 1.2 (new); B7 and MAKL from the unchanged J1 factory |
| `j3_bridge.py` | J1 bridge, unchanged, with the extended link factory (the switch gets the same d input as MAKL) |
| `j3_run_trial.py` | J1 v2 `j1_run_trial.py` + declared edits (`tests/j3_run_trial_edits.py`) |
| `j3_analysis.py` | frozen J1 measures; J1 paired rule; hypotheses H1–H4, R; outcome |
| `j3_run_all.py` | lock -> 2 gates -> 75 runs -> analysis -> ZIP |
| `tests/test_j3_offline.py` | switch rule; live bridge (ROS stubbed) = replay for all 5 arms; matrix; gates; analysis |
