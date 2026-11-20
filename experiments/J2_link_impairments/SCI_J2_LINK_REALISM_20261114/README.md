# SCI_J2_LINK_REALISM_20261114 — J2: is the J1 result robust to jitter, loss and varying capacity? (252 counted runs)

Simulation only. Protocol: `J2_PROTOCOL_FREEZE.md` (frozen; hashes in `J2_FREEZE_HASHES.json`).
Needs on the VM (all hash-checked, none edited): `SCI_J1_SCHEDULERS_V2_20261113`, `SCI_P0J_SCHEDULERS_20261111`,
`SCI_E7_QUEUE_DISCIPLINE_20261102`, `SCI_E5_PED_SPEED_20261101`, `SCI_E3B_EDGE_NAV_RUNNER_20261031`.

## Run (about 10 h, resumable)
```bash
cd ~/ros2_ws && unzip -n ~/Downloads/SCI_J2_LINK_REALISM_20261114.zip -d ~/ros2_ws && bash SCI_J2_LINK_REALISM_20261114/JALANKAN_J2.sh
```
Upload `~/Downloads/SCI_J2_RESULTS_<UTC>.zip` and `~/Downloads/SCI_J2_CONSOLE_<UTC>.log`.
If it stops (halt or power cut), upload the console log first; do not delete `~/ros2_ws/results/SCI_J2_LINK_REALISM_20261114`.

## Files
| File | Content |
|---|---|
| `j2_impair.py` | jitter, Bernoulli loss, Gilbert–Elliott loss (in time), square-wave capacity; common random numbers |
| `j2_bridge.py` | J1 bridge with the impaired link; `J2_IMPAIR` and `J2_STATE` log lines |
| `j2_run_trial.py` | J1 v2 `j1_run_trial.py` + declared edits (`tests/j2_run_trial_edits.py`) |
| `j2_metrics.py`, `j2_analysis.py` | frozen J1 measures + channel loss and impairment evidence; frozen 3-pair tests and verdicts |
| `j2_run_all.py` | lock -> 2 gates -> 252 runs -> analysis -> ZIP |
| `tests/test_j2_offline.py` | impairment model; live bridge (ROS stubbed) = impaired replay for B3, B7, MAKL under 5 conditions; matrix; analysis |
