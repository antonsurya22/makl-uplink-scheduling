# Experiments

Hash = first 16 hex digits of the SHA256 of the freeze file (full values: `python3 tools/verify_freezes.py`).

## J experiments (in this repo)

| Paper | Package | Protocol | Freeze file (hash) | Counted runs | Results |
|---|---|---|---|---|---|
| P0-J | `SCI_P0J_SCHEDULERS_20261111` | package README | `P0J_FREEZE_HASHES.json` (e5bccfc25d764c4f) | unit tests | `experiments/P0J_schedulers/results` |
| J0 | `analysis/exploratory/j0_explore_v2.py` | exploratory | - | 300 traces | `analysis/exploratory/results/j0` |
| J1 | `SCI_J1_SCHEDULERS_V2_20261113` | `J1_PROTOCOL_FREEZE.md` (amendment A1 in section 12) | `J1_FREEZE_HASHES.json` (befd9065f79b9ae7) | 240 | `experiments/J1_closed_loop/results` |
| J1, first version (not used) | `SCI_J1_SCHEDULERS_20261112` | `J1_PROTOCOL_FREEZE.md` | `J1_FREEZE_HASHES.json` (91d5462eeedb2172) | 0 | - |
| J2 | `SCI_J2_LINK_REALISM_20261114` | `J2_PROTOCOL_FREEZE.md` | `J2_FREEZE_HASHES.json` (a25b2f605e6b4b42) | 252 | `experiments/J2_link_impairments/results` |
| J3 | `SCI_J3_SWITCH_ABLATION_20261116` | `J3_PROTOCOL_FREEZE.md` | `J3_FREEZE_HASHES.json` (b0ba0679fe922fd9) | 75 | `experiments/J3_distance_switch/results` |
| J4 | `SCI_J4_SPEED_ABLATION_20261117` | `J4_PROTOCOL_FREEZE.md`, `J4_PROTOCOL_AMENDMENT_A1.md` | `J4_FREEZE_HASHES.json` (2b48368bef385384) | 75 | `experiments/J4_speed/results` |
| J5 | `SCI_J5_NETSTACK_20261118` | `J5_PROTOCOL_FREEZE.md` | `J5_FREEZE_HASHES.json` (981b567d3523ad2c) | 120 replays | `experiments/J5_network_stack/results` |
| J5b | `SCI_J5B_PACE75_A1_20261118` | `J5B_PROTOCOL_FREEZE.md`, `J5B_AMENDMENT_A1.md` | `J5B_FREEZE_HASHES.json` (c14886b7b501d5f2), `J5B_A1_FREEZE_HASHES.json` (30af302ec633a020) | 40 (+20 exploratory) | `experiments/J5b_pacing/results` |

The J3 thresholds come from `analysis/exploratory/dswitch_replay.py` (exploratory).

## E experiments (packages and raw results on Zenodo)

| Paper | Package name in the archive | Runs |
|---|---|---|
| E1, E2 | `SCI_V3I_E1_FIXED_Q1_20261030`, `SCI_E2_MATCHED_RUNNER_20261030`, `SCI_E2_ANALYSIS_PIPELINE_20261030` | 86 |
| E3 | `SCI_E3_ENVELOPE_RUNNER_20261031` | 45 |
| E3-B | `SCI_E3B_EDGE_NAV_RUNNER_20261031` | 72 |
| E5 | `SCI_E5_PED_SPEED_20261101` | 180 |
| E6 | `SCI_E7_QUEUE_DISCIPLINE_20261102` | 48 |
| E7 | `SCI_X1_SAFETY_ABLATION_20261104` | 144 |
| E8 | `SCI_X2_HARD_ENCOUNTER_20261105` and the replication `SCI_X3_LIVENESS_CONFIRM_20261106` | 200 |

Note: E6, E7 and E8 were called E7, X1 and X2/X3 while we ran them. The files and logs inside these packages use the
old names.

## Dependencies

- J1-J4 run on the simulation stack of E3-B, E5 and E6 (these packages must be in the ROS 2 workspace; the runners
  check their hashes) and use the P0-J code.
- J5 and J5b do not need the simulator. They include copies of the P0-J and J1 scheduler files (`vendor/`) and replay
  traces built from J1 runs.
