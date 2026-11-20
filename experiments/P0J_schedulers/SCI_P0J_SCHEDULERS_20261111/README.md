# SCI_P0J_SCHEDULERS_20261111 — P0-J: new uplink schedulers, unit tests, trace export, J0 replay

**Offline only.** No ROS, no Gazebo, no robot. Nothing frozen is changed. Run time: about 5–15 minutes.

## Run (on the VM)
```bash
cd ~/ros2_ws && unzip -n ~/Downloads/SCI_P0J_SCHEDULERS_20261111.zip -d ~/ros2_ws && bash SCI_P0J_SCHEDULERS_20261111/JALANKAN_P0J.sh
```
Upload the two files it writes: `~/Downloads/SCI_P0J_RESULTS_<UTC>.zip` and `~/Downloads/SCI_P0J_CONSOLE_<UTC>.log`.

## What it does
1. Checks the SHA256 of every file of this package (`P0J_FREEZE_HASHES.json`) and of the vendored frozen files
   (`frozen_deps/`, byte copies of E4 v3, E7 (= article E6), E3-B and E5 files) against their original freezes and,
   if present, against the frozen packages in `~/ros2_ws`.
2. Unit tests (`tests/test_p0j.py`):
   - T1 a\*(v,d) = the 36 cells of Table S21; v_max(a\*,d) = v.
   - T2 ledger conservation for all 15 configurations; message-level arms never interleave two messages.
   - T3 Proposition 1 (B7 peak scan age ≤ a_min(c)), Q1/Q2 × 0.13–0.21 Mbit/s × 23 phases.
   - T3b Proposition 2 (after a camera frame, scan age ≥ a_LB(c)) for B3, B5, B7, MAKL, REF.
   - T4 MAKL limits: β = 0 → same outcome as B7; β = ∞ → same as camera-first keep-last (SLO_m = ∞).
   - T5 (needs traces) the J replay reproduces the frozen E4 harness record by record and the frozen
     `E4_RESULTS_ALL.csv` values.
   - T6 CPU time per MAKL decision; T7 bridge interface (statuses, prune, record names).
3. Exports the recorded traces of E3-B, E5 and E7 (read-only copy) for J0 and T5.
4. J0 (exploratory, open-loop replay): all arms × capacity {0.10–0.25} × {8, 25 s} on the E5 C0 traces with the
   pedestrian at 0.5 m/s, plus the E6 C0 traces at 0.13 Mbit/s for calibration. Go/no-go gate: in the Q1 0.18 Mbit/s
   25 s cell, MAKL must have less violation than B3 **and** more useful camera frames than B7.

## Files
| File | Content |
|---|---|
| `j_links.py` | B4 FQ-CoDel (RFC 8290/8289), B5 max-age-first, B7 priority + keep-last, MAKL (Algorithm 1) and J0 variants; ledger wrapper; `make_jlink` (B0–B3 = frozen E6 code) |
| `j_replay.py` | E4 replay timing model + robot state (v, d) from `E3B_GRID.jsonl` + violation metric (E5 definition) |
| `j0_explore.py` | J0 grid, summaries, calibration vs E6, gate |
| `export_j0_traces.py` | read-only trace export |
| `run_p0j.py`, `JALANKAN_P0J.sh` | batch |
| `tests/` | unit tests and synthetic sources |
| `frozen_deps/` | byte copies of frozen files (hash-checked), incl. `E4_RESULTS_ALL.csv` |

## Notes for the protocol (decisions before J1)
- New arms serve whole messages (non-preemptive), as Propositions 1–2 and Algorithm 1 assume. The frozen E6 K2
  ("B2") serves packets, so a scan can pass between the two packets of a camera frame.
- L in Propositions 1–2 must be the bytes **on the wire** (payload + 64 B CDR for camera + 28 B per packet).
  The tests show both bounds are tight with these sizes.
- With camera Q1 the bridge marks frames "reliable"; the frozen B0 (SharedLink) then expires them after 1 s,
  so B0 = B1 in Q1 cells unless the J1 protocol says otherwise.
