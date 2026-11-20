# J1 protocol (frozen before the first gate run) — MAKL vs seven baselines in the closed ROS 2 / Nav2 loop

Package `SCI_J1_SCHEDULERS_20261112`. Simulation only; no physical robot. Safety layer ON in every run.
Basis: P0-J (`SCI_P0J_SCHEDULERS_20261111`, all unit tests pass on the VM) and J0 v2 (exploratory open-loop replay,
gate passed; `claude/J0_EXPLORATORY_REPORT_20261112.md`, including the weighted max-age sweep).

## 1. Question
In the closed loop, does MAKL keep the age–speed violation of the strongest scan-first queue (B7: priority + keep-last)
while delivering more useful camera frames, and does it beat keep-last (B3), FQ-CoDel (B4) and max-age-first (B5) on
violation?

## 2. Stack (unchanged)
- E5/E6 stack: P0 fixes (AMCL tolerance 1.0 s), architecture B (planner costmaps on the uplinked scan; collision
  monitor and hard guard on the raw scan), E5 crossing at x = 3.2 m with the locked calibration, pedestrian 0.50 m/s,
  NORMAL mode (0.15 m/s), E5 trial process `e5_trial.py`.
- `j1_run_trial.py` = E7 `e7_run_trial.py` + declared edits (byte-checked).
- Only the uplink scheduler changes (`j1_bridge.py`).
- Capacity plans: `e3b_policy_logic.plan(level)` with the event duration set to 8 or 25 s. The frozen bridge rules
  accept both durations. C0 uses the unchanged `plan(C0)`.

## 3. Arms
| Arm | Scheduler | Code |
|---|---|---|
| B0 | shared FIFO (frozen SharedLink) | E6 K0 |
| B1 | FIFO + 1 s camera expiry | E6 K1 |
| B2 | strict scan priority (packet level) | E6 K2 |
| B3 | per-stream keep-last in a shared FIFO | E6 K3 |
| B4 | FQ-CoDel (RFC 8290 / 8289; 5 ms, 100 ms; quantum 1514 B) | P0-J |
| B5 | max-age-first (sender-side edge age), keep-last | P0-J |
| B7 | strict scan priority + keep-last, whole messages | P0-J |
| MAKL | Algorithm 1 with the P0-J defaults | P0-J |

MAKL settings (J0 decision):
- peak-age test;
- u = 0.5 m/s, b = 0.2 m/s², Δt = 0.1 s, m = 0.1 s, β_min = 0.2 s, SLO_m = 1.0 s;
- v = max(v_odom, v_cmd);
- d = raw 360° /scan minimum − 0.22 m;
- ĉ = delivered wire rate over 2 s.

Notes:
- With camera Q1 the frames are "reliable" (1 s lifespan) in the frozen bridge, so **B0 and B1 coincide in the Q1
  cells**. Both are run (balanced design), and this is stated in the paper.
- The weighted max-age arm is **not** included: no fixed weight reached MAKL's point in any of the 8 J0 cells. The
  sweep is reported as exploratory.

## 4. Matrix
- Cells (6):
  - C0-Q2 and C0-Q1 (3 Mbit/s);
  - LC-Q2-25s and LC-Q2-8s (0.13 Mbit/s, camera Q2 = F-N2);
  - LB-Q1-25s and LB-Q1-8s (0.18 Mbit/s, camera Q1 = F-N1).
- Paired start shifts δ = −0.4, −0.2, 0, +0.2, +0.4 s (rep 1–5), as in X1–X3.
- 8 arms × 6 cells × 5 = **240 counted runs**.
- Block = one δ × cell with all 8 arms. The arm order is rotated by one position per block (30 blocks, 8 rotations).
- The 8 s cells were decided before any J1 data, because short events are realistic (brief Wi-Fi drops). Every cell is
  reported.

## 5. Gate (not counted; both must pass, else halt)
- G1: MAKL, LC-Q2-25s, δ = −0.4 s.
- G2: MAKL, LB-Q1-8s, δ = +0.4 s.

Checks for each gate run:
1. the run SUCCEEDED;
2. congestion boundary at the trigger with the cell's capacity, and recovery exactly 25 / 8 s later;
3. a_nav logged;
4. pedestrian arrival within 0.15 s;
5. bridge line and ledger header = MAKL;
6. MAKL inputs received (odometry > 0, Nav2 command > 0, decisions with v > 0);
7. at least 20 MAKL decisions in W, with both streams chosen at least once;
8. CPU time logged.

## 6. Rules and halts
- Failures after goal acceptance are data: never retried, never excluded.
- Pre-motion blocks: up to 3 attempts.
- Contacts and emergency stops are outcomes.

Halts (the batch stops and the reason is written to STATE):
- 2 robot-moving contacts in total;
- hard-guard death;
- bridge or ledger configuration mismatch (B0–B3 ledgers are named K0–K3);
- capacity plan mismatch;
- MAKL inputs missing in a MAKL run that moved;
- a failed gate.

## 7. Metrics
- Window W = trigger → +25 s, for 8 s events too (E5 definition).
- **Primary:**
  - age–speed violation time `W_viol_anav_unom_b0.2_s` (nominal u = 0.5 m/s, b = 0.2 m/s², d = ground-truth
    clearance; frozen E5 code);
  - useful camera frames per second `F_fps` and stale-camera share `V` (frozen E3-B analysis);
  - a_nav p95 (full W; frozen) and a_nav p95 clipped at the run end (X1 definition).
- **Secondary:**
  - violation with the actual u and with b = 0.5 and 1.0;
  - P1, minimum clearance in W, contacts, emergency stops;
  - ledger counts per stream and status (replaced, dropped_codel, dropped_slo, expired);
  - MAKL decisions (camera share, contested camera share, median β);
  - CPU time per MAKL decision;
  - measured event duration.

## 8. Decision rule (frozen; no p-values)
- Paired "clear difference": pairs = same δ. A is CLEARLY GREATER than B if all of these hold:
  - at least 4 pairs exist;
  - every paired difference A − B > 0;
  - the mean difference > J.
- J (per metric and camera profile) = max(floor, range over all C0 runs of that profile).
- Floors: violation 0.5 s, camera F 0.2 fps, P1 1.0 s, clearance 0.02 m.
- Non-inferiority margin (MAKL vs B7 violation) = max(0.3 s, C0 range), as asked by the review.
- Mean ± SD and 95 % bootstrap CIs (10,000 resamples, seed 20261112) are reported as description.

## 9. Hypotheses (from J0 effect sizes) and outcome classes
| ID | Statement | Cells | J0 expectation |
|---|---|---|---|
| H1 | MAKL violation is NOT clearly greater than B7 (margin max(0.3 s, range)) | all 4 congested | difference ±0.03 s |
| H1b | B3 violation clearly greater than MAKL | LC-Q2-25s, LB-Q1-25s | +0.67 / +1.37 s |
| H2a | MAKL camera F clearly greater than B7 | LB-Q1-25s, LB-Q1-8s | +0.64 / +0.48 fps |
| H2b | same, Q2 (secondary; small expected gain) | LC-Q2-25s, LC-Q2-8s | +0.28 / +0.28 fps |
| H3 | B4 and B5 violation each clearly greater than MAKL | LC-Q2-25s, LB-Q1-25s | B4 +3.1 / +2.1 s; B5 +0.67 / +0.93 s |
| H4a | no robot-moving contact in MAKL runs | all | 0 |
| H4b | MAKL P1 and minimum clearance not clearly different from MAKL at C0 (same profile) | 4 congested | — |
| R | at LC-Q2-25s, B0–B3 a_nav p95 within ±0.5 s of E6 (4.67, 2.03, 0.81, 0.76 s) | anchor | replay 4.68 / 1.30 / 0.67 / 0.80 |
| P (check) | B7 clipped a_nav p95 ≤ a_min(c) + pipeline offset (B7 at C0 − a_min(3 Mbit/s)), measured frame sizes | 25 s cells | — |

**Outcome:**
- **MAKL_SUPPORTED** = H1 and H2a pass;
- **MAKL_PARTIALLY_SUPPORTED** = only one of them passes;
- **MAKL_NOT_SUPPORTED** = neither passes.

Frozen verdicts stand when they fail; they are diagnosed and reported, never "fixed".

Expected risk (stated now): J0 under-predicted B3's closed-loop violation by about 0.5 s and is open loop, so the
MAKL–B7 difference can differ in J1. H2b may fail (small absolute gain at 0.13 Mbit/s).

## 10. Exploratory (labelled as such)
- % changes vs B0 and B7;
- the Pareto figure (violation vs camera F per cell);
- the 8 s vs 25 s comparison;
- the weighted max-age sweep (J0);
- anything not listed above.

## 11. Run time
About 140 s per run on the VM (E5/E6 mean) → 242 runs ≈ 9.5–10 h. Resumable.
