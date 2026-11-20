# J3 protocol (frozen before the first gate run) — Does MAKL need its state-dependent budget, or is a distance switch enough?

Package `SCI_J3_SWITCH_ABLATION_20261116`. Simulation only; no physical robot. Safety layer ON in every run.
Basis: J1 v2 (outcome MAKL_SUPPORTED), J2 (PARTIALLY_ROBUST) and the exploratory replay
`claude/ABLATION_REPLAY_RESULTS_20261116.md`. No DDQN retraining. MAKL and B7 are unchanged.

## 1. Question
Fig. 5(e) of the manuscript shows MAKL behaving like priority + keep-last (B7) near the pedestrian and like keep-last far
from it. A reviewer can ask: would a simple switch at a fixed distance do the same?

The replay (exploratory, open loop) found:
- a fixed age budget does not reach MAKL;
- a distance switch at 1.0 m comes close to MAKL at the design point;
- a threshold that is too small (0.8 m) violates much longer when the capacity drops, and a threshold that is too
  large (1.2 m) gives fewer camera frames.

J3 tests this in the closed loop, at the design point and at a lower capacity.

## 2. Stack (unchanged from J1)
- E5/E6/J1 stack: P0 fixes, architecture B, E5 crossing at x = 3.2 m with the locked calibration (x_meet = 2.4 m),
  pedestrian 0.50 m/s, NORMAL speed cap (0.15 m/s), ideal link (no jitter or loss), 25 s congestion events.
- `j3_run_trial.py` = J1 v2 `j1_run_trial.py` + declared edits (byte-checked in the tests). The edits only start
  `j3_bridge.py` and accept the J3 arm names.
- `j3_bridge.py` = the frozen J1 bridge (`j1_bridge.py`, imported unchanged) with the link factory extended by
  `j3_links.py`. B7 and MAKL are built by the unchanged J1 factory.
- Frozen packages are pinned by hash: E5 358daffd, E7 a0cc9ef7, P0-J e5bccfc2, J1 v2 befd9065.

## 3. Arms
| Arm | Rule |
|---|---|
| B7 | priority + keep-last (J1) |
| MAKL | Algorithm 1, J1 parameters (u = 0.5 m/s, b = 0.2 m s⁻², Δt = 0.1 s, m = 0.1 s, β_min = 0.2 s, SLO 1 s) |
| DSW_0.8 / DSW_1.0 / DSW_1.2 | distance switch: depth-1 queues, probes first; if d < threshold (m) scan first (as B7), else the older head first (keep-last order) |

- d for the switch is the same input as MAKL: the minimum raw LiDAR range − 0.22 m, supplied by the J1 bridge.
- The switch uses no speed input and no capacity input.
- The thresholds were chosen from the replay before this freeze. 1.0 m is the best fixed threshold there; 0.8 and
  1.2 m are ±0.2 m.

## 4. Matrix (75 counted runs)
5 arms × 3 cells × 5 paired start shifts (δ = −0.4, −0.2, 0, +0.2, +0.4 s, as J1):

| Cell | Camera | Capacity in the event | Role |
|---|---|---|---|
| LB-Q1 | Q1 | 0.18 Mbit/s | design point (as J1) |
| LC-Q1 | Q1 | 0.13 Mbit/s | lower capacity (new for MAKL) |
| LC-Q2 | Q2 | 0.13 Mbit/s | J1 main cell |

Block = one δ × cell with all 5 arms; arm order rotated per block.

## 5. Gate (not counted; both must pass, else halt)
- G1: DSW_0.8, LC-Q1, δ = −0.4 s.
- G2: MAKL, LC-Q1, δ = +0.4 s.

Checks for each gate run:
1. the run SUCCEEDED;
2. capacity plan as in J1;
3. a_nav logged; pedestrian arrival within 0.15 s;
4. bridge line and ledger header = the arm;
5. onboard inputs received (odometry, command and scan-d counts > 0);
6. at least 20 decisions in W with both streams chosen;
7. for DSW, the logged threshold equals the arm's threshold.

## 6. Rules and halts
As J1: failures after goal acceptance are data (never retried, never excluded); pre-motion blocks up to 3 attempts;
contacts and emergency stops are outcomes.

Halts:
- 2 robot-moving contacts;
- hard-guard death;
- bridge/ledger configuration mismatch;
- capacity plan mismatch;
- onboard inputs missing in a MAKL or DSW run that moved;
- a failed gate.

## 7. Metrics
- As J1 (frozen J1 `per_run`): primary are violation time `W_viol_anav_unom_b0.2_s` and useful camera frames per
  second F; secondary are V, a_nav p95, P1, clearance and contacts.
- New for DSW: share of decisions in W with d < threshold.
- Per cell: mean, SD, range and 95 % bootstrap interval.

## 8. Decision rule (frozen; no p-values)
- J1 paired rule: A is CLEARLY GREATER than B if at least 4 of the 5 pairs exist, every paired difference A − B > 0,
  and the mean difference > J.
- Thresholds are fixed from J1 (J3 has no C0 cell):
  - violation J: 0.8 s in the Q1 cells, 0.7 s in the Q2 cell;
  - frames J: 0.4 fps in the Q1 cells, 0.2 fps in the Q2 cell.

## 9. Hypotheses
| ID | Statement |
|---|---|
| H1 | At LC-Q1, MAKL's F is clearly greater than B7's, and MAKL's violation is not clearly greater than B7's |
| H2a | At LC-Q1, DSW_0.8's violation is clearly greater than MAKL's (a threshold that is too small fails at low capacity) |
| H2b | In at least one Q1 cell (LB-Q1 or LC-Q1), MAKL's F is clearly greater than DSW_1.2's (a threshold that is too large costs frames) |
| H3 | In all 3 cells, the tuned switch DSW_1.0 is not clearly better than MAKL: MAKL's violation is not clearly greater than DSW_1.0's, and DSW_1.0's F is not clearly greater than MAKL's |
| H4 | no robot-moving contact |
| R | B7 and MAKL at LB-Q1 and LC-Q2 reproduce J1 (mean within max(J, 2 × J1 SD)) |

## 10. Outcome
- **SUPPORTED** = H3 and (H2a or H2b). MAKL is as good as the best tuned switch, and a mistuned switch fails
  somewhere.
- **TUNED_SWITCH_BETTER** = H3 fails.
- **THRESHOLD_INSENSITIVE** = H3 holds, but neither H2a nor H2b. The switch threshold does not matter much here.

All per-cell results are reported. Frozen verdicts stand when they fail.

**What SUPPORTED would and would not show.**
- It would show that MAKL reaches the performance of a distance switch tuned on recorded traces, without that
  tuning, and that ±0.2 m of tuning error can cost safety margin or frames.
- It would **not** show that MAKL beats every simple rule.

Expected risk, stated now from the replay:
- H3 at LC-Q2: in the replay, DSW_1.0 gave 0.1–0.2 fps more frames than MAKL (J = 0.2), so H3 may fail there.
- H2b is borderline in the replay (−0.31 and −0.43 fps against J = 0.4).
- Speed is not varied. MAKL's speed input includes the planner's command before the mode gate, so a SLOW mode cap is
  not visible to it. A speed test needs a different planner speed limit and is left out.

## 11. Exploratory (labelled as such)
- MAKL − DSW_1.0 mean differences;
- the Pareto reading by means (FIG_J3_PARETO.png);
- bootstrap intervals and % changes;
- anything else.

## 12. Run time
About 140 s per run (J1/J2 mean) → 77 runs (2 gates + 75) ≈ 3 h. Resumable.
