# J4 protocol (FROZEN) — Does MAKL's speed term matter?

Package `SCI_J4_SPEED_ABLATION_20261117`. This is draft v3 of 17 Nov 2026 (other chat) with the changes in §10. It is frozen
together with the runner, metrics and analysis (`J4_FREEZE_HASHES.json`). The pilots in §4 run under this freeze. If a
pilot fails outside the pre-declared fallback, J4 halts, and an amendment is written before any counted run.

Rules as in J1–J3:
- simulation only, with the safety layer ON in every run;
- failures after goal acceptance are data;
- no p-values;
- every hypothesis is reported, including the ones that fail;
- anything not fixed here is exploratory.

## 1. Questions
- **Q-speed:** does the speed term in a\*(v, d) change MAKL's decisions and outcomes? This is tested as MAKL vs MAKL-FixedV.
- **Q-dsw:** is MAKL better than a fixed distance switch when the robot's speed changes? This is tested as MAKL vs DSW_1.0.

Only MAKL vs MAKL-FixedV isolates the speed term. MAKL and DSW also differ in ĉ and in the peak-age prediction.

**MAKL-FixedV** is the frozen P0-J configuration `MAKL_cap` (`v_mode = 'cap'`, `v_cap = 0.15`). It runs the same code as
MAKL, with the same ĉ, peak test, m and β_min. The only difference is that it passes the constant 0.15 m/s to a\*(v, d),
where MAKL passes v̂ = max(v_odom, v_cmd). It was used in J0 only. J4 adds its name to the accepted configurations and
writes v̂ into every MAKL-type decision record. The decisions themselves are unchanged.

## 2. Definitions and predictions (stated before any run)

**d.** For MAKL, FixedV and DSW, d = minimum valid raw LiDAR range − 0.22 m. The raw range ends at the obstacle surface, so
d is about 0.10 m below the ground-truth clearance, and the raw range = d + 0.22 m exactly.

**Switching value.** With both messages waiting, MAKL sends the scan first when the predicted peak age â_peak > β, where
β = max(β_min, a\*(v, d) − m). In the archived J1 MAKL runs (Q1, 0.18 Mbit/s), the logged â_peak of contested decisions
was about 0.6–0.9 s, median about 0.68 s. This is larger than the 0.49 s of draft v3, because â_peak also contains
t − τ_sent. Solving a\*(v, d) = 0.68 + 0.1 for d (u = 0.5, b = 0.2, Δt = 0.1) gives:

| Speed level | MAKL d_sw (raw) | FixedV d_sw (raw) | DSW_1.0 (raw) | Shield threshold (raw) |
|---|---|---|---|---|
| 0.10 m/s | 0.80 m (1.02) | 1.00 m (1.22) | 1.00 m (1.22) | 0.70 m |
| 0.15 m/s | 1.00 m (1.22) | 1.00 m (1.22) | 1.00 m (1.22) | 0.77 m |
| 0.22 m/s | 1.31 m (1.53) | 1.00 m (1.22) | 1.00 m (1.22) | 0.96 m |
| 0.20 m/s (fallback only) | 1.22 m (1.44) | 1.00 m (1.22) | 1.00 m (1.22) | 0.89 m |

**Check against data.**
- In J1–J3 the robot drove at 0.15 m/s, but DWB was capped at 0.18 m/s and only the mode gate capped the robot at 0.15. MAKL's speed input therefore included the planner command before the gate: its median while moving was 0.17 m/s. For v = 0.17 the formula gives 1.09 m. The measured d_sw (rule of §6) in the five J1 MAKL runs of this cell was 1.08–1.13 m.
- This also explains the J3 design point: a 1.0 m switch nearly matched MAKL there.
- All switching values lie above the shield threshold in raw range, so the shield acts after the decisions that differ.

**Phases.** In the archived J1 runs (0.15 m/s) the closest approach to the pedestrian came 7.8–8.2 s after the trigger, and
the goal about 19 s after it.
- Before the encounter, walls set d at about 1.33–1.41 m.
- During the approach, the pedestrian sets d for about 3.5 s.
- After the encounter, walls set d at about 0.77–0.95 m.

Predicted direction by phase:

| | MAKL 0.10 | MAKL 0.15 | MAKL 0.22 | FixedV (all levels) | DSW_1.0 |
|---|---|---|---|---|---|
| before the encounter (walls, d ≈ 1.33–1.41) | camera | camera | **mixed** (switching value 1.31) | camera | camera |
| approach (pedestrian) | switch at 0.80 | switch at 1.00 | switch at 1.31 | switch at 1.00 | switch at 1.00 |
| after the encounter (walls, d ≈ 0.8–0.95) | **mostly camera** | scan first | scan first | scan first | scan first |

**Expected size, stated honestly.**
- At 0.15 m/s all three switching arms switch near 1.0 m, so no difference is expected there.
- At 0.10 m/s MAKL should deliver more frames than FixedV and DSW, both in the approach and after the encounter (walls).
- At 0.22 m/s MAKL switches 0.31 m earlier. This should give less violation in phase A, but also fewer frames, because MAKL may already send scans first near the walls before the encounter.
- The violation differences come from a window of about 1–2 s, so they may stay below the J tolerance (0.8 s).
- A frame result must not be presented as protection against the pedestrian.

## 3. Stack (declared edits only)
J4 runs the J3 stack: ideal link, camera Q1, 0.18 Mbit/s for 25 s from the trigger, and a pedestrian crossing at 0.50 m/s with a
1.0 s hold. The only changes are listed below. Each is made on a generated copy or at launch, by the same mechanism E8 used
for its 0.22 m/s cap.
1. `j4_run_trial.py` = J3 `j3_run_trial.py` + the declared edits in `tests/j4_run_trial_edits.py` (byte check in the offline tests).
2. Speed level V: DWB `max_vel_x` = `max_speed_xy` = V (`j4_stack.apply_speed`; it was 0.18 in J1–J3); mode-gate NORMAL cap = V (`j4_mode_gate.py`; it was 0.15); hard-guard command cap MAX_LIN = max(0.18, V) (`j4_guard.py`). The braking threshold is unchanged and already depends on the measured speed. As a result, v_cmd ≤ V and v̂ follows the robot's speed.
3. Bridge: `j4_bridge.py` = the frozen J1 launcher with the J4 link factory (`j4_links.py`: J3 factory + `MAKL_cap` + v̂ in the decision log).
4. Pedestrian start: the E5/J1 'meet' rule, unchanged. It uses the robot's measured speed, so it adapts to V. The only change is the shift, x_meet = 2.40 + V·δ (J1: 2.40 + 0.15·δ), so that δ keeps meaning seconds of robot travel.
5. Network trigger: unchanged (robot x ≥ 1.30 m). W = trigger → +25 s starts at the same place on the route at every speed. The run ends at the goal, as in J1–J3.

## 4. Pilots (uncounted; under the freeze; before the gates)
At each level (0.10, 0.15, 0.22 m/s) there are two C0 runs with MAKL (δ = −0.4 and +0.4 s). Every pilot must pass P1–P6:
- **P1** the run succeeds, the speed level is applied (DWB YAML, gate and guard lines, planner command ≤ V + 0.01), and the bridge and ledger configurations are correct.
- **P2** the pedestrian reaches the path when the robot is 0.75–1.05 m before the crossing. In J1 it was 0.90–0.96 m.
- **P3** the speed reaches the switching window. From the pedestrian's start until d < D_LOW[V] (0.80 / 1.00 / 1.00 m), ≥ 80 % of the 10 Hz ticks have |v − V| ≤ 0.02 m/s (and there are ≥ 5 ticks), and the planner command is ≤ V + 0.01.
- **P4** the shield acts after the switching window. The raw range at the first safety tick is < D_LOW[V] + 0.22 m. A safety tick is one where the mode is not NORMAL or the hard guard cut the command by more than 0.02 m/s.
- **P5** no safety tick is caused by a wall, meaning the raw range is never more than 0.15 m below the pedestrian's raw-equivalent distance (clearance + 0.12 m).
- **P6** the closest approach comes ≤ 20 s after the trigger.

**Pre-declared fallback.** If a 0.22 m/s pilot fails only P3, P4 or P5 (DWB slows the robot, or the shield reacts to walls),
the top level becomes 0.20 m/s and its two pilots are run. If they pass, 0.20 m/s replaces 0.22 m/s in the gates, the matrix
and the analysis, and the report says so. Any other pilot failure halts J4.

## 5. Gates, arms and matrix
**Gates.** There are 3 gate runs. They are not counted, and all must pass. All use Q1 at 0.18 Mbit/s for 25 s:
- MAKL at 0.10 m/s, δ −0.4;
- MAKL_cap at the top level, δ +0.4;
- DSW_1.0 at the top level, δ 0.

Each gate must pass the J3 gate checks, plus: speed level applied, P3, and the speed implementation check. For that check, MAKL must use v̂ in every decision, MAKL_cap must use 0.15, and recomputing the logged choices must give the same result in ≥ 99 % of decisions.

**Arms:** B7, MAKL (J1 parameters), MAKL_cap (FixedV), DSW_1.0.
**Levels:** 0.10, 0.15 and 0.22 m/s (or the fallback 0.20).

**Blocks.** One block = one start shift δ at one level. It runs the 4 arms with congestion in rotated order, then one B7 run
without congestion (C0).

**Run count.** 5 shifts × 3 levels × (4 + 1) = **75 counted runs**. The expected time is about 4 h (≈ 84 runs including
pilots and gates).

**Halts:**
- 2 robot-moving contacts;
- hard-guard death;
- bridge/ledger mismatch;
- capacity plan mismatch;
- onboard inputs missing;
- speed level not applied;
- the speed implementation check fails in a MAKL or MAKL_cap run that moved.

A failed run stays in the denominator.

## 6. Metrics (`j4_metrics.py`)
**Times.** W = [t0, t0 + 25 s), where t0 is the trigger.
- t_ca is the grid time of the smallest ground-truth pedestrian clearance after the pedestrian starts.
- t_run = min(goal, t0 + 25 s).

**Phases.** A = [t0, t_ca). B = [t_ca, t0 + 25 s) for violation, so that A + B equals the frozen W violation, and
[t_ca, t_run) for frames.

**Primary F (`F_run`).** Useful frames per second over [t0, t_run), counted with the frozen E3 rule (a frame is useful if it
is received within 1.0 s of capture). F_A and F_B are reported separately. The 25 s value F_W is kept as a secondary measure,
for comparison with J1–J3. F is compared only within one speed level, in pairs.

**Violation.** The frozen W violation (`W_viol_anav_unom_b0.2_s`) is primary. viol_A and viol_B are reported, computed with
the frozen E5 formula.

**d_sw (per run).** Uses QUALIFYING decisions, which must be:
- in phase A;
- contested (scan and camera both waiting);
- with d set by the pedestrian, meaning |d − (clearance − 0.10)| ≤ 0.15 m.

d_sw is the largest d\* among qualifying scan-first decisions such that the window [d\* − 0.15, d\*] holds ≥ 3 qualifying
decisions, of which ≥ 50 % sent the scan first. d_sw is missing if there are fewer than 6 qualifying decisions or no such d\*.
This replaces the 0.05 m bins of draft v3: decisions are about 0.05–0.1 m apart in d, so most bins would hold one decision.
The largest d of a qualifying scan-first decision (`d_first_scan`) is also reported.

**Speed implementation (per MAKL-type run).**
- MAKL: the speed used in a\* equals v̂ in every decision.
- MAKL_cap: the speed used equals 0.15 in every decision.
- Sanity: recomputing the logged contested choices with the logged speed reproduces them in ≥ 99 % of decisions.

**Agreement.** The share of contested decisions in W, with a finite predicted peak, whose choice does not change when
recomputed with the other arm's speed input (MAKL with 0.15; MAKL_cap with v̂).

**Safety record.**
- contacts and emergency stops;
- shield time (safety ticks × 0.1 s);
- safety ticks caused by walls;
- minimum clearance.

## 7. Decision rules
- **J-rule** (J1): A is clearly greater than B if at least 4 of 5 pairs exist, every paired difference A − B > 0, and the mean difference exceeds J.
  - J per level and metric = max(J1 Q1 value, range of the 5 B7 C0 runs at that level).
  - The J1 Q1 values are 0.8 s for violation (total and phase A) and 0.4 fps for F_run.
  - The J1 floors were 0.5 s / 0.2 fps; 0.8 s / 0.4 fps were the resulting J1 values for Q1.
- **Sign rule** (weaker): ≥ 4 pairs, and every paired difference has the predicted sign. It is reported with the mean and a 95 % bootstrap interval, and is never called "clearly".
- **Mechanism rule** (d_sw): ≥ 4 pairs, every paired difference has the predicted sign, and the mean difference exceeds 0.10 m.

## 8. Hypotheses and outcomes
"top" = 0.22 m/s, or 0.20 m/s if the fallback was used.

| ID | Statement | Rule |
|---|---|---|
| M1 | MAKL's d_sw at top is clearly greater than at 0.10 m/s (paired by shift) | mechanism |
| M2 | FixedV's mean paired d_sw difference (top − 0.10) is within ±0.10 m (≥ 4 pairs) | mechanism |
| S0a | Speed implementation holds in every congested MAKL / MAKL_cap run | direct (validity) |
| S0b | At 0.15 m/s, mean agreement of MAKL and FixedV runs ≥ 95 % | direct (reported) |
| S1 | At top, FixedV's violation in phase A is greater than MAKL's | J-rule and sign rule |
| S2 | At 0.10 m/s, MAKL's F_run is greater than FixedV's | J-rule and sign rule; F_A / F_B reported |
| D1 | At top, DSW_1.0's violation in phase A is greater than MAKL's | J-rule and sign rule |
| D2 | At 0.10 m/s, MAKL's F_run is greater than DSW_1.0's | J-rule and sign rule; F_A / F_B reported |
| H3 | At every level, MAKL's total violation is not clearly greater than B7's | J-rule |
| H4 | At every level, MAKL's F_run is clearly greater than B7's | J-rule (risk at top, §2) |
| H5 | No robot-moving contact | count |
| R | At 0.15 m/s, B7 and DSW_1.0 reproduce J3 (Q1, 0.18 Mbit/s) on the total violation and F_W, within max(J, 2 × J3 SD) | J-rule tolerance |

**R for MAKL.** MAKL's R is reported but not tested. At 0.15 m/s the DWB cap is now 0.15 instead of 0.18, so MAKL's speed
input is lower than in J3 (§2).

**Outcomes.**
- **Speed term:**
  - INVALID_SPEED_IMPLEMENTATION, if S0a fails;
  - SPEED_TERM_SUPPORTED = M1 ∧ M2 ∧ (S1 ∨ S2 under the J-rule);
  - SPEED_TERM_MECHANISM_ONLY = M1 ∧ M2, with S1 or S2 holding only under the sign rule;
  - otherwise SPEED_TERM_NOT_SHOWN. Before using this label, report P3–P5 in the counted runs.
  - In every case, the report states in which phase the frame differences arose.
- **Distance switch:**
  - DSW_ADVANTAGE = H3 ∧ (D1 ∨ D2 under the J-rule);
  - DSW_SIGN_ONLY = H3 ∧ (D1 ∨ D2 under the sign rule only);
  - otherwise DSW_NOT_SHOWN.
- **MAKL_MARGIN_TRADEOFF_DEGRADED** = H3 fails at some level. This means more margin violation than B7. It is a weaker trade-off, not a safety failure.
- **Safety record:** reported per arm and level. It is not a verdict on the schedulers.

**Why S0b is not part of the speed-term outcome.** Near the pedestrian the robot brakes, so v̂ < 0.15 and the two arms can
disagree even at the 0.15 m/s level. That is the speed term acting, not a fault. Validity is checked by S0a instead.

## 9. Outputs
`J4_RUNS.csv`, `J4_CELLS.csv`, `J4_TESTS.csv`, `J4_VERDICTS.json` (frozen verdicts), `J4_EXPLORATORY.json`, `FIG_J4.png`
(exploratory), and the results ZIP in `~/Downloads`. In the manuscript, Section 6.4 gets one paragraph and one panel, the
Limitations sentence on speed is replaced, and ESM S17 is added. Mechanism-only or negative results are reported with the
same weight.

## 10. Changes from draft v3 (made before the freeze)
1. **Prediction table recomputed with the measured â_peak** (≈ 0.68 s), not 0.49 s. Switching values rise by about 0.1–0.15 m, and the J1 data confirm the model (d_sw 1.08–1.13 m against 1.09 m).
2. **The J1–J3 speed input stated.** DWB was capped at 0.18 m/s, so MAKL's v̂ was about 0.17 m/s while the robot drove 0.15. In J4, DWB = V at every level. R is therefore tested on B7 and DSW_1.0 only.
3. **FixedV = the frozen P0-J configuration `MAKL_cap`.** No new decision code; only v̂ is added to the log.
4. **Trigger and pedestrian.** The network trigger is not tuned (same W start at every speed). The 'meet' rule is unchanged and adapts to the speed itself. The δ shift is V·δ. Pilots only verify (P2–P6), and the fallback is pre-declared.
5. **P3 ends at D_LOW[V]**, not at d < 0.7 m: at 0.22 m/s the mask brakes at d ≈ 0.74 m by design.
6. **d_sw uses a 0.15 m window with ≥ 3 decisions**, not 0.05 m bins with a majority. Decisions are too sparse for bins.
7. **S0 split into S0a (validity) and S0b (reported).**
8. **Runs still end at the goal (no edit).** The primary F is computed over [t0, t_run), so no post-goal time enters it.
9. **The pilots run under the freeze**, with a pre-declared fallback for the top level. Any other pilot failure halts J4 before any counted run.
