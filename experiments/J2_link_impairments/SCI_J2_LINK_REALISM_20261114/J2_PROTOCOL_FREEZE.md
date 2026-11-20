# J2 protocol (frozen before the first gate run) — Is the J1 result robust to jitter, loss and varying capacity?

Package `SCI_J2_LINK_REALISM_20261114`. Simulation only; no physical robot. Safety layer ON in every run.
Basis: J1 v2 (`SCI_J1_SCHEDULERS_V2_20261113`, outcome MAKL_SUPPORTED, 240/240 runs SUCCEEDED, 0 contacts;
`claude/J1_FINAL_REPORT_20261114.md`). No DDQN retraining. No scheduler parameter is changed.

## 1. Question
J1 used an ideal link: fixed capacity, fixed 20 ms delay, no loss. A reviewer can ask whether MAKL's result holds on
a less ideal link. J2 asks: under delay jitter, random loss, bursty loss and a capacity that changes over time,
does MAKL still (a) keep the age–speed violation of B7 (priority + keep-last) and (b) deliver more useful camera
frames than B7, without being clearly beaten on both axes by another scheduler?

## 2. Stack (unchanged from J1)
- E5/E6/J1 stack: P0 fixes, architecture B, E5 crossing at x = 3.2 m with the locked calibration, pedestrian
  0.50 m/s, NORMAL mode (0.15 m/s).
- `j2_run_trial.py` = J1 v2 `j1_run_trial.py` + declared edits (byte-checked in the tests). The edits only pass
  the impairment condition and seed to the bridge and start `j2_bridge.py`.
- `j2_bridge.py` = the J1 bridge (`j1_bridge.py`, unchanged) whose link is wrapped by `j2_impair.impair()`.
  It prints one `J2_IMPAIR` line (condition, seed) and a `J2_STATE` line every 20 scans.
- Schedulers, MAKL parameters, capacity plans, metrics and the J1 analysis code are unchanged
  (frozen packages pinned by hash: E5 358daffd, E7 a0cc9ef7, P0-J e5bccfc2, J1 v2 befd9065).

## 3. Link impairments (`j2_impair.py`)
Applied in the same way to every arm, after the scheduler has sent a packet. With all impairments off the link is
identical to the J1 link, record by record (unit-tested for all 6 arms).

| Condition | What changes | Parameters |
|---|---|---|
| none | nothing (J1 link) | — |
| jit10 | one-way delay jitter | offset U(−10, +10) ms per packet |
| jit50 | one-way delay jitter | offset U(−50, +50) ms per packet |
| loss1 | random packet loss | Bernoulli, p = 1 % per packet |
| loss5 | random packet loss | Bernoulli, p = 5 % per packet |
| ge | bursty packet loss | Gilbert–Elliott in time: good/bad periods with exponential lengths, means 3.75 s / 0.25 s (bad 6.25 % of the time); loss probability 0.5 in the bad state, 0 in the good state (mean loss 3.1 %) |
| capsq | capacity varies | square wave while congested: nominal × (1 ± 0.07/0.13), switching every 1 s, starting high at the congestion boundary; the mean equals the nominal value (0.13 Mbit/s → 0.20 / 0.06 Mbit/s; 0.18 Mbit/s → 0.277 / 0.083 Mbit/s) |

Details:
- **Jitter:** arrival = max(previous arrival, end of transmission + 20 ms + max(offset, −20 ms)). So the one-way
  delay stays in [0, 20 ms + J] and packets stay in order (no reordering on one wire).
- **Loss:** a lost packet still used link time. A message with a lost packet ends as `channel_loss` when its last
  packet would have arrived. There is no retransmission (MAC retransmissions are assumed inside the residual loss
  rate).
- **Capacity:** only the serializer uses the varying value. The bridge still reads and logs the nominal value, so
  its phase events and the capacity-plan checks are unchanged. MAKL's capacity estimate ĉ is the delivered wire rate,
  so it does see the variation.
- **Common random numbers:** each random draw is a hash of (run seed, purpose, stream, message stamp, packet index).
  The Gilbert–Elliott timeline is drawn from the run seed alone. So a packet's draw does not depend on how much
  other traffic an arm sent, and all arms of one block see the same channel (unit-tested).
- **Seed** = SHA-256 of (condition, start shift, camera profile): the same for all 6 arms of a block.

## 4. Arms
B2 (strict scan priority, E6 K2), B3 (keep-last, E6 K3), B4 (FQ-CoDel), B5 (max-age-first), B7 (priority +
keep-last), MAKL — exactly as in J1. B0 and B1 are left out: J1 already showed they are far worse (FIFO queues
of several seconds).

## 5. Matrix (252 counted runs)
6 arms × 7 conditions × 2 cells × 3 paired start shifts:
- cells: **LC-Q2** (0.13 Mbit/s, camera Q2) and **LB-Q1** (0.18 Mbit/s, camera Q1), both with 25 s congestion events;
- start shifts: δ = −0.4, 0, +0.4 s (the J1 pairing; pairs = same δ);
- block = one δ × cell × condition with all 6 arms and one seed; arm order rotated per block (6 rotations).

## 6. Gate (not counted; both must pass, else halt)
- G1: MAKL, LC-Q2, ge, δ = −0.4 s.
- G2: MAKL, LB-Q1, capsq, δ = +0.4 s.

Checks for each gate run:
1. the run SUCCEEDED;
2. capacity plan as in J1 (boundary at the trigger, nominal value, recovery 25 s later or run end);
3. a_nav logged; pedestrian arrival within 0.15 s;
4. bridge line and ledger header = MAKL; MAKL inputs received;
5. at least 20 MAKL decisions in W, with both streams chosen;
6. the `J2_IMPAIR` line matches the condition and the seed;
7. the impairment is active: G1 lost packets > 0 and `channel_loss` in the ledger; G2 at least 2 capacity changes.

## 7. Rules and halts
As J1: failures after goal acceptance are data (never retried, never excluded); pre-motion blocks up to 3 attempts;
contacts and emergency stops are outcomes.
Halts: 2 robot-moving contacts; hard-guard death; bridge/ledger configuration mismatch (B2, B3 ledgers named K2, K3);
capacity plan mismatch; MAKL inputs missing in a MAKL run that moved; impairment line missing or not matching the
condition/seed; a failed gate.

## 8. Metrics
Same as J1 (frozen J1 `per_run`):
- primary: age–speed violation time `W_viol_anav_unom_b0.2_s`; useful camera frames per second F;
- secondary: V, a_nav p95 (full and clipped), P1, minimum clearance, contacts, ledger counts;
- J2 additions: `channel_loss` messages per stream in W; measured packet loss rate, mean |jitter| and capacity
  changes (from the bridge log);
- per cell: mean, SD and **range** of every metric (run-to-run stability; descriptive).

## 9. Decision rule (frozen; no p-values)
- 3 pairs (same δ). A is CLEARLY GREATER than B if all 3 pairs exist, every paired difference A − B > 0, and the
  mean difference > J.
- J is fixed from J1 (J2 has no C0 cell):
  - violation: 0.7 s (LC-Q2), 0.8 s (LB-Q1) — the J1 non-inferiority margins, max(0.3 s, C0 range);
  - camera F: 0.2 fps (LC-Q2), 0.4 fps (LB-Q1).

## 10. Hypotheses and outcome
The 12 impaired cells = 2 camera cells × 6 impaired conditions.

| ID | Statement | Cells |
|---|---|---|
| H1 | MAKL violation is NOT clearly greater than B7 (non-inferiority, margin J_viol) | all 12 |
| H2 | MAKL camera F clearly greater than B7 | all 12 |
| H3 | MAKL is not clearly dominated: no arm has mean violation < MAKL − J_viol AND mean F > MAKL + J_F | all 12 |
| H4 | no robot-moving contact | all runs |
| R | 'none' reproduces J1: each arm's mean violation and F within max(J, 2 × J1 SD) of J1 | 2 cells × 6 arms |

**Outcome:**
- **ROBUST** = H1, H2 and H3 pass in all 12 cells;
- **PARTIALLY_ROBUST** = H1 and H3 pass in all 12, H2 does not;
- **NOT_ROBUST** = otherwise.

Per-cell verdicts are reported in full. Frozen verdicts stand when they fail; they are diagnosed and reported,
never "fixed". If R fails, the J2 numbers are reported with that warning.

Expected risk (stated now):
- H1 at LC-Q2 is the weakest point: in J1 the MAKL − B7 violation difference was +0.34 s (range −0.04 to +0.70 s)
  against a 0.7 s margin. With 3 pairs per cell and 6 conditions, one cell can fail by chance.
- Loss removes camera frames from every arm, so the absolute F gain at LC-Q2 (J1: +0.47 fps) may shrink.
- J2 is still an emulator: it adds link effects, not a real radio or a real DDS stack.

## 11. Exploratory (labelled as such)
- Kendall τ between the arm ranking under each condition and under 'none' (violation and F);
- the Pareto figure per condition and cell (FIG_J2_PARETO.png);
- % changes against B7;
- anything not listed above.

## 12. Run time
About 140 s per run on the VM (J1 mean) → 254 runs (2 gates + 252) ≈ 10 h. Resumable.
