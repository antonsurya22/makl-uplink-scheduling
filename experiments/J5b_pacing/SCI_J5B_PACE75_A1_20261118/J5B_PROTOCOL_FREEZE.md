# J5b protocol (FROZEN) — Does a pace that includes the overhead below the scheduler give MAKL its control back?

Status: frozen with the package `SCI_J5B_PACE75_20261118` on 18 Nov 2026 (SHA256 of every file in
`J5B_FREEZE_HASHES.json`). J5b follows J5 (`claude/J5_PROTOCOL_FREEZE_20261118.md`, results
`claude/J5_FINAL_REPORT_20261118.md`). Rules as in J1–J5:
- every hypothesis is reported, failed or not;
- no p-values;
- anything not fixed here is exploratory;
- frozen files are never edited; a change after the freeze is a declared amendment.

## 1. Why J5b
In J5, with the capacity limit c in the kernel (a token bucket on `wlan0`), pacing the scheduler at 0.9 c gave part of
MAKL's control back (P1), but not all of it (P2 failed in both cells). The reason was found after the analysis:
- the bytes entering the kernel queue were 1.16–1.23 × the bytes the scheduler counts;
- about 10 kbit/s of that is background traffic (6–8 % of c); the rest is framing beyond the model's 28 B per packet;
- so the scheduler, paced at 0.9 c, still offered 1.02 c (Q1) and 1.07 c (Q2) to the kernel, and the kernel queue grew.

J5b tests whether a pace that leaves room for this overhead keeps the kernel queue short and gives the control back.

**What J5b does not test.** The pace 0.75 c is chosen from the overhead measured in J5 and the known c. A robot in the
field would have to estimate both. J5b tests the remedy (pace with the overhead included), not how a robot finds its
pace. That stays a limitation of the paper.

## 2. What changes from J5 — one constant
- The J5 package is vendored **unchanged** (`vendor/SCI_J5_NETSTACK_20261118`, freeze file 981b567d; checked by both
  scripts and by the offline tests, which also compare it byte by byte with the J5 package installed on each host).
- **The one changed constant:** the pace of mode NP is **0.75 c** (J5: 0.9 c), in the scheduler's own byte count:
  - Q1: link model at 135 kbit/s, kernel token bucket at 180 kbit/s during W;
  - Q2: link model at 97.5 kbit/s, kernel token bucket at 130 kbit/s during W.
- Everything else is the J5 code: schedulers, link model, replay loop, traces, DDS transport and QoS, tc settings,
  session checks, pilots S2–S6, block validity and repeat rule.
- J5b wrappers add only:
  - the matrix (§4);
  - a versions check against the J5 lock (§5);
  - a background pilot (§5);
  - the J5b analysis (§7).

**Assumption (A1) of Proposition 1 in Q1:** one scan needs 8 × 3,000 / 135,000 = 0.178 s, less than T_s = 0.2 s, so (A1)
holds. **In Q2 it fails:** 97.5 kbit/s is below the scan load of 120 kbit/s (one scan needs 0.246 s).

## 3. Setup
As J5 (same hosts, router, channel 9, BSSID 5c:e9:31:85:b4:1b, power save off, CycloneDDS domain 77, unicast
discovery, chrony with the VM as server, no `apt upgrade` since J5).

To keep background traffic low, after the start the tmux session on the Pi is detached, so that no screen output
travels over Wi-Fi. This is recommended, not checked.

## 4. Matrix
- **Counted (Q1, c = 0.18 Mbit/s):** modes A (limit c in the scheduler, as J5) and NP75 (limit c in the kernel,
  scheduler paced at 0.75 c), 4 arms (B0, B3, B7, MAKL) × 5 traces (J1 C0 B7 runs, rep 1–5, as J5).
  - 40 replays in 10 blocks; one block = one trace × one mode with all 4 arms.
  - Order: rep 1 → 5; mode order A, NP75 for odd reps and NP75, A for even reps; arm order rotated by block.
  - Mode A is repeated in this session, so that NP75 − A is paired within one session.
- **Demonstration (Q2, c = 0.13 Mbit/s, exploratory):** NP75 only, 4 arms × 5 traces = 20 replays in 5 blocks, after
  the counted blocks. It shows what happens on a real stack when (A1) fails. No hypothesis; reported in the ESM only.
- Pilots (uncounted): the four J5 pilots (S2–S6, same checks and limits as J5), then iperf3, then the background pilot.
- Time: about 41 s per replay; about 50 min in total.

## 5. Checks added to J5
- **Versions = J5 (before LOCK):** the system versions (kernel and packages) must equal those recorded at the J5 lock
  (`~/ros2_ws/results/SCI_J5_NETSTACK_20261118/J5_MAIN/STATE.json` on the Pi). Otherwise the run stops before LOCK
  (`J5_NOT_STARTED`), with nothing counted.
- **Background pilot (after the J5 pilots, before any counted replay, not counted):** 60 s with all nodes running and
  nothing replayed.
  - Recorded: the `wlan0` qdisc byte rate (mean, and the 90th percentile and maximum over 2 s windows), and the change
    of the kernel UDP datagram and TCP segment counters.
  - It does not stop the run. It is recorded before the counted replays, so it cannot be tuned to the results.
  - **Expectation:** about 10 kbit/s, as in J5. The kernel load of about 0.88 c (Q1) already includes this. If the
    background is much higher (more than about 20 kbit/s, i.e. the remaining 0.12 c), the headroom is gone and K1 is
    expected to fail.

## 6. Predictions (frozen)
**Link-model predictions.** `predictions/J5B_PREDICTIONS.json` holds the predictions for every arm and trace: the frozen
J5 replay core with a virtual clock (no network), the same traces, the same settings. The offline test recomputes the
file byte for byte. J5 showed that the scheduler on the Pi reproduces this replay exactly (80 of 80 replays). If the
network adds no queue, the edge must see these ages plus a few milliseconds.

**Predicted means over the 5 traces (Q1; scan age at the link, p95):**

| Arm | A: age (s) / F (fps) / violation (s) | NP75: age / F / violation | NP75 − A, age (per trace) |
|---|---|---|---|
| B0 | 1.286 / 0.54 / 13.1 | 1.353 / 0.19 / 13.3 | +0.067 (+0.065 to +0.068) |
| B3 | 0.618 / 2.68 / 2.0 | 1.192 / 1.58 / 4.3 | +0.574 (+0.09 to +1.30) |
| B7 | 0.519 / 2.26 / 1.3 | 0.620 / 0.73 / 1.6 | +0.101 (+0.093 to +0.111) |
| MAKL | 0.709 / 3.07 / 1.3 | 0.906 / 1.54 / 1.7 | **+0.196** (+0.106 to +0.271) |

Notes:
- For B7 the predicted rise (+0.10 s) is the shift of Proposition 1's bound, 8(L_m + L_s)/c · (1/0.75 − 1) = +0.091 s.
- MAKL's age rises more (+0.20 s), because a slower link changes its decisions: its budget test sees a lower ĉ, and it
  still spends its margin on camera frames.
- **F in NP75 is not near zero:** keep-last arms replace waiting scans, so B3 and MAKL still send 1.5–1.6 fps (B7
  0.73 fps). This replaces the rough estimate "F ≤ 0.5 fps for every arm" made before this replay.

**Kernel load.** With the J5 overhead (1.16–1.23 × the scheduler's bytes, background included), NP75 offers about
0.88 c (Q1) and 0.92 c (Q2) to the kernel.

**Q2 demonstration (exploratory, predicted by the same replay).** NP75 at 97.5 kbit/s is below the scan load:
- scans replace each other (keep-last arms); scan age at the link, p95: B7 0.74 s, MAKL 0.84 s, B3 0.99 s;
- B0's FIFO grows through W (p95 9.6 s);
- F: B3 1.58, MAKL 0.36, B7 0.02, B0 0.06 fps;
- the kernel queue stays short (load about 0.92 c).

## 7. Hypotheses and tests (Q1, counted; pairs = traces rep 1–5 in this session)
J_age = max(0.1 s, range over traces of B7's p95 a_nav in mode A of this session). J for violation 0.8 s (as J1/J5).
The J1 rule ("clearly greater": at least 4 of 5 pairs, all paired differences > 0, mean > J) is used where stated.

| ID | Statement (test in `j5b_analysis.py`) | Prediction |
|---|---|---|
| K1 | Kernel queue short: for every arm in NP75, the median kernel backlog in W is below 3,000 B in at least 4 of 5 replays | holds (expected share of W with > 3,000 B queued below 0.10; reported) |
| K2 | Network delay small: MAKL in NP75, median over traces of the per-replay p99 of (end of transmission in the scheduler → edge callback) ≤ 50 ms | holds |
| K3 | The edge follows the link model: for every arm in NP75, mean over traces of (measured p95 a_nav − predicted p95 link age) ≤ J_age | holds |
| **R1** | **Age restored (primary):** MAKL, mean paired (NP75 − A) of p95 a_nav ≤ predicted mean (NP75 − A) + J_age = 0.196 s + J_age, with ≥ 4 pairs | holds |
| R2 | Violation: MAKL's violation in NP75 is not clearly greater than in A (J 0.8 s; evaluable) | holds (predicted paired differences 0.7, 0.1, 0.7, 0.0, 0.4 s: not all > 0) |
| — | Reported: R1 for B7, B3 and B0; F and violation of every arm, measured against the prediction; MAKL − B7 in F in NP75; mode A of J5b against mode A of J5 (cell means, within 0.1 s and 0.4 fps); NP75 (J5b) against NP 0.9 c (J5), cross-session, exploratory | — |

**Verdict.**
- **CONTROL_RESTORED** = R1 ∧ R2: with a pace that includes the overhead below the scheduler, MAKL's age and violation
  over the real stack follow what the scheduler alone produces.
- Otherwise **CONTROL_NOT_RESTORED**.
- K1–K3 explain the verdict. If R1 fails, K1 and K3 show whether the kernel queue was the reason; the background pilot
  shows whether other traffic filled the headroom.

**J5b does not test MAKL against the other arms.** Differences in F between arms are reported, not tested. A low F in
NP75 is the cost of the pace (P3 of J5), not a failure of MAKL.

## 8. Outputs (edge VM, `~/ros2_ws/results/SCI_J5B_PACE75_20261118/J5B_MAIN/final/`)
`J5B_REPLAYS.csv`, `J5B_CELLS.csv`, `J5B_TESTS.csv`, `J5B_VERDICTS.json`, `J5B_PLATFORM.json`, `J5B_PILOTS.json`
(including the background pilot). Raw logs as J5.

## 9. Known behaviour
- **Abort on exit.** In J5 the sender process aborted on exit (rc 134) after `J5_DONE`, while ROS shut down. The J5 code
  is not changed. The J5b robot script reads the state file and reports a finished run as finished.
- **Cross-session comparisons are exploratory.** NP75 (J5b) against NP 0.9 c (J5) comes from two sessions. Only
  NP75 − A within J5b is paired and counted.

## 10. What the manuscript gets
- Section 6.6 (J5) gains one paragraph with the J5b result, and the J5 table gains one NP75 column for Q1.
- The Limitations keep the sentence that the pace was set from a measured overhead and a known c.
- ESM S18 gains J5b: the protocol, the background pilot, all tests, and the Q2 demonstration.
