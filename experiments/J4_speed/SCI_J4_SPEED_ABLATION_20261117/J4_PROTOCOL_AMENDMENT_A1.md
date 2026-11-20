# J4 protocol amendment A1 (17 Nov 2026) — pilot checks P3, P4, P5

**Status.** This amendment was written after the six pilots of J4 and before any gate or counted run. It changes three pilot
measures and nothing else: no scheduler, stack setting, hypothesis, metric of a hypothesis, decision rule, or outcome. The
package is frozen again with this file (`J4_FREEZE_HASHES.json`). The pilots are **not re-run**. Their stored evidence is
re-evaluated with the A1 measures, and the batch continues only if the frozen pilot rule (§4, with its fallback) passes.

## 1. What happened
On 17 Nov 2026, J4 locked (freeze `f70d35e3…`) and ran the six C0 MAKL pilots: two each at 0.10, 0.15 and 0.22 m/s. All six
runs succeeded. They had correct configurations, the speed level applied (planner command ≤ V), the meeting rule met (P2) and
the closest approach in time (P6). The pilot rule halted J4 because P3 failed in 4/6 runs, P4 in 6/6 and P5 in 5/6. The
batch stopped as designed.

## 2. Diagnosis (from the stored 10 Hz grids and decision logs)

**P4 and P5 (safety ticks) were a fault in my detector, not safety actions.**
- The v0 detector counted a tick as a guard cut when `cmd_final_v < cmd_raw_v − 0.02`.
- With the DWB cap at V, DWB picks its command from a discrete set of velocity samples. The logged commands at 0.10 m/s were 0.068, 0.074, 0.079, 0.084, 0.089, 0.095 and 0.100 m/s. The two logged commands, before and after the guard, are one tick apart, so every change of sample looked like a "cut". This produced 25–56 flagged ticks per run with mode NORMAL, no latch and raw ranges of 1.5–1.7 m, far above any safety threshold.
- The guard (frozen `safety_chain.py`) can only pass the command or replace it with zero, so a real guard stop has a zero output.
- In J1–J3 the mode gate (0.15) clipped every DWB sample (0.15–0.18), so the gated command was constant and the fault could not appear. The J1 data used to check the metrics did not reveal it.

**P3 (speed in the switching window) measured DWB's sampling, not slowing.**
- In the window, the robot's speed had a median of 0.090–0.094, 0.137–0.141 and 0.208–0.209 m/s, and a minimum of 0.067, 0.119 and 0.193 m/s, at 0.10, 0.15 and 0.22 m/s.
- The share of ticks within ±0.02 m/s of V was 0.65–0.86, because the command jumps between samples.
- There was no stop or slow-down before the window: the minimum was ≥ 0.67 V in every run.
- MAKL's speed input v̂ in W had a median of 0.095, 0.142 and 0.219 m/s (p10 0.084–0.088, 0.126–0.127 and 0.197). It used v̂ in 100 % of decisions.

## 3. Changes
1. **Safety tick (metric `safety`, used by P4, P5 and the safety record).** A tick is a safety tick if the mode is not NORMAL (decision mask), or the guard is latched, or the guard output is zero (< 0.005 m/s) on this tick and the previous one while the gated command is > 0.02 m/s. The rest is unchanged: the wall test, the first safety tick and its raw range, and shield time = ticks × 0.1 s.
2. **P3.** In the speed window (pedestrian start → d < D_LOW[V]):
   - the median robot speed is within 0.02 m/s of V;
   - no tick is below 0.5 V (no stop or slow-down before the window);
   - there are ≥ 5 ticks;
   - the planner command is ≤ V + 0.01.

   The v0 share within ±0.02 m/s is still reported. The 0.5 V bound is set from what P3 must detect (a stop or slow-down before the window, which would merge the levels), not from the observed minimum.
3. **Re-evaluation.** `j4_run_all.py` re-evaluates the six stored pilots once, with the A1 measures and checks, when the STATE is halted by the pilot rule. The old halt and the v0 pilot records are kept in `STATE['amendment_A1']`.

## 4. Result of the re-evaluation (computed offline from the uploaded evidence; the VM repeats it)

| Pilot | P3 median / min (m/s) | First safety tick, raw range | Wall ticks | Result |
|---|---|---|---|---|
| 0.10, δ −0.4 | 0.090 / 0.067 | none | 0 | pass |
| 0.10, δ +0.4 | 0.094 / 0.074 | 0.68 m | 0 | pass |
| 0.15, δ −0.4 | 0.137 / 0.119 | 0.71 m | 0 | pass |
| 0.15, δ +0.4 | 0.141 / 0.127 | 0.72 m | 0 | pass |
| 0.22, δ −0.4 | 0.209 / 0.196 | 0.83 m | 0 | pass |
| 0.22, δ +0.4 | 0.209 / 0.193 | 0.91 m | 0 | pass |

With all six pilots passing, the frozen pilot rule keeps the planned levels 0.10 / 0.15 / 0.22 m/s, and no fallback is needed.
Every first safety action comes below the switching range: raw 0.68–0.91 m against D_LOW + 0.22 = 1.02 / 1.22 / 1.22 m.

## 5. What the report must say
- **The speed levels are DWB caps.** The robot's median speed was about 0.09, 0.14 and 0.21 m/s, and MAKL's median speed input was 0.095, 0.142 and 0.219 m/s. Report the levels with these measured values.
- **The middle level.** MAKL's input (median 0.142) is slightly below FixedV's constant 0.15. The predicted switching values differ by about 0.02 m (0.98 vs 1.00 m), which is within the 0.10 m mechanism tolerance. S0b (agreement), which is reported only, may therefore fall below 95 %.
- **This amendment.** It is reported in the ESM with the v0 failures (§1) and the diagnosis (§2).
