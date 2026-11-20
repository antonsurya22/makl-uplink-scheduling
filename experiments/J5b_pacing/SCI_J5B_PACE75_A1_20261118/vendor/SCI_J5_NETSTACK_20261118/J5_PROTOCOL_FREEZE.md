# J5 protocol (FROZEN) — Does MAKL keep control over a real ROS 2 / Wi-Fi stack?

Status: frozen with the package `SCI_J5_NETSTACK_20261118` on 18 Nov 2026 (SHA256 of every file in
`J5_FREEZE_HASHES.json`). It is draft v2 (`claude/J5_PROTOCOL_DRAFT_V2_REAL_NETWORK_20261118.md`) with the changes listed
in §11; where the two differ, this text holds. Rules as in J1–J4:
- every hypothesis is reported, failed or not;
- no p-values;
- anything not fixed here is exploratory;
- frozen files are never edited; a change after the freeze is a declared amendment.

This is a **network-stack-in-the-loop microbenchmark** and runs open loop: no robot moves and no Gazebo run is added. The
capacity limit is still set by software (the link model in the sender, or `tc` in the sender's kernel). The real parts
are ROS 2 DDS, the Linux network stack, one Wi-Fi hop, and the robot computer.

## 1. Questions
- **Q1 (transfer):** with the bottleneck in the application, do the J1 results hold over real DDS and Wi-Fi?
- **Q2 (bottleneck below the application):** if the capacity limit sits in the kernel and the sender is not paced, does MAKL lose control?
- **Q3 (remedy):** does pacing the sender below the capacity give the control back, and what does it cost?
- **Q4 (end-to-end terms):** how large are the delays outside the link on a robot computer (Eq. 2 and Eq. 9), and how long does MAKL take to decide on that computer?

## 2. Setup (frozen)

**Sender ("robot")**: Raspberry Pi 3 Model B+ (Rev 1.3) of the TurtleBot3, Ubuntu 22.04.5 LTS, aarch64, 4 cores, 1 GB.
ROS 2 Humble, `rmw_cyclonedds_cpp`. The robot stands still; bring-up, LiDAR, camera and motors are off (the script stops
if a bring-up process runs). Wall power. `wlan0`, IP 192.168.200.159. Wi-Fi power save off (set and checked by the
package).

**Receiver ("edge")**: Ubuntu 22.04 VM (VMware) on the Windows laptop DESKTOP-KFRH, bridged adapter `ens33`,
IP 192.168.200.235; the laptop is wired to the router. ROS 2 Humble, `rmw_cyclonedds_cpp`.

**Access point**: TP-Link AX3000 (shared lab router; Internet not used by J5). 2.4 GHz SSID "Sweet_Dumpling",
**channel 9 (2,452 MHz)**, 20 MHz. The sender must stay on BSSID **5c:e9:31:85:b4:1b** (checked before, every second
during, and after each replay). The second BSS with the same SSID (32:16:9d:ae:e5:67, −45 dBm) is recorded.

**Background**: other 2.4 GHz clients during the 18 Nov check: one smart plug, one laptop, one phone, all idle. Neighbouring
networks on every channel 1–11. Anton records the client list and a channel scan before and after the session.

**Baseline (S1 reference)**: channel 9, power save off, ICMP with 3,000 B on the wire, 5 Hz, 600 packets: RTT p50 8.3 ms,
p95 16.5, p99 22.9, max 27.0 ms; loss 0 %.

**DDS**: `ROS_DOMAIN_ID = 77`; CycloneDDS bound to `wlan0` (sender) and `ens33` (edge); multicast off, unicast discovery
with the other host as the only peer (`j5_env.sh`).
- Data topics `/j5/scan`, `/j5/camera`, `/j5/probe` (`std_msgs/UInt8MultiArray`): writer best effort, keep-last 1;
  reader best effort, keep-last 50 (so the edge never drops a sample because its callback ran late).
- Control and file topics `/j5/ctl`, `/j5/ack` (`std_msgs/String`): reliable, keep-last 200. They carry no data of the
  experiment and are idle while a replay runs, except the `begin` request before and `end` after it.
- Each data message has exactly the payload size of the offer; its first 44 bytes are a header (magic, replay number,
  stream, message id, capture, offer, end-of-transmission and publish times).

**Software freeze**: no `apt upgrade` on either host until J5 ends. Versions (kernel and packages) are recorded at LOCK
and must be unchanged at every block.

**Clock**: chrony. The edge VM serves time (`allow 192.168.200.0/24`, `local stratum 10`); the sender uses it as
`server 192.168.200.235 iburst minpoll 0 maxpoll 2 prefer trust`. The sender's reference must be the VM and
|offset| ≤ 2 ms at LOCK, at the start of every block, and before and after every replay. Clock error is a common offset
within a replay, so it does not affect the paired arm comparisons; absolute delays (Q4) are reported with this bound.

## 3. Traffic and schedulers

**Traces.** The archived J1 v2 runs **without congestion (C0), arm B7, rep 1–5**, for camera profile Q1 and Q2 (10
traces, `traces/J5_TRACE_<cell>_R<rep>.json`), as J0 did. The congested runs log only the frames that were delivered,
so they cannot give the offered traffic; the C0 runs delivered every offer. Each trace holds:
- scan capture times (5 Hz, 3,000 B on the wire) and camera frame capture times and sizes, via frozen
  `e4_harness.load_trace`;
- the robot trajectory (v_odom, v_cmd, d) via frozen `j_replay.state_series`, and the grid used for the open-loop
  violation;
- the SHA256 of the five source files. On the VM the traces are rebuilt from the J1 results and must be byte-identical.

**Replay window.** From the goal time (rounded down to the 100 ms tick) to T + 30 s, where T is the trigger time of the
trace: about 34.5 s in real time per replay. W = [T, T + 25 s) is the congestion window, as in J1.

**Offers.** On every 100 ms tick: robot state to MAKL, scans captured up to the tick, camera frames captured up to the tick
(camera reliable after the goal in Q1, as in J1), and a probe/ACK message every 0.5 s, all exactly as frozen
`j_replay.replay`.

**Scheduler code.** The frozen J1 v2 link classes (`j1_links.make_link`, unchanged, vendored with their freeze hashes)
run in real time on the sender: capacity 3 Mbit/s outside W, queue limit 768,000 B, delay 0. The replay loop wakes at
every link event and at every capacity boundary. When the link model finishes "transmitting" a message, the sender
publishes it over DDS at once. With a virtual clock the replay core gives the same status, times and order for every
message as frozen `j_replay.replay` (offline test, 4 arms × 2 cells × 3 modes).

**Arms:** B0 (FIFO), B3 (keep-last), B7 (priority + keep-last), MAKL (J1 parameters).
**Cells:** Q2 at c = 0.13 Mbit/s and Q1 at c = 0.18 Mbit/s.

## 4. Where the bottleneck sits

| Mode | Link model in the sender during W | Kernel limit on `wlan0` | Counted |
|---|---|---|---|
| **A: app** | capacity c (as J1) | none | yes |
| **N: net, unpaced** | 3 Mbit/s (unchanged) | TBF, rate c during W | yes (N1 is a demonstration) |
| **NP: net, paced** | capacity 0.9 c (the known c) | TBF, rate c during W | yes |

**Kernel limit.** In N and NP a TBF replaces the root qdisc of `wlan0` for the whole replay: rate 3 Mbit/s, switched to c
at T and back to 3 Mbit/s at T + 25 s (same trace times as the link model), burst 1,600 B, limit 768,000 B. It is
deleted after the 2 s drain. `tc -s qdisc` is sampled at 10 Hz. The TBF counts link-layer bytes (payload plus RTPS, UDP,
IP and Ethernet headers, a few per cent more than the payload); the 10 % pacing margin of NP covers this. In N the
kernel queue may push back on the sender's socket; publish lateness is logged and reported.

**Why NP uses the known c.** MAKL's ĉ is the delivered wire rate over the last 2 s. Pacing at 0.9 ĉ would lower ĉ by
10 % every window, so the pacing rate would ratchet down.

## 5. Measurements
Sender, per message: capture, offer, end of transmission in the link model, DDS publish time, size (`sent.jsonl`), and the
final status of every offer (`status.jsonl`); link ledger and MAKL decisions with CPU time per decision; qdisc samples
(10 Hz) and BSSID samples (1 Hz). Edge: receive time as the first action of the subscriber callback (`edge/<rid>.jsonl`).
Times on the edge map to trace time with the sender's wall time at the replay start.

Per replay:
- **Planner age a_nav**: at each scan delivered at the edge in W, its time minus the newest capture among the scans
  delivered before it; p95 and max. **Link age a_link**: the same with the end of transmission in the link model.
- **F**: camera frames received at the edge in W within 1.0 s of capture, per second of W.
- **Open-loop violation time**: frozen `j_replay.violation` with the edge scan deliveries and the C0 trajectory. A replay
  metric, not a closed-loop outcome.
- **Delays outside the link**: ε_ce = offer − capture (0 by construction in replay, reported); one-way delay = publish →
  edge callback; **ε_net+d = end of transmission in the link model → edge callback** (includes publish lateness, DDS,
  kernel, the `tc` queue and the Wi-Fi hop; delivery "at the edge interface" cannot be observed). p50 and p99.
- **Publish lateness**: end of transmission in the link model → DDS publish.
- **MAKL decision time on the Pi**: thread CPU time per decision; p50, p99, max.
- **Kernel backlog in W**: median, max, share of samples with > 3,000 B queued.
- **Link health**: BSSIDs seen, RSSI, clock offset before and after, baseline ping before the block.

## 6. Setup checks and pilots
- **S1 Channel [S]: channel 9 kept** (see §11, D6); reference p50 RTT 8.3 ms.
- **E1 Session checks [E]** at LOCK and at the start of every block: sender IP; BSSID as frozen; power save off; no
  throttling; clock (reference = VM, |offset| ≤ 2 ms); domain 77, CycloneDDS, bound to `wlan0`; versions and channel
  unchanged since LOCK. A failing check at a block start is re-checked every 30 s, up to 10 times; then the run halts.
  At LOCK a failing check stops the script without starting (nothing is counted yet).
- **Pilots (uncounted, run once after LOCK):** PIL_A_Q1_B7, PIL_A_Q1_MAKL, PIL_N_Q1_B0, PIL_N_Q2_B0 (trace rep 1), then
  iperf3 UDP 2 Mbit/s for 10 s from the sender to the edge.
  - **S2 DDS on wlan0:** PIL_A_Q1_B7 is valid and the qdisc byte counter of `wlan0` rose by ≥ 0.9 × the payload bytes published.
  - **S3 Ceiling:** iperf3 received ≥ 1.9 Mbit/s with loss ≤ 2 %; one-way delay p50 ≤ 10 ms in PIL_A_Q1_B7.
  - **S4 Capacity:** in PIL_N_Q1_B0 and PIL_N_Q2_B0, the qdisc sent rate over W is within 5 % of c. (In mode A the link
    model sets the rate; the offline test shows it equals the J0 replay.)
  - **S5 Real-time fidelity:** publish lateness in PIL_A_Q1_B7 p50 ≤ 10 ms and p99 ≤ 50 ms.
  - **S6 Decision time:** MAKL thread CPU time per decision in PIL_A_Q1_MAKL, p99 ≤ 5 ms.
  - All four pilot replays valid.
  If any pilot check fails, the run halts; going on needs a declared amendment.

**Validity.** A replay is valid if the BSSID was the frozen one before, during and after it, the clock check passed
before and after it, the edge received data, and all its files reached the edge (SHA256 checked). Before each block the
sender pings the edge (50 × 3,000 B at 5 Hz); the ping is bad if loss > 1 % or p50 > 2 × 8.3 ms.
- A block whose ping is bad, or with any invalid replay, is repeated once (replay ids get `_a2`; after a bad ping the
  sender waits 60 s).
- A block that fails twice is reported as missing and not repeated again.
- Every first attempt, repeat and missing block is reported. Only the replays of the valid attempt of a block are counted.

## 7. Matrix
- **Counted:** 4 arms × 2 cells × 3 modes × 5 traces = **120 replays** in 30 blocks. Order: rep 1 → 5; within a rep,
  cell Q2 then Q1; within a cell, the three modes in an order rotated per rep (A,N,NP / N,NP,A / NP,A,N / …); within a
  block, the arm order rotated by block.
- **Not in this package:** NP-est and C3 (draft v2: exploratory). If they are run later, they get their own short protocol.
- Time: about 41 s per replay (34.5 s replay + 2 s drain + checks and uploads), so about 1.5–2 h for pilots and matrix.

## 8. Predictions and hypotheses
**Decision rule (J1).** A is clearly greater than B if at least 4 of 5 pairs exist, all paired differences are > 0, and
the mean exceeds J. Pairs are the traces (rep 1–5).
- J for age = J_age(cell) = max(0.1 s, range over reps of B7's p95 a_nav in mode A of that cell).
- J for F = 0.2 fps (Q2) / 0.4 fps (Q1). J for violation = 0.7 s (Q2) / 0.8 s (Q1).

**Capacity left for the camera.** The scan stream alone needs 3,000 B × 8 × 5 Hz = 120 kbit/s.

| Cell | c | 0.9 c | Left for the camera at c | Left at 0.9 c |
|---|---|---|---|---|
| Q2 | 130 kbit/s | 117 kbit/s | ≈ 10 kbit/s | none (scans alone exceed it) |
| Q1 | 180 kbit/s | 162 kbit/s | ≈ 60 kbit/s | ≈ 42 kbit/s |

| ID | Statement (test in `j5_analysis.py`) | Prediction |
|---|---|---|
| T1 | Mode A, both cells: B0's p95 a_nav clearly greater than B3's, B7's and MAKL's; Q1: MAKL's F clearly greater than B7's | holds |
| T2 | Mode A, both cells: MAKL's violation not clearly greater than B7's (and evaluable) | holds |
| N1 | Mode N, both cells: B0's p95 a_nav not clearly greater than MAKL's (and evaluable) | holds by construction (the application queue stays nearly empty, so all arms send the same bytes). **Demonstration**, not a test |
| N2 | Mode N: mean share of W with kernel backlog > 3,000 B is > 0.5 for every arm and cell | holds |
| P1 | Both cells: MAKL's p95 a_nav in mode N clearly greater than in NP | holds |
| P2 | Both cells: mean paired (NP − A) of MAKL's p95 a_nav ≤ J_age (≥ 4 pairs) | holds; the bound rises by 8(L_m + L_s)/c · (1/0.9 − 1): +40 ms (Q2, L_s 2,819 B) and +30 ms (Q1, L_s 3,170 B) |
| P3 | Mode NP − mode A in F, every arm and cell | Q1: about −30 % for B7 and MAKL; Q2: F ≈ 0 in NP. Reported |
| E1 | ε_net+d, one-way delay, publish lateness, MAKL decision time on the Pi | reported (no hypothesis) |

**Outcomes**
- TRANSFERS = T1 ∧ T2.
- CONTROL_NEEDS_PACING = P1. With N1 as the demonstrated failure, the paper states pacing below capacity as a requirement
  of MAKL when the bottleneck may lie below the application, with the cost P3.
- CONTROL_NOT_RECOVERED = ¬P1.
- If N1 fails, first check S4 and N2 (was the kernel queue in fact long?).

## 9. Outputs (edge VM, `results/SCI_J5_NETSTACK_20261118/J5_MAIN/final/`)
`J5_REPLAYS.csv` (one row per counted replay), `J5_CELLS.csv` (means and SD per cell × mode × arm), `J5_TESTS.csv`
(every test with its paired differences), `J5_VERDICTS.json`, `J5_PLATFORM.json` (Q4 values), `J5_PILOTS.json`.
Raw logs: `edge/`, `sender/`, `STATE_SENDER.json`, console logs of both hosts.

## 10. What the manuscript gets
- A short Section 6.6 "Real network stack (J5)" and one figure (p95 a_nav and F by mode and arm).
- The Limitations sentences on the emulated link and on the location of the bottleneck are replaced by the J5 result.
- ESM S18: setup (§2), clock check, QoS, pilots, block repeats, and all tables.
- The measured ε_net+d and the decision time on the Pi go into Eq. (9) and Section 4 as platform values.

## 11. Changes from draft v2 (declared at the freeze)
- **D1 Traces:** the C0 B7 runs (rep 1–5), not the congested B7 runs. The congested runs log only delivered frames, so
  they cannot give the offered traffic. Same choice as J0.
- **D2 Replay length:** goal time → T + 30 s (about 34.5 s), the J0 replay window, instead of "60 s with W from 15 s".
  W is unchanged (25 s).
- **D3 QoS:** the reader uses keep-last 50 (writer keep-last 1, both best effort), so that a late callback at the edge
  never drops a sample. Discovery is unicast with multicast off (multicast over Wi-Fi is slow and unreliable).
- **D4 S5:** the link model is the frozen J0/J1 code, and the offline test shows the same output as `j_replay.replay`
  record by record. S5 therefore checks real-time fidelity (publish lateness) instead of comparing link ages.
- **D5 S4:** checked in mode N (the kernel limit) with B0. In mode A the link model sets the rate exactly (offline test).
- **D6 S1:** channel 9 is kept. The router is shared with the lab, and channel 9 already gives p99 RTT 22.9 ms and no loss
  with power save off. Channels 6 and 11 are not tested.
- **D7 Clock checks:** at LOCK, at every block start, and before and after every replay (not every 10 s). A drift at a
  block start gets up to 5 min to settle before the run halts.
- **D8 Exploratory modes:** NP-est and C3 are not in this package.
- **D9 P2 arithmetic:** the Q1 bound is +30 ms (draft: +34 ms); computed with L_m = 3,000 B and L_s = 3,170 B.
- **D10 tc timing:** the TBF is installed at 3 Mbit/s for the whole N/NP replay and its rate is changed at T and T + 25 s,
  so the kernel limit and the link model switch at the same trace times.
