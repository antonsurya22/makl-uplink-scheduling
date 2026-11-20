"""ESM Section S18 (J5 and J5b, real ROS 2 / Wi-Fi stack) from the frozen J5 and J5b outputs.
Usage: gen_esm_j5.py DATA_J5_DIR DATA_J5B_DIR OUT.tex"""
import csv, json, sys, statistics as st
from pathlib import Path

D5, D5B, OUT = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
f = lambda x: float(x) if x not in ('', None) else None
C5 = list(csv.DictReader(open(D5 / 'J5_CELLS.csv')))
T5 = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(D5 / 'J5_TESTS.csv'))}
CB = list(csv.DictReader(open(D5B / 'J5B_CELLS.csv')))
TB = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(D5B / 'J5B_TESTS.csv'))}
PB = json.load(open(D5B / 'J5B_PREDICTIONS.json'))['cells']
NAME = {'B0': 'B0 FIFO', 'B3': 'B3 keep-last', 'B7': 'B7 priority + keep-last', 'MAKL': '\\textbf{MAKL}'}
L = []; A = L.append
def v(r, k, d=2):
    x = f(r.get(k)); return '--' if x is None else f'{x:.{d}f}'
def diffs(t, d=2): return ', '.join(f'{x:+.{d}f}' for x in t['diffs'])

A(r"""\section{J5 and J5b: real ROS~2 and Wi-Fi stack}\label{sec:s-j5}

\paragraph{Purpose and type.} J5 asks whether the J1 result holds over a real network stack, and what happens when the
capacity limit lies below the application. It is a network-stack-in-the-loop microbenchmark and runs open loop: no robot
moves and no simulation run is added. The capacity limit is still set in software (the link model in the sender, or a
token bucket in the sender's kernel). The real parts are ROS~2 DDS, the Linux network stack, one Wi-Fi hop and the robot
computer. Package \texttt{SCI\_J5\_NETSTACK\_20261118}, frozen with its runner and analysis (SHA256 of 49 files); run on
18 November 2026. J5b (package \texttt{SCI\_J5B\_PACE75\_20261118}) followed on the same day with one changed constant.

\paragraph{Hosts and network.} Sender: the Raspberry Pi~3 Model~B+ of the TurtleBot3 (Ubuntu 22.04.5, aarch64, 1\,GB),
ROS~2 Humble with CycloneDDS~0.10.5, on wall power, with bring-up, LiDAR, camera and motors off; Wi-Fi power save off.
Edge: an Ubuntu 22.04 virtual machine (VMware, bridged adapter) on a laptop wired to the router. Access point: a shared
TP-Link AX3000 lab router, 2.4\,GHz, channel~9, 20\,MHz; the sender had to stay on one frozen BSSID (checked before, every
second during, and after each replay). A second access point with the same SSID was recorded. With power save off, a
3,000\,B ping at 5\,Hz had a round-trip time of 8.3\,ms (median), 22.9\,ms (99th percentile) and no loss.

\paragraph{DDS and clock.} Domain 77; CycloneDDS bound to the Wi-Fi interface of the sender and the bridged interface of
the edge; multicast off and unicast discovery with the other host as the only peer. Data topics (scan, camera, probe):
writer best effort, keep-last~1; reader best effort, keep-last~50, so that a late callback never drops a sample. Each data
message has exactly the payload size of the offer, with a 44\,B header of time stamps. Control topics are reliable and idle
during a replay. Clock: chrony with the edge as server; the sender's offset estimate had to stay within $\pm2$\,ms at the
lock, at every block start and before and after every replay (J5: at most 0.28\,ms in 248 checks; J5b: 0.12\,ms in 128).
The clock error is a common offset within a replay, so it does not affect the paired comparisons.

\paragraph{Traces and replay.} The archived J1 runs without congestion, arm B7, repetitions 1--5, for camera profiles Q1
and Q2 (as in J0; congested runs log only delivered frames, so they cannot give the offered traffic). Each replay runs
from the goal time to $T+30$\,s in real time (about 34.5\,s), with the congestion window $W=[T,T+25\,\text{s})$ as in J1.
On every 100\,ms tick the sender offers the robot state to MAKL, the scans and camera frames captured up to the tick, and
a probe every 0.5\,s, exactly as the frozen J0 replay. The frozen J1 link classes run unchanged in real time; when the
link model finishes a message, the sender publishes it over DDS at once. With a virtual clock the replay core gives the
same output as the frozen offline replay record by record.

\paragraph{Where the bottleneck sits.}
\begin{itemize}
\item Mode A (application): the link model runs at $c$ during $W$; no kernel limit.
\item Mode N (kernel, unpaced): the link model stays at 3\,Mbit/s; a token bucket (TBF) on the Wi-Fi interface runs at
  $c$ during $W$ (burst 1,600\,B, limit 768,000\,B, installed at 3\,Mbit/s for the whole replay and switched at the same
  trace times as the link model).
\item Mode NP (kernel, paced): as N, with the link model at $0.9c$ of the known $c$. (Pacing at $0.9\hat c$ would ratchet
  down, because $\hat c$ is the delivered rate.)
\end{itemize}

\paragraph{Measurements.} Planner age $a_{\text{nav}}$: at each scan delivered at the edge in $W$, its time minus the
newest capture among the scans delivered before it; link age $a_{\text{link}}$: the same with the end of transmission in
the link model. $F$: camera frames received at the edge within 1.0\,s of capture, per second of $W$. Open-loop violation:
the frozen J0 violation with the edge deliveries and the recorded trajectory (a replay metric, not a closed-loop outcome).
Delays: one-way (DDS publish to edge callback) and $\epsilon_{\text{net+d}}$ (end of transmission in the link model to
the edge callback). Kernel backlog sampled at 10\,Hz. MAKL's thread CPU time per decision on the Pi.

\paragraph{Pilots and validity.} Pilots (uncounted): the qdisc byte counter of the Wi-Fi interface rose by 807,624\,B in
the B7 pilot ($\ge0.9\times$ the payload, so the data took this path); iperf3 delivered 2.00\,Mbit/s with no loss and the
one-way delay had a median of 4.4\,ms; the kernel rate in $W$ was 180.0 and 130.1\,kbit/s for $c=180$ and 130\,kbit/s;
publish lateness had a 99th percentile of 9.0\,ms; MAKL's decision on the Pi had a median of 0.81\,ms and a 99th
percentile of 1.09\,ms. A replay was valid if the BSSID, the clock and all files checked out; a block (one trace, mode
and cell with the four arms) was repeated once if its baseline ping was bad or a replay invalid. All 30 J5 blocks were
valid at the first attempt (ping median 7.4--9.1\,ms).

\paragraph{Decision rule.} As J1: A is clearly greater than B if at least four of five pairs exist, every paired
difference is positive and the mean exceeds $J$. $J$ for age $=\max(0.1\,\text{s}, \text{range of B7 in mode A})$ (0.1\,s
in both cells); for $F$ 0.2\,fps (Q2) and 0.4\,fps (Q1); for violation 0.7\,s (Q2) and 0.8\,s (Q1).
""")

# ---- J5 cell table
A(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{3.2pt}
\caption{J5 cell means (five replays each). $a_{\text{nav}}$, $a_{\text{link}}$: 95th-percentile scan age at the edge and
in the link model; $F$: useful camera frames per second; Viol.: open-loop violation time; OW: one-way delay, median;
Backlog: share of $W$ with more than 3,000\,B in the kernel queue.}\label{tab:s-j5cells}
\begin{tabular}{@{}lllcccccc@{}}
\toprule
Cell & Mode & Scheduler & $a_{\text{nav}}$ (s) & $a_{\text{link}}$ (s) & $F$ (fps) & Viol. (s) & OW (ms) & Backlog \\
\midrule""")
first = True
for cell in ('Q2', 'Q1'):
    for mode in ('A', 'N', 'NP'):
        if not first: A(r"\midrule")
        first = False
        for i, arm in enumerate(('B0', 'B3', 'B7', 'MAKL')):
            r = next(x for x in C5 if x['cell'] == cell and x['mode'] == mode and x['arm'] == arm)
            ow = f(r['ow_p50_ms']); ows = f'{ow:,.0f}' if ow >= 100 else f'{ow:.1f}'
            A(f"{cell if i == 0 else ''} & {mode if i == 0 else ''} & {NAME[arm]} & {v(r,'anav_p95_s')} & {v(r,'alink_p95_s')} & "
              f"{v(r,'F_fps')} & {v(r,'viol_s',1)} & {ows} & {v(r,'backlog_share_gt_3000B')} \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}""")

# ---- J5 tests
A(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{3pt}
\caption{J5 frozen tests (paired over the five traces).}\label{tab:s-j5tests}
\begin{tabular}{@{}llccl@{}}
\toprule
Test & Comparison & Mean & $J$ & Result; paired differences \\
\midrule""")
rows = [('T1', 'Q2, A: B0 $>$ B3, $a_{\\text{nav}}$', 'T1_Q2_A_B0_gt_B3_anav'), ('T1', 'Q2, A: B0 $>$ B7', 'T1_Q2_A_B0_gt_B7_anav'),
        ('T1', 'Q2, A: B0 $>$ MAKL', 'T1_Q2_A_B0_gt_MAKL_anav'), ('T1', 'Q1, A: B0 $>$ B3', 'T1_Q1_A_B0_gt_B3_anav'),
        ('T1', 'Q1, A: B0 $>$ B7', 'T1_Q1_A_B0_gt_B7_anav'), ('T1', 'Q1, A: B0 $>$ MAKL', 'T1_Q1_A_B0_gt_MAKL_anav'),
        ('T1', 'Q1, A: MAKL $>$ B7, $F$', 'T1_Q1_A_MAKL_gt_B7_F'),
        ('T2', 'Q2, A: MAKL $>$ B7, violation (must not be clear)', 'T2_Q2_A_MAKL_gt_B7_viol'),
        ('T2', 'Q1, A: MAKL $>$ B7, violation (must not be clear)', 'T2_Q1_A_MAKL_gt_B7_viol'),
        ('N1', 'Q2, N: B0 $>$ MAKL (demonstration)', 'N1_Q2_N_B0_gt_MAKL_anav'), ('N1', 'Q1, N: B0 $>$ MAKL (demonstration)', 'N1_Q1_N_B0_gt_MAKL_anav'),
        ('P1', 'Q2: MAKL N $>$ NP, $a_{\\text{nav}}$', 'P1_Q2_MAKL_N_gt_NP_anav'), ('P1', 'Q1: MAKL N $>$ NP', 'P1_Q1_MAKL_N_gt_NP_anav'),
        ('P2', 'Q2: MAKL NP $-$ A $\\le J$ (pred.\\ $+0.04$)', 'P2_Q2_MAKL_NP_minus_A_anav'), ('P2', 'Q1: MAKL NP $-$ A $\\le J$ (pred.\\ $+0.03$)', 'P2_Q1_MAKL_NP_minus_A_anav')]
for hyp, comp, key in rows:
    t = T5.get(key)
    if t is None:
        cand = [k for k in T5 if k.startswith(key.split('_')[0] + '_' + key.split('_')[1])]
        raise SystemExit(f'missing {key}; candidates {cand}')
    if hyp == 'P2':
        res = 'holds' if t['mean_diff'] <= t['J'] else 'fails'
    elif hyp in ('T2', 'N1'):
        res = 'not clear (holds)' if not t.get('clearly_greater') else 'clear (fails)'
    else:
        res = 'clear' if t.get('clearly_greater') else 'not clear'
    A(f"{hyp} & {comp} & {t['mean_diff']:+.2f} & {t.get('J', '--')} & {res}; {diffs(t)} \\\\")
A(r"""N2 & N: backlog $>$3\,kB for more than half of $W$, every arm and cell & -- & -- & holds (0.96--0.98) \\
\bottomrule
\end{tabular}
\end{table}

\paragraph{J5 verdicts.} TRANSFERS (T1 and T2 hold). N1 holds as a demonstration: without pacing every scheduler reached
a 95th-percentile age of 7.4--8.9\,s. CONTROL\_NEEDS\_PACING (P1 holds). P2 failed in both cells: at $0.9c$ MAKL's age
stayed 2.0\,s (Q2) and 1.2\,s (Q1) above mode A. P3 (reported): $F$ in NP minus A was $-0.13$, $-1.37$, $-0.41$ and
$-0.52$\,fps (Q2: B0, B3, B7, MAKL) and $-0.26$, $-1.96$, $-1.56$ and $-1.75$\,fps (Q1); the predicted cost in Q1 was
about $-30$\,\%, the measured one about twice that.

\paragraph{Platform values (mode A, median over the 40 replays; largest value in brackets).} One-way delay 4.7\,ms (p50)
and 18.4\,ms (p99; 34.6); $\epsilon_{\text{net+d}}$ 6.5\,ms (p50) and 20.8\,ms (p99; 38.4); publish lateness p99
8.5\,ms (10.6); MAKL decision on the Pi 0.76\,ms (p50) and at most 1.34\,ms (p99; largest single decision 3.3\,ms). The
capture-to-enqueue delay in the replay is the 100\,ms tick of the frozen harness (median 58\,ms) and is not a platform
value.

\paragraph{Why P2 failed (exploratory, after the analysis).} The scheduler paces its own byte count (the message plus
28\,B per packet of at most 1,472\,B), while the token bucket counts link-layer bytes. In $W$ the bytes entering the
kernel queue were 1.16--1.23 times the bytes the scheduler counted (1.18--1.26 times the serialized messages). About
10\,kbit/s of this was background traffic (DDS discovery, heartbeats of the reliable control topic, clock
synchronization), measured in the 2\,s after each mode A replay (noisy; 90th percentile 66\,kbit/s); the rest is framing
beyond the model (RTPS, UDP, IP and Ethernet headers; about 3.5 packets per scan), about 1.10 (Q1) and 1.14 (Q2) times
the scheduler's bytes. In NP the scheduler sent 155.9 (Q1) and 113.0\,kbit/s (Q2) of its own bytes, and 182.8 and
139.4\,kbit/s entered the kernel queue, 1.02$c$ and 1.07$c$. MAKL's link age rose by only 0.054 and 0.073\,s from A to NP
(predicted 0.030 and 0.040\,s); almost all of the excess is the kernel queue.

\paragraph{Other observations (exploratory).} With Q1 in mode A, MAKL's 95th-percentile age was above B7's (0.72 and
0.53\,s) at the same violation: MAKL spends its budget on frames while the pedestrian is far. With Q2 in mode A, keep-last
gave the most frames (1.58\,fps) at a violation of 2.2\,s (MAKL 1.6\,s). In NP with Q1, FIFO kept the kernel queue short
(backlog share 0.26, one-way delay 72\,ms) because its own queue held the backlog in the application.

\paragraph{Changes declared at the J5 freeze.} D1: traces from the uncongested runs (the congested ones log only
delivered frames). D2: replay from the goal time to $T+30$\,s (W unchanged). D3: reader keep-last 50; unicast discovery,
multicast off. D4: the real-time check is the publish lateness (the link model equals the offline replay record by record
in an offline test). D5: the kernel-rate pilot in mode N. D6: channel~9 kept (no test of channels 6 and 11). D7: clock
checks at the lock, every block start and before and after each replay. D8: the estimated-pace and contention modes of
the draft are not part of J5. D9: the Q1 prediction of P2 is $+30$\,ms. D10: the token bucket switches at the same trace
times as the link model. After the last replay the sender process aborted on exit (a ROS shutdown error), after all files
had been checked; it affects no result.

\paragraph{J5b: pace $0.75c$.} J5b vendored the J5 package unchanged and changed one constant, the pace of mode NP, to
$0.75c$ in the scheduler's byte count (Q1: link model 135\,kbit/s, token bucket 180\,kbit/s), so that the load entering
the kernel, with framing and background traffic, stays below $c$. Counted: Q1, modes A and NP75 in the same session,
4 arms $\times$ 5 traces = 40 replays; Q2 NP75 (20 replays) is a demonstration, because 97.5\,kbit/s is below the scan
load of 120\,kbit/s, so assumption (A1) of Proposition~1 fails there. The frozen predictions are the link-model values
of the same traces (offline replay at the J5b pace). Hypotheses: K1 the median kernel backlog is below 3\,kB in at least
four of five replays of every arm; K2 MAKL's one-way delay p99 $\le$ 50\,ms; K3 the edge follows the link model within
$J_{\text{age}}$; R1 MAKL's age NP75 $-$ A $\le$ the predicted change (0.196\,s) $+\,J_{\text{age}}$; R2 MAKL's violation
in NP75 not clearly above mode A. A first attempt stopped in its pilots because one read of the Wi-Fi association
returned nothing (no disconnection in any log); amendment A1 added retries for failed reads before any counted replay. A
background pilot (60\,s, all nodes running, no replay) measured 21.6\,kbit/s on average (90th percentile 64\,kbit/s),
about twice the J5 estimate and at the 0.12$c$ headroom; the frozen protocol records this pilot and does not use it as a
stop rule. All 15 blocks were valid at the first attempt.
""")

# ---- J5b Q1 table with predictions
def pmean(key, k):
    return st.mean(x[k] for x in PB[key])
A(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{3.2pt}
\caption{J5b, Q1 at 0.18\,Mbit/s: measured means (five replays) and the frozen link-model predictions [in brackets].
OW: one-way delay, median / 99th percentile.}\label{tab:s-j5b}
\begin{tabular}{@{}llccccc@{}}
\toprule
Mode & Scheduler & $a_{\text{nav}}$ (s) [pred.\ $a_{\text{link}}$] & $F$ (fps) [pred.] & Viol. (s) [pred.] & OW (ms) & Backlog \\
\midrule""")
for j, mode in enumerate(('A', 'NP75')):
    if j: A(r"\midrule")
    for i, arm in enumerate(('B0', 'B3', 'B7', 'MAKL')):
        r = next(x for x in CB if x['cell'] == 'Q1' and x['mode'] == mode and x['arm'] == arm)
        key = f'Q1|{mode}|{arm}'
        A(f"{mode if i == 0 else ''} & {NAME[arm]} & {v(r,'anav_p95_s',3)} [{pmean(key,'alink_p95_s'):.3f}] & {v(r,'F_fps')} [{pmean(key,'F_fps'):.2f}] & "
          f"{v(r,'viol_s',1)} [{pmean(key,'viol_s'):.1f}] & {f(r['ow_p50_ms']):.1f} / {f(r['ow_p99_ms']):.0f} & {v(r,'backlog_share_gt_3000B')} \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}""")

# ---- J5b tests
k1 = [TB[f'K1_{a}_median_backlog_lt_3000B'] for a in ('B0', 'B3', 'B7', 'MAKL')]
def tget(prefix):
    ks = [k for k in TB if k.startswith(prefix)]
    if len(ks) != 1: raise SystemExit(f'{prefix}: {ks}')
    return TB[ks[0]]
A(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{3pt}
\caption{J5b frozen tests (Q1; paired over the five traces).}\label{tab:s-j5btests}
\begin{tabular}{@{}lp{6.2cm}l@{}}
\toprule
Test & Values & Result \\
\midrule""")
A(f"K1 & median backlog 0.6--1.8\\,kB in all 20 replays; share of $W$ above 3\\,kB "
  f"{min(t['share_gt_3000B_mean'] for t in k1):.2f}--{max(t['share_gt_3000B_mean'] for t in k1):.2f} (expected below 0.10) & holds \\\\")
A(r"K2 & MAKL's one-way delay p99 per trace 581--613\,ms (median 596\,ms) & fails \\")
A(r"K3 & measured minus predicted 95th-percentile age: MAKL $+0.25$, B7 $+0.48$, B3 $+0.36$, B0 $+0.47$\,s ($J=0.1$\,s) & fails \\")
r1 = TB['R1_MAKL_NP75_minus_A_anav']; r2 = TB['R2_MAKL_viol_NP75_gt_A']
A(f"R1 & MAKL's age NP75 $-$ A: {diffs(r1)}; mean {r1['mean_diff']:+.3f}\\,s (limit 0.196 + 0.1\\,s) & fails \\\\")
A(f"R2 & MAKL's violation NP75 $-$ A: {diffs(r2,1)}; mean {r2['mean_diff']:+.2f}\\,s ($J=0.8$\\,s) & fails (clearly greater) \\\\")
A(r"""Reported & R1 for B7 / B3 / B0: $+0.58$ / $+0.93$ / $+0.53$\,s (predicted $+0.10$ / $+0.57$ / $+0.07$\,s) & above prediction \\
Reported & MAKL $-$ B7 in $F$ in NP75: $+0.60$, $+1.08$, $+0.84$, $+0.92$, $+0.68$; mean $+0.82$\,fps & clearly greater \\
\bottomrule
\end{tabular}
\end{table}

\paragraph{J5b verdict.} CONTROL\_NOT\_RESTORED: the pace of $0.75c$ kept the median kernel queue short (K1) and the mean
load below $c$, but it did not bring the absolute scan age or the margin back to what the scheduler alone produces, for any
scheduler. The relative result did not change (paired MAKL $-$ B7 in Q1; $F$ reported, the rest exploratory): in mode A
$F$ $+0.81$\,fps (five of five pairs), violation 0.0\,s (mixed signs), age $+0.19$\,s; in NP75 $F$ $+0.82$\,fps (five of
five pairs), violation $-0.3$\,s (mixed signs), age $+0.05$\,s.

\paragraph{Why (exploratory, after the analysis).} The mean load was as planned: 0.88$c$ (Q1) and 0.93$c$ (Q2) entered the
kernel queue, 1.21--1.29 times the scheduler's bytes with background traffic. Three effects kept the queue from staying
short.
\begin{itemize}
\item Discovery bursts: every 8.0\,s (median of 92 intervals) the kernel queue jumped by about 11.5\,kB (8--25\,kB) within
  0.1\,s, in every NP75 replay. This is consistent with CycloneDDS unicast discovery as configured (one peer, automatic
  participant index, maximum index 30): the configuration options state that the maximum automatic participant index sets
  the range of ports pinged for unicast participant discovery and that the default announcement interval is about 80\,\%
  of the lease duration (8\,s with the default 10\,s), and the discovery configuration page notes that peers configured
  this way cause a large burst of packets each time an announcement is sent. The background pilot counted 240 UDP
  datagrams in 60\,s, about 32 per 8\,s. We did not capture packets. At 180\,kbit/s such a burst takes about 0.5\,s to
  drain, which matches the 99th-percentile delay of about 0.6\,s.
\item High load on a FIFO: with 0.88$c$ plus the bursts, more than 3\,kB was queued for 28--39\,\% of $W$. About 18\,\% of
  the scans delivered in $W$ left the scheduler within 1.5\,s after a burst; without them, MAKL's age is still 0.22\,s
  above the link-model prediction (B7: 0.42\,s).
\item A second transmission at the bottleneck: the scheduler hands a message to DDS at the end of its own transmission,
  and the token bucket transmits it again at $c$. With the first 1,600\,B passed at once, a scan waits about
  $(3{,}300-1{,}600)\times8/180{,}000\approx76$\,ms; the measured median one-way delay was 75\,ms (5.7\,ms in mode A).
  The frozen prediction left this term out; on its own it is within $J_{\text{age}}$. It also applies to NP in J5.
\end{itemize}
Across sessions (exploratory), NP75 was better than NP at $0.9c$ in J5: MAKL's 95th-percentile age 1.16 against 1.92\,s,
violation 5.4 against 12.5\,s, $F$ 1.50 against 1.32\,fps, one-way delay 75 against 620\,ms (medians).
""")

# ---- J5b Q2 demo
A(r"""\begin{table}[H]
\centering\scriptsize
\caption{J5b demonstration, Q2 at 0.13\,Mbit/s with pace $0.75c=97.5$\,kbit/s, below the scan load (exploratory; means of
five replays; frozen link-model predictions in brackets).}\label{tab:s-j5bq2}
\begin{tabular}{@{}lccc@{}}
\toprule
Scheduler & $a_{\text{nav}}$ (s) [pred.\ $a_{\text{link}}$] & $F$ (fps) [pred.] & Viol. (s) [pred.] \\
\midrule""")
for arm in ('B0', 'B3', 'B7', 'MAKL'):
    r = next(x for x in CB if x['cell'] == 'Q2' and x['mode'] == 'NP75' and x['arm'] == arm)
    key = f'Q2|NP75|{arm}'
    A(f"{NAME[arm]} & {v(r,'anav_p95_s')} [{pmean(key,'alink_p95_s'):.2f}] & {v(r,'F_fps')} [{pmean(key,'F_fps'):.2f}] & {v(r,'viol_s',1)} [{pmean(key,'viol_s'):.1f}] \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}
As predicted, FIFO ran away and the keep-last schedulers sent scans that replaced each other. The kernel queue was not
short (one-way delay 100--144\,ms at the median, about 0.85\,s at the 99th percentile; backlog above 3\,kB for 55--60\,\% of
$W$ at a load of 0.93--0.95$c$). In Q1, mode A again gave a one-way delay of 5.7\,ms (p50) and 16.3\,ms (p99), and MAKL's
decision on the Pi took 0.78\,ms (p50) and at most 1.20\,ms (p99). One Q2 FIFO replay lost one of 257 messages (best
effort); the sender again aborted on exit after the run.
""")
open(OUT, 'w').write('\n'.join(L) + '\n')
print('ok')
