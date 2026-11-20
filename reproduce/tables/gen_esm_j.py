"""Write the J-experiment sections of the supplementary material (S13-S15) from the frozen outputs.
Every number in a generated table comes from the CSV/JSON files in data/J1/final and data/J2/final.
The P0-J/J0 numbers (S13) are copied from J0_EXPLORATORY_REPORT_20261112.md (exploratory replay).
Usage: python3 gen_esm_j.py DATA_DIR OUT.tex"""
import csv, json, sys
from pathlib import Path

D = Path(sys.argv[1]); OUT = Path(sys.argv[2])
V = 'W_viol_anav_unom_b0.2_s'
f2 = lambda x: f'{float(x):.2f}'
f3 = lambda x: f'{float(x):.3f}'
c1 = list(csv.DictReader(open(D / 'J1/final/J1_CELLS.csv')))
p1 = list(csv.DictReader(open(D / 'J1/final/J1_PAIRED.csv')))
c2 = list(csv.DictReader(open(D / 'J2/final/J2_CELLS.csv')))
t2 = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(D / 'J2/final/J2_TESTS.csv'))}
k2 = {(r['profile_cell'], r['condition']): r for r in csv.DictReader(open(D / 'J2/final/J2_RANKS.csv'))}
v2 = json.load(open(D / 'J2/final/J2_VERDICTS.json'))
ARMS1 = ('B0', 'B1', 'B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
ARMS2 = ('B2', 'B3', 'B4', 'B5', 'B7', 'MAKL')
CELL1 = {'LC-Q2-25s': 'Q2, 0.13\\,Mbit/s, 25\\,s', 'LC-Q2-8s': 'Q2, 0.13\\,Mbit/s, 8\\,s',
         'LB-Q1-25s': 'Q1, 0.18\\,Mbit/s, 25\\,s', 'LB-Q1-8s': 'Q1, 0.18\\,Mbit/s, 8\\,s',
         'C0-Q2': 'C0, Q2', 'C0-Q1': 'C0, Q1'}
COND = (('none', 'ideal'), ('jit10', 'jitter $\\pm10$\\,ms'), ('jit50', 'jitter $\\pm50$\\,ms'), ('loss1', 'loss 1\\,\\%'),
        ('loss5', 'loss 5\\,\\%'), ('ge', 'GE burst loss'), ('capsq', 'capacity $\\pm54$\\,\\%'))
PC = (('LC-Q2', 'Q2, 0.13\\,Mbit/s'), ('LB-Q1', 'Q1, 0.18\\,Mbit/s'))
L = []
w = L.append

# ------------------------------------------------------------------ S13
w(r"""\section{Implementation checks and offline replay (P0-J, J0)}\label{sec:s-j0}
\paragraph{Implementation checks.} Before J0, the new schedulers (B4, B5, B7 and MAKL) were added to the frozen
bridge as new queue disciplines and tested on the simulation computer (package \texttt{SCI\_P0J\_SCHEDULERS}).
Table~\ref{tab:s-p0j} lists the checks. Test T5 compared the new replay code with the frozen replay harness of the
earlier offline replay (Section~S6). Part (a) matched record by record on all six E3-B traces (48 replays, 17,088
records). Part (b) compared the archived summary file of that harness: 27,030 of 27,216 values were reproduced
exactly; the other 186 were scan-age or delay values of the third repetition only and differed by exactly
$+0.004$\,s. The frozen harness, run again on the same exported traces, gave the same values as the new code, so the
difference lies in the exported trace files of that repetition, not in the code. We kept T5 as failed in the record
and used the record-level comparison (part a) as the equivalence check for J1.

\begin{table}[H]
\centering\small
\caption{Implementation checks before J0 (all on the simulation computer).}\label{tab:s-p0j}
\begin{tabular}{@{}p{8.6cm}p{6cm}@{}}
\toprule
Check & Result \\
\midrule
Package hashes; copies of the frozen files equal the original freezes (6 of 6) & pass \\
T1: closed form of $a^\ast(v,d)$ equals Table~\ref{tab:s-amax} (36 cells) & pass \\
T2: per-stream ledger conservation (15 configurations $\times$ 4 seeds) & pass \\
T3: Proposition~1 on synthetic traffic (worst case equals $a_{\min}$ to within 1\,ms) & pass \\
T3b: Proposition~2 on synthetic traffic (tight) & pass \\
T4: MAKL limits ($\beta=0$ gives B7; $\beta=\infty$ gives camera-first keep-last) & pass \\
T5: replay equivalence with the frozen replay harness & part (a) pass; part (b) 186 of 27,216 values differ by 4\,ms \\
T6: decision time of MAKL & median 14.2--14.5\,$\mu$s, 99th percentile 46--54\,$\mu$s \\
T7: bridge interface & pass \\
\bottomrule
\end{tabular}
\end{table}

\paragraph{Offline replay J0 (exploratory).} J0 replayed recorded traces (72 E3-B, 180 E5 and 48 E7 runs) through
every scheduler. It is open loop: the robot's trajectory is the recorded one and does not react to the data age. A
first version cut the replay 5\,s after the end of the event, which ended the 8\,s cells inside the window $W$; those
cells were invalid and were recomputed with a corrected horizon (the 25\,s cells were identical in both versions).
Table~\ref{tab:s-j0cal} compares the replay with the closed-loop E6 measurements, Table~\ref{tab:s-j0} gives the
25\,s cells, and Table~\ref{tab:s-j0dec} the design choices that were frozen before J1. No J0 number is a counted
result.

\begin{table}[H]
\centering\small
\caption{J0 calibration: replay of the E6 traces against the closed-loop E6 measurements (Q2, 0.13\,Mbit/s, 25\,s,
pedestrian 0.5\,m\,s$^{-1}$; replay / measured).}\label{tab:s-j0cal}
\begin{tabular}{@{}lccc@{}}
\toprule
Scheduler & $a_{\text{nav}}$ p95 (s) & Violation (s) & $F$ (fps) \\
\midrule
B0 FIFO & 4.68 / 4.67 & 13.28 / 13.37 & 0.26 / 0.28 \\
B1 FIFO + expiry & 1.30 / 2.03 & 13.28 / 13.27 & 0.26 / 0.28 \\
B2 Priority & 0.67 / 0.81 & 1.55 / 1.50 & 0.01 / 0.013 \\
B3 Keep-last & 0.80 / 0.76 & 2.44 / 2.97 & 1.56 / 1.57 \\
\bottomrule
\end{tabular}
\end{table}

\begin{table}[H]
\centering\small
\caption{J0 replay, 25\,s events, pedestrian 0.5\,m\,s$^{-1}$ (means over three E5 traces per profile).
Violation floor without congestion: 0.73\,s (Q1) and 0.70\,s (Q2).}\label{tab:s-j0}
\begin{tabular}{@{}lcccccc@{}}
\toprule
 & \multicolumn{3}{c}{Q1, 0.18\,Mbit/s} & \multicolumn{3}{c}{Q2, 0.13\,Mbit/s} \\
Scheduler & $a_{\text{nav}}$ p95 & Viol. & $F$ & $a_{\text{nav}}$ p95 & Viol. & $F$ \\
\midrule
B0 & 1.34 & 13.30 & 0.55 & 4.73 & 13.40 & 0.23 \\
B1 & 1.34 & 13.30 & 0.55 & 1.30 & 13.37 & 0.23 \\
B2 & 0.77 & 1.40 & 0.17 & 0.66 & 1.67 & 0.00 \\
B3 & 0.84 & 3.13 & 2.89 & 0.83 & 2.63 & 1.56 \\
B4 & 0.84 & 3.87 & 2.65 & 0.90 & 5.07 & 1.56 \\
B5 & 0.77 & 2.70 & 3.65 & 0.83 & 2.63 & 1.56 \\
B7 & 0.77 & 1.73 & 2.24 & 0.73 & 2.00 & 0.44 \\
MAKL & 0.84 & 1.77 & 2.88 & 0.76 & 1.97 & 0.72 \\
\bottomrule
\end{tabular}
\end{table}

\begin{table}[H]
\centering\small
\caption{MAKL design choices from J0, frozen before J1.}\label{tab:s-j0dec}
\begin{tabular}{@{}p{2.2cm}p{8.2cm}p{3.6cm}@{}}
\toprule
Item & J0 evidence & Choice \\
\midrule
Test & a test on the age of the waiting scan let the scan age run away at or below 0.13\,Mbit/s ($a_{\text{nav}}$ p95 1.9--4.6\,s with Q1) & peak-age test, Eq.~(11) \\
$u$ & $u=0.08$\,m\,s$^{-1}$ made MAKL serve the camera first (violation 5--14\,s) & $u=0.5$\,m\,s$^{-1}$ \\
$m$ & $m=0$: same violation in most cells, 3--18\,\% more camera frames, about 0.1\,s more violation with Q1 at or below 0.15\,Mbit/s & $m=0.1$\,s \\
$\hat v$ & odometry only was slightly better in the replay, which cannot show a restart & $\max(v_{\mathrm{odom}},v_{\mathrm{cmd}})$ \\
$d$ & $d$ from moving obstacles only: 8--30\,\% more camera frames at the same violation, but needs obstacle segmentation & raw 360$^\circ$ LiDAR (future work) \\
$\hat c$ & an oracle capacity changed nothing & delivered rate over 2\,s \\
\bottomrule
\end{tabular}
\end{table}
""")

# ------------------------------------------------------------------ S14
w(r"""\section{J1: MAKL against the baselines in the closed loop}\label{sec:s-j1}
J1 (package \texttt{SCI\_J1\_SCHEDULERS\_V2}; protocol frozen on 12 November 2026 with amendment A1, which fixed an
error in the gate code of a first version that stopped at its first gate and is not counted) ran 240 runs: eight
schedulers $\times$ six cells $\times$ five start shifts. All 240 runs succeeded, with no contacts and no emergency
stops. The frozen analysis was re-run independently from the raw run files and gave identical verdicts, cell rows and
run rows; 2,400 per-run values were recomputed from the raw files without a mismatch. Table~\ref{tab:s-j1cells}
gives all cell means, Table~\ref{tab:s-j1tests} every frozen paired test, and Table~\ref{tab:s-j1makl} MAKL's
behaviour.
""")
w(r"""\begin{table}[H]
\centering\footnotesize
\caption{J1 cell means $\pm$ SD (five runs per cell). Viol.: age--speed violation time in $W$ (nominal pedestrian
speed, $b=0.2$\,m\,s$^{-2}$). $F$: useful camera frames per second. $V$: share of $W$ with camera AoI above 1\,s.
$a_{\text{nav}}$ p95: clipped at the end of the run. P1: time from the trigger to the goal. Clear.: minimum clearance
in $W$.}\label{tab:s-j1cells}
\begin{tabular}{@{}llcccccc@{}}
\toprule
Cell & Sched. & Viol. (s) & $F$ (fps) & $V$ & $a_{\text{nav}}$ p95 (s) & P1 (s) & Clear. (m) \\
\midrule""")
cells1 = {(r['cell'], r['config']): r for r in c1}
for k, (cell, name) in enumerate(CELL1.items()):
    if k:
        w(r'\midrule')
    for i, a in enumerate(ARMS1):
        r = cells1[(cell, a)]
        w(f"{name if i == 0 else ''} & {a} & {f2(r[V])} $\\pm$ {f2(r[V + '_sd'])} & {f2(r['F_fps'])} $\\pm$ {f2(r['F_fps_sd'])} & "
          f"{f2(r['V'])} & {f2(r['a_nav_p95_clipped_s'])} & {float(r['P1_s']):.1f} & {f2(r['W_min_clearance_m'])} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}
""")
NAMEH = {'H1': 'H1: MAKL $>$ B7, violation', 'H1b': 'H1b: B3 $>$ MAKL, violation', 'H2a': 'H2a: MAKL $>$ B7, $F$',
         'H2b': 'H2b: MAKL $>$ B7, $F$', 'H3': 'H3: B4/B5 $>$ MAKL, violation'}
w(r"""\begin{table}[H]
\centering\footnotesize
\caption{J1 frozen paired tests (five pairs). A difference is clear if all pairs have the same sign and the mean
exceeds $J$. H1 passes if MAKL's difference to B7 is \emph{not} clearly positive (non-inferiority margin $J$). H4
(time to goal and clearance of MAKL against its own C0 runs, 16 tests) was not clear in any cell and is omitted.}\label{tab:s-j1tests}
\begin{tabular}{@{}llcccl@{}}
\toprule
Test & Cell & Mean diff. & $J$ & Clear & Paired differences \\
\midrule""")
for r in p1:
    t = r['test'].split('_')[0]
    if t == 'H4':
        continue
    lab = NAMEH[t]
    if t == 'H3':
        lab = f"H3: {r['a']} $>$ MAKL, violation"
    unit = 'fps' if r['metric'] == 'F_fps' else 's'
    w(f"{lab} & {CELL1[r['cell']]} & {float(r['mean_diff']):+.2f}\\,{unit} & {float(r['J']):.1f} & "
      f"{'yes' if r['clearly_greater'] == 'True' else 'no'} & {r['diffs'].replace('[', '').replace(']', '')} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}

Verdicts of the frozen script: H1 passed in all four congested cells; H1b failed; H2a and H2b passed; H3 failed
(clear only for B4 with Q2 at 0.13\,Mbit/s); H4a (no contact while the robot moved) and H4b passed; the anchor R
passed (B0--B3 reproduced the E6 scan age within $\pm0.5$\,s); and the bound check held (B7's clipped
$a_{\text{nav}}$ p95 0.578 and 0.478\,s against $a_{\min}=0.575$ and 0.491\,s plus the pipeline offset measured at
C0, 0.040 and 0.013\,s). Outcome: MAKL supported.
""")
w(r"""\begin{table}[H]
\centering\footnotesize
\caption{MAKL behaviour in J1 (means of five runs). Camera share: share of MAKL's decisions in $W$ that sent a camera
frame. CPU: thread CPU time per decision in the live bridge, including writing the decision log.}\label{tab:s-j1makl}
\begin{tabular}{@{}lccccc@{}}
\toprule
Cell & Camera share & Median $\beta$ (s) & Scans replaced & CPU median ($\mu$s) & CPU p99 ($\mu$s) \\
\midrule""")
for cell, name in CELL1.items():
    r = cells1[(cell, 'MAKL')]
    w(f"{name} & {f2(r['makl_cam_share_W'])} & {f2(r['makl_beta_med_W_s'])} & {float(r['L_scan_replaced']):.1f} & "
      f"{float(r['makl_cputhread_med_us']):.0f} & {float(r['makl_cputhread_p99_us']):.0f} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}
""")

# ------------------------------------------------------------------ S15
cells2 = {(r['profile_cell'], r['condition'], r['config']): r for r in c2}
w(r"""\section{J2: robustness to link impairments}\label{sec:s-j2}
J2 (package \texttt{SCI\_J2\_LINK\_REALISM}; protocol frozen on 14 November 2026) repeated the two 25\,s cells of J1
under seven link conditions with six schedulers and three paired start shifts (252 runs). Every random draw depended
only on the run seed and the packet's identity, so the six schedulers of a block faced the same channel. Offline tests
(38 of 38) and both gate trials passed, and the frozen analysis, re-run independently from the raw run files, gave
byte-identical outputs. The impairments were active as designed: every run's bridge configuration matched its
condition, each block used one seed, and the measured packet loss was 1.3\,\% (0.7--1.8\,\%) for 1\,\% loss,
5.4\,\% (3.2--7.2\,\%) for 5\,\% loss and 4.8\,\% (1.5--7.5\,\%) for burst loss (designed mean 3.1\,\%; only six
burst-loss timelines were drawn). The mean absolute jitter was 5.0 and 20.5\,ms, and the capacity changed 23 times in
every run with varying capacity.

Of the 252 runs, 251 succeeded. In one keep-last run (Q1, varying capacity, second shift) the test pilot stopped the
robot 4.4\,s after the trigger because the raw scan reached it more than 0.5\,s late in wall-clock time, a fault of
the simulation computer and not of the emulated link. As frozen, the run is kept in the cell means (violation
0.0\,s, 0.60 useful frames per second). One MAKL run needed an allowed retry before the robot moved.
Table~\ref{tab:s-j2tests} gives the frozen tests per cell and the rank stability, and Table~\ref{tab:s-j2cells} all
cell means.
""")
w(r"""\begin{table}[H]
\centering\footnotesize
\caption{J2 frozen tests per impaired cell (three pairs). H1: MAKL $-$ B7 violation, passes if not clearly above the
margin (0.7\,s for Q2, 0.8\,s for Q1). H2: MAKL $-$ B7 useful frames, passes if all pairs are positive and the mean
exceeds $J$ (0.2\,fps for Q2, 0.4\,fps for Q1). H3: no scheduler clearly better than MAKL on both metrics. $\tau$:
Kendall rank correlation of the six schedulers against the ideal link (exploratory).}\label{tab:s-j2tests}
\begin{tabular}{@{}llccccccc@{}}
\toprule
Cell & Condition & \multicolumn{2}{c}{H1 violation (s)} & \multicolumn{2}{c}{H2 frames (fps)} & H3 & $\tau$ viol. & $\tau$ $F$ \\
 & & mean & pass & mean & pass & pass & & \\
\midrule""")
for i, (pc, pname) in enumerate(PC):
    if i:
        w(r'\midrule')
    for j, (cond, cname) in enumerate(COND[1:]):
        h1 = t2[f'H1_{pc}|{cond}_MAKL_gt_B7_viol']; h2 = t2[f'H2_{pc}|{cond}_MAKL_gt_B7_F']
        h3 = t2[f'H3_{pc}|{cond}_MAKL_clearly_dominated_by']
        kr = k2[(pc, cond)]
        h1p = 'yes' if v2['H1_by_cell'][pc + '|' + cond] else 'no'
        h2p = 'yes' if v2['H2_by_cell'][pc + '|' + cond] else r'\textbf{no}'
        w(f"{pname if j == 0 else ''} & {cname} & {h1['mean_diff']:+.2f} & {h1p} & "
          f"{h2['mean_diff']:+.2f} & {h2p} & "
          f"{'yes' if not h3['arms'] else 'no'} & {float(kr['tau_violation']):.2f} & {float(kr['tau_camera_F']):.2f} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}

Verdicts of the frozen script: H1 passed in 12 of 12 impaired cells, H2 in 11 of 12 (it failed with Q2 under varying
capacity: all three pairs were positive, $+0.28$, $+0.20$ and $+0.04$\,fps, but the mean, $+0.17$\,fps, was below
$J$), H3 in 12 of 12, H4 (no contact while the robot moved) passed, and the anchor R passed (on the ideal link every
scheduler reproduced its J1 result within the frozen tolerance). Outcome: partially robust.
""")
w(r"""{\footnotesize
\setlength{\LTcapwidth}{\textwidth}
\begin{longtable}{@{}llcccccc@{}}
\caption{J2 cell means $\pm$ SD (three runs; columns as in Table~\ref{tab:s-j1cells}). $^\dagger$Includes the run
stopped by the test pilot (0.0\,s, 0.60\,fps); the two completed runs had 1.00\,s and 2.40 and 2.56\,fps.}\label{tab:s-j2cells}\\
\toprule
Condition & Sched. & Viol. (s) & $F$ (fps) & $V$ & $a_{\text{nav}}$ p95 (s) & P1 (s) & Clear. (m) \\
\midrule
\endfirsthead
\toprule
Condition & Sched. & Viol. (s) & $F$ (fps) & $V$ & $a_{\text{nav}}$ p95 (s) & P1 (s) & Clear. (m) \\
\midrule
\endhead
\bottomrule
\endfoot""")
for pc, pname in PC:
    w(f"\\multicolumn{{8}}{{@{{}}l}}{{\\textbf{{{pname}, 25\\,s events}}}} \\\\")
    for cond, cname in COND:
        for i, a in enumerate(ARMS2):
            r = cells2[(pc, cond, a)]
            dag = r'$^\dagger$' if int(r['succeeded']) < int(r['n']) else ''
            p1v = f"{float(r['P1_s']):.1f}" if r['P1_s'] not in ('', 'nan') else '--'
            w(f"{cname if i == 0 else ''} & {a}{dag} & {f2(r[V])} $\\pm$ {f2(r[V + '_sd'])} & {f2(r['F_fps'])} $\\pm$ {f2(r['F_fps_sd'])} & "
              f"{f2(r['V'])} & {f2(r['a_nav_p95_clipped_s'])} & {p1v} & {f2(r['W_min_clearance_m'])} \\\\")
        w(r'\addlinespace')
w(r"""\end{longtable}}
""")
OUT.write_text('\n'.join(L) + '\n')
print('written', OUT, len(L), 'blocks')
