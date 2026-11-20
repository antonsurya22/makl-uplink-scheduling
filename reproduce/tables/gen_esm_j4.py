"""ESM Section S17 (J4 speed ablation) from the frozen J4 outputs. Usage: gen_esm_j4.py J4_CELLS.csv J4_TESTS.csv J4_VERDICTS.json OUT.tex"""
import csv, json, sys

C = list(csv.DictReader(open(sys.argv[1])))
T = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(sys.argv[2]))}
V = json.load(open(sys.argv[3]))
f = lambda x: float(x) if x not in ('', None) else None
NAME = {'B7': 'B7', 'MAKL': 'MAKL', 'MAKL_cap': 'FixedV', 'DSW_1.0': 'DSW 1.0\\,m'}

def pm(r, k, d=2):
    m, s = f(r[k]), f(r[k + '_sd'])
    return '--' if m is None else (f'{m:.{d}f} $\\pm$ {s:.{d}f}' if s is not None else f'{m:.{d}f}')

def v(r, k, d=2):
    m = f(r[k]); return '--' if m is None else f'{m:.{d}f}'

L = []
A = L.append
A(r"""\section{J4: does MAKL's speed term matter?}\label{sec:s-j4}

\paragraph{Design.} Package \texttt{SCI\_J4\_SPEED\_ABLATION\_20261117}, frozen with its runner, metrics and analysis
(SHA256 file \texttt{J4\_FREEZE\_HASHES.json}), and frozen again with amendment A1 before any gate or counted run. Arms:
priority with keep-last (B7), MAKL (J1 parameters), MAKL-FixedV (the P0-J configuration \texttt{MAKL\_cap}: the same code,
with the constant 0.15\,m\,s$^{-1}$ in place of $\hat v$ in $a^\ast$) and the 1.0\,m distance switch. Levels: planner
(DWB) speed caps of 0.10, 0.15 and 0.22\,m\,s$^{-1}$, with the mode-gate cap at or above the level. Cell: Q1 at
0.18\,Mbit/s for 25\,s from the trigger, crossing pedestrian at 0.50\,m\,s$^{-1}$. Five start shifts per level; each block
ran the four arms with congestion in rotated order and one B7 run without congestion (C0): 75 counted runs, plus six
pilots and three gates (not counted). All 75 runs succeeded, with no retries and no robot-moving contact.

\paragraph{Metrics.} Phase A runs from the trigger $t_0$ to the closest approach to the pedestrian $t_{ca}$; phase B
from $t_{ca}$ to the end of $W$ (violation) or to $t_{\text{run}}=\min(\text{goal}, t_0+25\,\text{s})$ (frames). The
primary frame rate $F_{\text{run}}$ counts useful frames over $[t_0,t_{\text{run}})$, so no time after the goal enters
it; $F_W$ is the J1 value over 25\,s. The switching clearance $d_{sw}$ uses qualifying decisions: in phase A, with a scan
and a camera frame waiting, and with $d$ set by the pedestrian ($|d-(\text{clearance}-0.10)|\le0.15$\,m). It is the
largest $d^\ast$ of a qualifying scan-first decision such that the window $[d^\ast-0.15,d^\ast]$ holds at least three
qualifying decisions, at least half of them scan-first; it is missing with fewer than six qualifying decisions.

\paragraph{Decision rules and hypotheses.} J-rule as in J1 (at least four pairs, all positive, mean above $J$), with
$J=\max(\text{J1 Q1 value}, \text{C0 range at the level})$; the J1 values (0.8\,s, 0.4\,fps) applied at every level.
Sign rule (weaker): all pairs have the predicted sign. Mechanism rule for $d_{sw}$: all pairs positive and mean above
0.10\,m. M1: MAKL's $d_{sw}$ at the top level exceeds that at the lowest. M2: MAKL-FixedV's $d_{sw}$ changes by at most
0.10\,m. S0a: MAKL used $\hat v$ and MAKL-FixedV 0.15 in every decision. S1: at the top level MAKL-FixedV's phase-A
violation exceeds MAKL's. S2: at the lowest level MAKL's $F_{\text{run}}$ exceeds MAKL-FixedV's. D1, D2: the same
against the distance switch. H3: MAKL's violation not clearly above B7's at any level. H4: MAKL's $F_{\text{run}}$
clearly above B7's at every level. H5: no robot-moving contact. R: B7 and the switch reproduce J3 at 0.15\,m\,s$^{-1}$.
Outcomes: speed term supported if M1, M2 and (S1 or S2) hold under the J-rule; mechanism only if S1 or S2 hold under the
sign rule only; distance-switch advantage if H3 and (D1 or D2) hold under the J-rule.

\paragraph{Prediction (fixed before the runs).} From the archived J1 MAKL decisions, the predicted peak age of contested
decisions had a median of about 0.68\,s. Solving $a^\ast(v,d)=0.68+m$ gives switching clearances of 0.80, 1.00 and
1.31\,m for MAKL at 0.10, 0.15 and 0.22\,m\,s$^{-1}$, and 1.00\,m for MAKL-FixedV at every level. In raw range ($d+0.22$\,m)
all lie above the shield thresholds (0.70, 0.77 and 0.96\,m).

\paragraph{Amendment A1 (before any gate or counted run).} The six C0 pilots under the first freeze all succeeded, but
the pilot rule halted J4: the speed-window check (P3) failed in 4 of 6 runs and the two safety-tick checks (P4, P5) in 6 of
6 and 5 of 6. The stored logs showed that the safety-tick detector flagged every change between DWB's discrete velocity
samples as a guard cut (25--56 flags per run in NORMAL mode, no latch, raw ranges 1.5--1.7\,m), a fault hidden in J1--J3
because the mode gate then clipped every DWB sample to 0.15\,m\,s$^{-1}$. P3 measured the same sampling: the median
speed in the window was 0.090--0.094, 0.137--0.141 and 0.208--0.209\,m\,s$^{-1}$, with no stop before the window. A1
redefined a safety tick (mode not NORMAL, guard latched, or a zero guard output on two ticks while the gated command is
above 0.02\,m\,s$^{-1}$) and P3 (median speed within 0.02\,m\,s$^{-1}$ of the level, no tick below half the level, at least
five ticks, planner command at most the level plus 0.01). No scheduler, setting, hypothesis, metric of a hypothesis,
decision rule or outcome changed. The six stored pilots were re-evaluated without re-running and all passed
(Table~\ref{tab:s-j4pilots}).

\begin{table}[H]
\centering\footnotesize
\caption{J4 pilots re-evaluated under amendment A1 (C0, MAKL).}\label{tab:s-j4pilots}
\begin{tabular}{@{}lccc@{}}
\toprule
Pilot (level, shift) & Speed in window: median / min (m\,s$^{-1}$) & First safety tick, raw range & Wall ticks \\
\midrule
0.10, $\delta=-0.4$ & 0.090 / 0.067 & none & 0 \\
0.10, $\delta=+0.4$ & 0.094 / 0.074 & 0.68\,m & 0 \\
0.15, $\delta=-0.4$ & 0.137 / 0.119 & 0.71\,m & 0 \\
0.15, $\delta=+0.4$ & 0.141 / 0.127 & 0.72\,m & 0 \\
0.22, $\delta=-0.4$ & 0.209 / 0.196 & 0.83\,m & 0 \\
0.22, $\delta=+0.4$ & 0.209 / 0.193 & 0.91\,m & 0 \\
\bottomrule
\end{tabular}
\end{table}

\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{4pt}
\caption{J4 speed levels as measured, and switching clearance $d_{sw}$ (means of five runs) against the prediction
fixed before the runs and against the model at the measured speed input.}\label{tab:s-j4speed}
\begin{tabular}{@{}lccccc@{}}
\toprule
Level (cap) & Robot speed, approach & MAKL $\hat v$ (median in $W$) & Moving part of $W$ & MAKL $d_{sw}$ (pred.; model) & FixedV $d_{sw}$ \\
\midrule""")
lv = [('V010', '0.10', '0.09', '0.095', '25.0\\,s', '0.80; 0.78'), ('V015', '0.15', '0.14', '0.142', '20.4--20.8\\,s', '1.00; 0.97'),
      ('V022', '0.22', '0.21', '0.215', '14.7--15.0\\,s', '1.31; 1.28')]
cell = {}
for r in C:
    cell.setdefault((r['cell'], r['config']), r)
for tag, cap, rs, vh, mv, pr in lv:
    m = cell[(f'{tag}|LB-Q1', 'MAKL')]; fx = cell[(f'{tag}|LB-Q1', 'MAKL_cap')]
    A(f"{cap}\\,m\\,s$^{{-1}}$ & {rs}\\,m\\,s$^{{-1}}$ & {vh}\\,m\\,s$^{{-1}}$ & {mv} & {v(m,'d_sw_m')}\\,m ({pr}) & {v(fx,'d_sw_m')}\\,m \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}

\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{3.5pt}
\caption{J4 cell means $\pm$ SD (five runs; Q1 at 0.18\,Mbit/s, 25\,s; C0: B7 without congestion). Viol.: age--speed
violation time in $W$ (pedestrian at 0.50\,m\,s$^{-1}$), split into phase A (approach) and B (after the closest
approach). $F_{\text{run}}$: useful frames per second over the moving part of $W$, split into A and B; $F_W$: over 25\,s.
$d_{sw}$: switching clearance (the switch's value is not meaningful, see text). Clear.: minimum clearance.}\label{tab:s-j4cells}
\begin{tabular}{@{}llccccccccc@{}}
\toprule
Level & Sched. & Viol. (s) & Viol. A & Viol. B & $F_{\text{run}}$ (fps) & $F$ A & $F$ B & $F_W$ & $d_{sw}$ (m) & Clear. (m) \\
\midrule""")
for i, (tag, cap, *_ ) in enumerate(lv):
    if i: A(r"\midrule")
    first = True
    for cfg in ('B7', 'MAKL', 'MAKL_cap', 'DSW_1.0'):
        r = cell[(f'{tag}|LB-Q1', cfg)]
        nm = r'\textbf{MAKL}' if cfg == 'MAKL' else NAME[cfg]
        A(f"{cap + ' m/s' if first else ''} & {nm} & {pm(r,'W_viol_anav_unom_b0.2_s')} & {v(r,'viol_A')} & {v(r,'viol_B')} & "
          f"{pm(r,'F_run')} & {v(r,'F_A')} & {v(r,'F_B')} & {v(r,'F_fps')} & {v(r,'d_sw_m') if cfg != 'B7' else '--'} & {v(r,'W_min_clearance_m')} \\\\")
        first = False
    r = cell[(f'{tag}|C0-Q1', 'B7')]
    A(f" & B7, C0 & {pm(r,'W_viol_anav_unom_b0.2_s')} & {v(r,'viol_A')} & {v(r,'viol_B')} & {pm(r,'F_run')} & {v(r,'F_A')} & {v(r,'F_B')} & {v(r,'F_fps')} & -- & {v(r,'W_min_clearance_m')} \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}

\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{2.2pt}
\caption{J4 frozen paired tests (five pairs). Rule: J = J-rule (all pairs positive, mean above $J$); sign = sign rule;
mech. = mechanism rule (mean above 0.10\,m); M2 passes if the mean change is at most 0.10\,m. CI: 95\,\% bootstrap
interval of the mean. Rows marked $\dagger$ are reported only.}\label{tab:s-j4tests}
\begin{tabular}{@{}llccccl@{}}
\toprule
Hyp. & Comparison & Mean diff. & CI & Threshold & Result & Paired differences \\
\midrule""")
def fm(x, d=2): return f'{x:+.{d}f}'
rows = [
 ('M1', 'MAKL $d_{sw}$, 0.22 minus 0.10', 'M1_MAKL_dsw_top_gt_lo', 'm', '0.10\\,m', lambda t: 'holds (mech.)' if t['clearly_greater'] else 'fails'),
 ('M2', 'FixedV $d_{sw}$, 0.22 minus 0.10', 'M2_FixedV_dsw_top_minus_lo', 'm', '$\\le$0.10\\,m', lambda t: 'holds' if t['mean_diff'] <= t['limit'] else 'fails'),
 ('$\\dagger$', 'MAKL minus FixedV $d_{sw}$ at 0.22', 'M_extra_MAKL_dsw_top_minus_FixedV_dsw_top', 'm', '--', lambda t: 'all positive' if t['all_predicted_sign'] else 'not all positive'),
 ('$\\dagger$', 'FixedV minus MAKL $d_{sw}$ at 0.10', 'M_extra_FixedV_dsw_lo_minus_MAKL_dsw_lo', 'm', '--', lambda t: 'all positive' if t['all_predicted_sign'] else 'not all positive'),
 ('S1', 'FixedV $>$ MAKL, viol. A, 0.22', 'S1_top_FixedV_gt_MAKL_violA', 's', '0.8\\,s', None),
 ('S2', 'MAKL $>$ FixedV, $F_{\\text{run}}$, 0.10', 'S2_lo_MAKL_gt_FixedV_Frun', 'fps', '0.4\\,fps', None),
 ('D1', 'DSW $>$ MAKL, viol. A, 0.22', 'D1_top_DSW_gt_MAKL_violA', 's', '0.8\\,s', None),
 ('D2', 'MAKL $>$ DSW, $F_{\\text{run}}$, 0.10', 'D2_lo_MAKL_gt_DSW_Frun', 'fps', '0.4\\,fps', None),
]
for lvl in ('V010', 'V015', 'V022'):
    cap = {'V010': '0.10', 'V015': '0.15', 'V022': '0.22'}[lvl]
    rows.append(('H3', f'MAKL $>$ B7, viol., {cap} (not clear)', f'H3_{lvl}_MAKL_gt_B7_violW', 's', '0.8\\,s', None))
    rows.append(('H4', f'MAKL $>$ B7, $F_{{\\text{{run}}}}$, {cap}', f'H4_{lvl}_MAKL_gt_B7_Frun', 'fps', '0.4\\,fps', None))
for hyp, comp, key, unit, thr, fn in rows:
    t = T[key]
    if fn is None:
        res = 'clear' if t.get('clearly_greater') else ('sign only' if t.get('sign_rule') else 'not clear')
    else:
        res = fn(t)
    ci = f"{t['ci_lo']:.2f}--{t["ci_hi"]:.2f}" if 'ci_lo' in t else '--'
    d = ', '.join(fm(x) for x in t['diffs'])
    A(f"{hyp} & {comp} & {fm(t['mean_diff'])}\\,{unit} & {ci} & {thr} & {res} & {d} \\\\")
A(r"""\bottomrule
\end{tabular}
\end{table}
""")
A(f"""\\paragraph{{Verdicts of the frozen script.}} Speed term: supported (M1, M2, S0a, and S2 under the J-rule; S1 under the
sign rule only). Distance switch: advantage (H3, and D2 under the J-rule; D1 failed the sign rule, one pair $-0.1$\\,s).
H3 held at every level, so the degraded-trade-off outcome did not occur. H4 held at every level, at 0.22\\,m\\,s$^{{-1}}$
only just ($+0.41$\\,fps against 0.40). H5: no robot-moving contact; no safety-layer action at walls. R: B7 and the 1.0\\,m
switch reproduced J3 at 0.15\\,m\\,s$^{{-1}}$ (violation 1.04 and 1.20\\,s against 1.24 and 1.38\\,s; $F_W$ 2.36 and
2.99\\,fps against 2.27 and 3.09\\,fps). S0b (reported only): at 0.15\\,m\\,s$^{{-1}}$ MAKL and MAKL-FixedV made the same
choice in {V['S0b_mean_agreement']*100:.1f}\\,\\% of decisions (target 95\\,\\%), as A1 anticipated, because MAKL's input there
was 0.142 rather than 0.15\\,m\\,s$^{{-1}}$.

\\paragraph{{Where the frame differences arose.}} At 0.10\\,m\\,s$^{{-1}}$ MAKL's gain over MAKL-FixedV was
{V['phase_of_frame_differences']['S2']['F_A']['mean_diff']:+.2f}\\,fps in phase A and
{V['phase_of_frame_differences']['S2']['F_B']['mean_diff']:+.2f}\\,fps in phase B: after the encounter, walls set $d$ at
about 0.8--0.95\\,m, below FixedV's switching value (1.0\\,m) but above MAKL's (0.78\\,m). Its gain over the switch was
{V['phase_of_frame_differences']['D2']['F_A']['mean_diff']:+.2f}\\,fps in A and
{V['phase_of_frame_differences']['D2']['F_B']['mean_diff']:+.2f}\\,fps in B. At 0.22\\,m\\,s$^{{-1}}$ MAKL delivered
{V['phase_of_frame_differences']['S_top']['F_run']['mean_diff']:+.2f}\\,fps relative to MAKL-FixedV
({V['phase_of_frame_differences']['S_top']['F_A']['mean_diff']:+.2f} in A, {V['phase_of_frame_differences']['S_top']['F_B']['mean_diff']:+.2f}
in B), the cost of switching earlier. At 0.22\\,m\\,s$^{{-1}}$ every congested arm violated the margin for 4.4--4.8\\,s, mostly in
phase B, and B7 for 1.3\\,s without congestion: at this speed $a^\\ast$ is zero within about 0.74\\,m of an obstacle
closing at 0.5\\,m\\,s$^{{-1}}$ (Table~\\ref{{tab:s-amax}}), and the metric treats walls as such obstacles.

\\paragraph{{Notes.}} The switching clearance of the distance switch is not meaningful: above its threshold it sends the
older head first, so about half of its decisions there are scan-first by age, and the rule reads 1.15--1.41\\,m; it
enters no hypothesis. Exploratory, across experiments: at the middle level MAKL's speed input fell from about
0.17\\,m\\,s$^{{-1}}$ (J3, planner cap 0.18) to 0.142\\,m\\,s$^{{-1}}$ (J4, planner cap 0.15); its $F_W$ rose from 3.18 to
3.50\\,fps with a similar violation (1.24 and 1.02\\,s), while B7 and the switch, which do not use speed, reproduced J3.
The planner cap also changes the robot's motion, so this comparison is not a test.
""")
open(sys.argv[4], 'w').write('\n'.join(L) + '\n')
print('ok')
