"""Write ESM Section S16 (J3, distance-switch ablation) from the frozen J3 outputs.
Usage: python3 gen_esm_j3.py DATA_DIR OUT.tex"""
import csv, json, sys, statistics as st
from pathlib import Path

D = Path(sys.argv[1]) / 'J3/final'; OUT = Path(sys.argv[2])
V = 'W_viol_anav_unom_b0.2_s'
f2 = lambda x: f'{float(x):.2f}'
cells = {(r['cell'], r['config']): r for r in csv.DictReader(open(D / 'J3_CELLS.csv'))}
runs = list(csv.DictReader(open(D / 'J3_RUNS.csv')))
tests = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(D / 'J3_TESTS.csv'))}
CELL = (('LB-Q1', 'Q1, 0.18\\,Mbit/s'), ('LC-Q1', 'Q1, 0.13\\,Mbit/s'), ('LC-Q2', 'Q2, 0.13\\,Mbit/s'))
ARMS = (('B7', 'B7'), ('MAKL', 'MAKL'), ('DSW_0.8', 'DSW 0.8\\,m'), ('DSW_1.0', 'DSW 1.0\\,m'), ('DSW_1.2', 'DSW 1.2\\,m'))


def share(c, a):
    rows = [r for r in runs if r['cell'] == c and r['config'] == a]
    if a == 'MAKL':
        return st.mean(float(r['makl_cam_share_W']) for r in rows), None
    if a.startswith('DSW'):
        cam = st.mean(float(r['dsw_cam_chosen_W']) / float(r['dsw_decisions_W']) for r in rows)
        return cam, st.mean(float(r['dsw_scan_first_share_W']) for r in rows)
    return None, None


L = []; w = L.append
w(r"""\section{J3: is the state-dependent budget needed?}\label{sec:s-j3}
J3 (package \texttt{SCI\_J3\_SWITCH\_ABLATION}; protocol frozen on 16 November 2026) compared MAKL with a distance
switch (DSW): depth-one queues, probes first, the scan first when the raw LiDAR distance $d$ is below a fixed
threshold, and otherwise the older head first. DSW uses the same $d$ as MAKL but no speed and no capacity. The
thresholds 0.8, 1.0 and 1.2\,m were fixed before the freeze from an exploratory open-loop replay of the recorded
traces, in which 1.0\,m came closest to MAKL over all replayed capacities, including Q1 at 0.13\,Mbit/s. J3 ran five
schedulers $\times$ three 25\,s cells $\times$ five paired start shifts (75 runs) with the safety layer on and the
ideal link. All 75 runs succeeded, with no retries and no contacts (time to goal 18.7--20.2\,s, clearance at least
0.50\,m). The 22 offline tests and both gate trials passed, and the frozen analysis, re-run independently from the
raw run files, gave byte-identical outputs. Outcome rule: \emph{supported} if H3 holds and H2a or H2b holds.
""")
w(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{4pt}
\caption{J3 cell means $\pm$ SD (five runs; columns as in Table~\ref{tab:s-j1cells}). Camera share: share of the
scheduler's decisions in $W$ that sent a camera frame. Scan-first share (DSW only): share of decisions taken with
$d$ below the threshold.}\label{tab:s-j3cells}
\begin{tabular}{@{}llccccccc@{}}
\toprule
Cell & Sched. & Viol. (s) & $F$ (fps) & $a_{\text{nav}}$ p95 (s) & P1 (s) & Clear. (m) & Camera share & Scan-first share \\
\midrule""")
for k, (c, cname) in enumerate(CELL):
    if k:
        w(r'\midrule')
    for i, (a, aname) in enumerate(ARMS):
        r = cells[(c, a)]
        cam, sf = share(c, a)
        w(f"{cname if i == 0 else ''} & {aname} & {f2(r[V])} $\\pm$ {f2(r[V + '_sd'])} & {f2(r['F_fps'])} $\\pm$ {f2(r['F_fps_sd'])} & "
          f"{f2(r['a_nav_p95_clipped_s'])} & {float(r['P1_s']):.1f} & {f2(r['W_min_clearance_m'])} & "
          f"{'--' if cam is None else f2(cam)} & {'--' if sf is None else f2(sf)} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}
""")
LAB = {
    'H1a_LC-Q1_MAKL_gt_B7_F': ('H1', 'Q1, 0.13', 'MAKL $>$ B7, $F$', 'fps'),
    'H1b_LC-Q1_MAKL_gt_B7_viol': ('H1', 'Q1, 0.13', 'MAKL $>$ B7, violation (must not be clear)', 's'),
    'H2a_LC-Q1_DSW0.8_gt_MAKL_viol': ('H2a', 'Q1, 0.13', 'DSW 0.8 $>$ MAKL, violation', 's'),
    'H2b_LB-Q1_MAKL_gt_DSW1.2_F': ('H2b', 'Q1, 0.18', 'MAKL $>$ DSW 1.2, $F$', 'fps'),
    'H2b_LC-Q1_MAKL_gt_DSW1.2_F': ('H2b', 'Q1, 0.13', 'MAKL $>$ DSW 1.2, $F$', 'fps'),
    'H3_LB-Q1_MAKL_gt_DSW1.0_viol': ('H3', 'Q1, 0.18', 'MAKL $>$ DSW 1.0, violation (must not be clear)', 's'),
    'H3_LB-Q1_DSW1.0_gt_MAKL_F': ('H3', 'Q1, 0.18', 'DSW 1.0 $>$ MAKL, $F$ (must not be clear)', 'fps'),
    'H3_LC-Q1_MAKL_gt_DSW1.0_viol': ('H3', 'Q1, 0.13', 'MAKL $>$ DSW 1.0, violation (must not be clear)', 's'),
    'H3_LC-Q1_DSW1.0_gt_MAKL_F': ('H3', 'Q1, 0.13', 'DSW 1.0 $>$ MAKL, $F$ (must not be clear)', 'fps'),
    'H3_LC-Q2_MAKL_gt_DSW1.0_viol': ('H3', 'Q2, 0.13', 'MAKL $>$ DSW 1.0, violation (must not be clear)', 's'),
    'H3_LC-Q2_DSW1.0_gt_MAKL_F': ('H3', 'Q2, 0.13', 'DSW 1.0 $>$ MAKL, $F$ (must not be clear)', 'fps'),
}
w(r"""\begin{table}[H]
\centering\scriptsize\setlength{\tabcolsep}{4pt}
\caption{J3 frozen paired tests (five pairs; clear: all pairs positive and mean above $J$).}\label{tab:s-j3tests}
\begin{tabular}{@{}lllcccl@{}}
\toprule
Hyp. & Cell & Comparison & Mean diff. & $J$ & Clear & Paired differences \\
\midrule""")
for t, (h, c, txt, u) in LAB.items():
    x = tests[t]
    w(f"{h} & {c} & {txt} & {x['mean_diff']:+.2f}\\,{u} & {x['J']:.1f} & {'yes' if x['clearly_greater'] else 'no'} & "
      f"{', '.join(f'{d:+.2f}' for d in x['diffs'])} \\\\")
w(r"""\bottomrule
\end{tabular}
\end{table}

Verdicts of the frozen script: H1, H2a and H2b (narrowly, in the cell with Q1 at 0.13\,Mbit/s only), H3 in all three
cells, H4 (no contact while the robot moved) and the anchor R (B7 and MAKL reproduced J1 in the two repeated cells)
held. Outcome: supported. Exploratory: with Q1 at 0.13\,Mbit/s the 1.0\,m switch violated the margin longer than MAKL
in all five pairs (mean $+1.02$\,s); this direction was not a frozen hypothesis.
""")
OUT.write_text('\n'.join(L) + '\n')
print('written', OUT)
