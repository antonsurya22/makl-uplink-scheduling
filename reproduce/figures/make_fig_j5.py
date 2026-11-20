"""Fig. J5: p95 planner scan age (top) and useful camera frames (bottom) by bottleneck mode and arm, cells Q2 and Q1.
v4.8: B3 in purple; Q1 adds the J5b pace 0.75c (NP75, second session). Dots = the 5 traces, bars = mean.
Usage: make_fig_j5.py J5_REPLAYS.csv J5B_REPLAYS.csv OUTDIR"""
import csv, sys, statistics as st
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FixedFormatter, NullLocator
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 7.5, 'axes.linewidth': 0.6, 'axes.spines.top': False,
                     'axes.spines.right': False, 'pdf.fonttype': 42})
R = list(csv.DictReader(open(sys.argv[1])))
R += [r for r in csv.DictReader(open(sys.argv[2])) if r['cell'] == 'Q1' and r['mode'] == 'NP75']
OUT = Path(sys.argv[3])
ORANGE, NAVY, PURPLE, GREY = '#c55a11', '#0b3d75', '#7b4fa6', '#8a8a8a'
ARMS = ('B0', 'B3', 'B7', 'MAKL'); COL = {'B0': GREY, 'B3': PURPLE, 'B7': NAVY, 'MAKL': ORANGE}
MODES = {'Q2': ('A', 'N', 'NP'), 'Q1': ('A', 'N', 'NP', 'NP75')}
MLAB = {'A': 'A: app limit', 'N': 'N: kernel,\nunpaced', 'NP': 'NP: kernel,\npaced 0.9c', 'NP75': 'NP75: kernel,\npaced 0.75c (J5b)'}
CELL = {'Q2': 'Q2, c = 0.13 Mbit/s', 'Q1': 'Q1, c = 0.18 Mbit/s'}
val = lambda c, m, a, k: [float(r[k]) for r in R if r['cell'] == c and r['mode'] == m and r['arm'] == a and r[k] != '']
fig, axs = plt.subplots(2, 2, figsize=(7.1, 4.6), gridspec_kw={'width_ratios': [3, 4]})
w = 0.2
for col, c in enumerate(('Q2', 'Q1')):
    for row, (k, lab) in enumerate((('anav_p95_s', 'p95 scan age at the planner (s)'), ('F_fps', 'Useful camera frames (fps)'))):
        ax = axs[row][col]
        for i, a in enumerate(ARMS):
            for j, m in enumerate(MODES[c]):
                ys = val(c, m, a, k); x = j + (i - 1.5) * w
                ax.bar(x, st.mean(ys), w * 0.86, color=COL[a], alpha=0.85, label=a if (j == 0 and row == 0 and col == 0) else None)
                ax.plot([x] * len(ys), ys, 'o', ms=1.8, color='black', alpha=0.55, mec='none')
        ax.set_xlim(-0.55, len(MODES[c]) - 0.45)
        if row == 0:
            ax.set_xticks(range(len(MODES[c]))); ax.set_xticklabels([])
            ax.set_yscale('log'); ax.set_ylim(0.3, 14)
            ax.yaxis.set_major_locator(FixedLocator([0.5, 1, 2, 5, 10])); ax.yaxis.set_major_formatter(FixedFormatter(['0.5', '1', '2', '5', '10']))
            ax.yaxis.set_minor_locator(NullLocator())
            ax.set_title(f'({"ab"[col]}) {CELL[c]}', loc='left', fontsize=7)
        else:
            ax.set_ylim(0, 3.9); ax.set_title(f'({"cd"[col]}) {CELL[c]}', loc='left', fontsize=7)
            ax.set_xticks(range(len(MODES[c]))); ax.set_xticklabels([MLAB[m] for m in MODES[c]], fontsize=6.2)
        if col == 0:
            ax.set_ylabel(lab, fontsize=7)
        ax.grid(axis='y', lw=0.3, color='#dddddd'); ax.set_axisbelow(True)
axs[0][0].legend(fontsize=6.2, frameon=False, ncol=4, loc='upper left')
fig.tight_layout(); fig.savefig(OUT / 'fig_j5.pdf'); fig.savefig(OUT / 'fig_j5.png', dpi=300)
print('ok')
