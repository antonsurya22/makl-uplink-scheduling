"""Fig. 6 (J2): paired differences MAKL - B7 per link condition. Usage: make_fig_j2.py J2_RUNS.csv OUTDIR"""
import csv, sys
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 7.5, 'axes.linewidth': 0.6, 'axes.spines.top': False,
                     'axes.spines.right': False, 'pdf.fonttype': 42})
runs = [r for r in csv.DictReader(open(sys.argv[1])) if r['status'] == 'SUCCEEDED']
OUT = Path(sys.argv[2])
V = 'W_viol_anav_unom_b0.2_s'
CONDS = [('none', 'ideal (J1 link)'), ('jit10', 'jitter ±10 ms'), ('jit50', 'jitter ±50 ms'), ('loss1', 'loss 1 %'),
         ('loss5', 'loss 5 %'), ('ge', 'burst loss (GE)'), ('capsq', 'capacity ±54 %')]
CELLS = (('LC-Q2', 'Q2, 0.13 Mbit/s', 0.7, 0.2), ('LB-Q1', 'Q1, 0.18 Mbit/s', 0.8, 0.4))
NAVY, ORANGE, GREY = '#0b3d75', '#c55a11', '#8a8a8a'
def pairs(pc, cond, metric):
    g = lambda a: {r['rep']: float(r[metric]) for r in runs if r['cell'] == f'{pc}|{cond}' and r['config'] == a}
    m, b = g('MAKL'), g('B7')
    return [m[k] - b[k] for k in sorted(m) if k in b]
fig, axes = plt.subplots(2, 2, figsize=(7.1, 4.3), sharey=True)
for i, (pc, title, Jv, Jf) in enumerate(CELLS):
    for j, (metric, lab, J, col) in enumerate(((V, 'Violation, MAKL − B7 (s)', Jv, NAVY), ('F_fps', 'Useful frames, MAKL − B7 (fps)', Jf, ORANGE))):
        ax = axes[i, j]
        for k, (c, name) in enumerate(CONDS):
            y = len(CONDS) - 1 - k
            d = pairs(pc, c, metric)
            ys, seen = [], {}
            for v in d:                                   # equal values side by side, not on top of each other
                k2 = round(v, 3); n = seen.get(k2, 0); seen[k2] = n + 1
                ys.append(y + (0.16 if n % 2 else -0.16) * (n > 0))
            ax.plot(d, ys, 'o', ms=3.2, color='#555555', alpha=0.8, mec='none')
            mean = sum(d) / len(d)
            ax.plot([mean, mean], [y - 0.28, y + 0.28], color='#1a1a1a', lw=2.0)
        ax.axvline(0, color='#444444', lw=0.6)
        if j == 0:
            ax.axvline(J, color=GREY, lw=0.8, ls='--'); ax.text(J, len(CONDS) - 0.45, f' tolerance {J} s', fontsize=6.5, color='#555555', va='bottom')
            ax.set_xlim(-1.0, 1.4)
        else:
            ax.axvline(J, color=GREY, lw=0.8, ls='--'); ax.text(J, len(CONDS) - 0.45, f' J = {J} fps', fontsize=6.5, color='#555555', va='bottom')
            ax.set_xlim(-0.1, 1.3)
        ax.set_yticks(range(len(CONDS))); ax.set_yticklabels([n for _, n in CONDS][::-1])
        ax.set_ylim(-0.6, len(CONDS) - 0.1)
        ax.set_title(f"({'abcd'[2 * i + j]}) {title}", loc='left', fontsize=7.5)
        if i == 1:
            ax.set_xlabel(lab)
fig.tight_layout()
fig.savefig(OUT / 'fig_j2.pdf'); fig.savefig(OUT / 'fig_j2.png', dpi=300)
