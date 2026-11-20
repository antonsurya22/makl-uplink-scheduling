"""Fig. 7 (J3): violation vs useful frames per cell for B7, MAKL and the distance switch at 0.8/1.0/1.2 m.
v4.3: linear axes in every panel; in (b) the 0.8 m switch (9.3 s) is drawn at the right edge with an arrow;
labels placed with thin leader lines so they do not cover the whiskers.
Usage: make_fig_j3.py J3_CELLS.csv OUTDIR"""
import csv, sys
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 7.5, 'axes.linewidth': 0.6, 'axes.spines.top': False,
                     'axes.spines.right': False, 'pdf.fonttype': 42})
C = list(csv.DictReader(open(sys.argv[1]))); OUT = Path(sys.argv[2])
V = 'W_viol_anav_unom_b0.2_s'
cell = {(r['cell'], r['config']): r for r in C}
PAN = (('LB-Q1', '(a) Q1, 0.18 Mbit/s (design point)', 2.8), ('LC-Q1', '(b) Q1, 0.13 Mbit/s', 3.4),
       ('LC-Q2', '(c) Q2, 0.13 Mbit/s', 2.3))
ORANGE, NAVY, GREY = '#c55a11', '#0b3d75', '#6b6b6b'
NAME = {'DSW_0.8': 'switch 0.8 m', 'DSW_1.0': 'switch 1.0 m', 'DSW_1.2': 'switch 1.2 m', 'B7': 'B7', 'MAKL': 'MAKL'}
# label positions in data coordinates (x, y) per panel
LAB = {
    'LB-Q1': {'MAKL': (0.25, 3.55), 'DSW_1.0': (1.85, 3.45), 'DSW_1.2': (1.85, 2.55), 'DSW_0.8': (1.25, 4.55),
              'B7': (1.85, 1.95)},
    'LC-Q1': {'MAKL': (0.25, 2.35), 'DSW_1.0': (2.62, 1.60), 'DSW_1.2': (2.2, 1.0), 'DSW_0.8': (1.55, 3.85),
              'B7': (2.0, 0.30)},
    'LC-Q2': {'MAKL': (0.25, 1.05), 'DSW_1.0': (1.75, 1.10), 'DSW_1.2': (1.75, 0.58), 'DSW_0.8': (0.45, 1.42),
              'B7': (1.0, 0.22)},
}
fig, axs = plt.subplots(1, 3, figsize=(7.1, 2.7))
for ax, (c, title, xmax) in zip(axs, PAN):
    pts = {}
    for a in ('B7', 'MAKL', 'DSW_0.8', 'DSW_1.0', 'DSW_1.2'):
        r = cell[(c, a)]
        pts[a] = (float(r[V]), float(r['F_fps']), r)
    edge = xmax - 0.12
    clip = {a: pts[a][0] > edge for a in pts}
    xs = [min(pts[a][0], edge) for a in ('DSW_0.8', 'DSW_1.0', 'DSW_1.2')]
    ys = [pts[a][1] for a in ('DSW_0.8', 'DSW_1.0', 'DSW_1.2')]
    ax.plot(xs[1:], ys[1:], '-', color=GREY, lw=0.8, zorder=1)
    ax.plot(xs[:2], ys[:2], '--' if clip['DSW_0.8'] else '-', color=GREY, lw=0.8, zorder=1)
    for a, (x, y, r) in pts.items():
        col = ORANGE if a == 'MAKL' else (NAVY if a == 'B7' else GREY)
        mk = 'o' if a in ('MAKL', 'B7') else 's'
        if clip[a]:
            ax.plot(edge, y, marker='>', ms=5, color=GREY, zorder=3, clip_on=False)
            ax.errorbar(edge, y, yerr=[[y - float(r['F_fps_ci_lo'])], [float(r['F_fps_ci_hi']) - y]], fmt='none',
                        ecolor=col, elinewidth=0.6, zorder=2)
            lx, ly = LAB[c][a]
            ax.annotate(f"{NAME[a]}: {x:.1f} s →", (edge, y), xytext=(lx, ly), textcoords='data', fontsize=6.3,
                        ha='left', color='#222222', arrowprops=dict(arrowstyle='-', lw=0.4, color='#999999'))
            continue
        ax.errorbar(x, y, xerr=[[x - float(r[V + '_ci_lo'])], [float(r[V + '_ci_hi']) - x]],
                    yerr=[[y - float(r['F_fps_ci_lo'])], [float(r['F_fps_ci_hi']) - y]], fmt=mk,
                    ms=4.8 if a == 'MAKL' else 3.8, color=col, mec='white', mew=0.6, elinewidth=0.6, capsize=0, zorder=3)
        lx, ly = LAB[c][a]
        ax.annotate(NAME[a], (x, y), xytext=(lx, ly), textcoords='data', fontsize=6.3, ha='left',
                    fontweight='bold' if a == 'MAKL' else 'normal', color='#222222',
                    arrowprops=dict(arrowstyle='-', lw=0.4, color='#999999', shrinkA=1, shrinkB=3))
    ax.set_title(title, loc='left', fontsize=7.3)
    ax.set_xlabel('Violation time (s)', fontsize=7)
    ax.set_xlim(0, xmax); ax.set_ylim(0, {'LB-Q1': 4.85, 'LC-Q1': 4.1, 'LC-Q2': 1.55}[c])
axs[0].set_ylabel('Useful camera frames (fps)', fontsize=7)
fig.tight_layout()
fig.savefig(OUT / 'fig_j3.pdf'); fig.savefig(OUT / 'fig_j3.png', dpi=300)
