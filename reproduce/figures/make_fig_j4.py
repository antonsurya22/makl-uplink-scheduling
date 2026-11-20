"""Fig. J4 (v4.6 layout, from the cell table only).
(a) switching clearance d_sw against MAKL's measured speed input (mean +- SD of five runs), with the model lines
    (a_peak = 0.68 s, fixed before the runs); the model of MAKL-FixedV coincides with the 1.0 m switch.
(b) useful camera frames in the approach (open) and after it (filled).
(c) violation time in the approach (mean +- SD), with the paired S1 difference at the top level.
Usage: make_fig_j4.py J4_CELLS.csv J4_TESTS.csv OUTDIR"""
import csv, json, sys
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 7.5, 'axes.linewidth': 0.6, 'axes.spines.top': False,
                     'axes.spines.right': False, 'pdf.fonttype': 42})
C = list(csv.DictReader(open(sys.argv[1])))
T = {r['test']: json.loads(r['result_json']) for r in csv.DictReader(open(sys.argv[2]))}
OUT = Path(sys.argv[3])
ORANGE, NAVY, TEAL, GREY = '#c55a11', '#0b3d75', '#1b8a7a', '#6b6b6b'
COL = {'B7': NAVY, 'MAKL': ORANGE, 'MAKL_cap': TEAL, 'DSW_1.0': GREY}
LAB = {'B7': 'priority + keep-last (B7)', 'MAKL': 'MAKL', 'MAKL_cap': 'MAKL-FixedV', 'DSW_1.0': 'switch 1.0 m'}
MK = {'B7': 's', 'MAKL': 'o', 'MAKL_cap': 'D', 'DSW_1.0': '^'}
LV = (0.10, 0.15, 0.22); TAG = {0.10: 'V010', 0.15: 'V015', 0.22: 'V022'}
VH = {0.10: 0.0947, 0.15: 0.1421, 0.22: 0.2154}   # MAKL median speed input in W (J4 report, Section 2)
cell = {}
for r in C:
    cell.setdefault((r['cell'], r['config']), r)   # first row = congested run of the cell (C0 rows come later)
f = lambda x: float(x) if x not in (None, '') else None
ARMS = ('B7', 'MAKL', 'MAKL_cap', 'DSW_1.0')

fig, axs = plt.subplots(1, 3, figsize=(7.1, 2.65), gridspec_kw={'width_ratios': [1.0, 1.05, 1.0]})

# (a) switching clearance
ax = axs[0]
vv = [0.08 + 0.0015 * k for k in range(101)]
ax.plot(vv, [0.88 * (v + 0.5) + v * v / 0.4 + 2.5 * v for v in vv], '-', color=ORANGE, lw=0.8, alpha=0.7)
ax.axhline(0.88 * 0.65 + 0.15 ** 2 / 0.4 + 2.5 * 0.15, color=GREY, lw=0.8, ls='--')
for a, dx in (('MAKL', -0.002), ('MAKL_cap', 0.002)):
    for v in LV:
        r = cell[(f'{TAG[v]}|LB-Q1', a)]
        ax.errorbar(VH[v] + dx, f(r['d_sw_m']), yerr=f(r['d_sw_m_sd']), fmt=MK[a], ms=4.4, color=COL[a],
                    mec='white', mew=0.6, elinewidth=0.8, capsize=1.8)
ax.set_xlabel('MAKL speed input $\\hat v$ (m/s), measured', fontsize=7)
ax.set_ylabel('Switching clearance $d_{sw}$ (m)', fontsize=7)
ax.set_xlim(0.075, 0.235); ax.set_ylim(0.6, 1.45)
h = [Line2D([], [], marker='o', ls='none', color=ORANGE, ms=4, label='MAKL'),
     Line2D([], [], marker='D', ls='none', color=TEAL, ms=3.6, label='MAKL-FixedV'),
     Line2D([], [], color=ORANGE, lw=0.8, alpha=0.7, label='model, MAKL'),
     Line2D([], [], color=GREY, lw=0.8, ls='--', label='model, FixedV = switch 1.0 m')]
ax.legend(handles=h, fontsize=5.8, frameon=False, loc='upper left', handlelength=1.6)
ax.set_title('(a) where the scan goes first', loc='left', fontsize=7)

# (b) frames by phase
ax = axs[1]; w = 0.19
for i, a in enumerate(ARMS):
    for j, v in enumerate(LV):
        r = cell[(f'{TAG[v]}|LB-Q1', a)]; x = j + (i - 1.5) * w
        fa, fb = f(r['F_A']), f(r['F_B'])
        ax.plot([x, x], [fa, fb], '-', color=COL[a], lw=0.8)
        ax.plot(x, fa, MK[a], ms=4.0, mfc='white', mec=COL[a], mew=0.9)
        ax.plot(x, fb, MK[a], ms=4.0, color=COL[a], mec='white', mew=0.5)
ax.set_xticks(range(3)); ax.set_xticklabels(['0.10', '0.15', '0.22']); ax.set_xlabel('Speed level (m/s)', fontsize=7)
ax.set_ylabel('Useful camera frames (fps)', fontsize=7); ax.set_ylim(0, 7.0); ax.set_yticks(range(0, 6))
h = [Line2D([], [], marker=MK[a], ls='none', color=COL[a], ms=3.8, label=LAB[a]) for a in ARMS]
h += [Line2D([], [], marker='o', ls='none', mfc='white', mec='k', ms=3.8, label='open: approach'),
      Line2D([], [], marker='o', ls='none', color='k', ms=3.8, label='filled: after it')]
ax.legend(handles=h, fontsize=5.5, frameon=False, ncol=2, loc='upper center', bbox_to_anchor=(0.5, 1.02), columnspacing=0.8, handletextpad=0.3)
ax.set_title('(b) frames before and after the encounter', loc='left', fontsize=7)

# (c) violation in the approach
ax = axs[2]
for i, a in enumerate(ARMS):
    for j, v in enumerate(LV):
        r = cell[(f'{TAG[v]}|LB-Q1', a)]; x = j + (i - 1.5) * w
        ax.bar(x, f(r['viol_A']), w * 0.92, color=COL[a])
        ax.errorbar(x, f(r['viol_A']), yerr=f(r['viol_A_sd']), fmt='none', ecolor='k', elinewidth=0.6, capsize=1.5)
s1 = T['S1_top_FixedV_gt_MAKL_violA']
x0, x1 = 2 + (1 - 1.5) * w, 2 + (2 - 1.5) * w
yb = 2.08
ax.plot([x0, x0, x1, x1], [yb - 0.05, yb, yb, yb - 0.05], '-', color='k', lw=0.6)
ax.text((x0 + x1) / 2, yb + 0.04, f"+{s1['mean_diff']:.2f} s,\n5 of 5 pairs", ha='center', va='bottom', fontsize=5.6)
ax.set_xticks(range(3)); ax.set_xticklabels(['0.10', '0.15', '0.22']); ax.set_xlabel('Speed level (m/s)', fontsize=7)
ax.set_ylabel('Violation time in the approach (s)', fontsize=7); ax.set_ylim(0, 2.6)
ax.set_title('(c) violation in the approach', loc='left', fontsize=7)

fig.tight_layout(w_pad=1.2)
fig.savefig(OUT / 'fig_j4.pdf'); fig.savefig(OUT / 'fig_j4.png', dpi=300)
print('ok')
