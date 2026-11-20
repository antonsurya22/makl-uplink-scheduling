"""J5b edge = frozen J5 EdgeCore; on all_done it runs the J5b analysis (not the J5 one)."""
from __future__ import annotations
import json, subprocess, sys
from pathlib import Path
import j5b_common as B
from j5_edge_core import EdgeCore


class J5BEdgeCore(EdgeCore):
    def __init__(self, base, run_analysis=True, j5_cells=None):
        super().__init__(base, run_analysis=False)
        self.run_j5b = run_analysis
        self.j5_cells = Path(j5_cells) if j5_cells else Path.home() / B.J5_RESULTS_REL / 'final' / 'J5_CELLS.csv'

    def on_ctl(self, kind, body):
        rep = super().on_ctl(kind, body)
        if kind == 'all_done' and self.run_j5b:
            p = subprocess.run([sys.executable, str(B.HERE / 'j5b_analysis.py'), str(self.base), str(self.base / 'final'),
                                str(self.j5_cells)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            (self.base / 'ANALYSIS_CONSOLE.log').write_text(p.stdout)
            rep['analysis_rc'] = p.returncode; rep['analysis_tail'] = p.stdout[-1500:]
        return rep
