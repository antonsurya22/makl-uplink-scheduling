#!/usr/bin/env python3
"""J4 bridge launcher = the frozen J1 v2 launcher (j1_bridge.py, imported unchanged) with the J4 link factory
(j4_links.py: J3 factory + MAKL_cap + v_hat in the decision log). B7 and MAKL are built by the unchanged J1 factory,
DSW_1.0 by the unchanged J3 factory."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import j4_links as L4
L4.install()
import j1_bridge as JB                 # sets JB.M.JointC52Bridge = race_safe(J1 launcher)
if __name__ == '__main__':
    print('J4_BRIDGE class', JB.M.JointC52Bridge.__name__, flush=True)
    JB.M.main()
