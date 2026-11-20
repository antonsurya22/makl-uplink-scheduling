#!/usr/bin/env python3
"""J3 bridge launcher = the frozen J1 v2 launcher (j1_bridge.py from SCI_J1_SCHEDULERS_V2_20261113, imported unchanged:
E3-B joint C5.2 bridge + E5 race fix + J1 scheduler swap, capture stamps and onboard inputs) with ONE addition: the J1 link
factory also builds the distance-switch arms DSW_0.8 / DSW_1.0 / DSW_1.2 (j3_links.py), and the bridge passes them the
same onboard input d as MAKL. B7 and MAKL are built by the unchanged J1 factory."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import j3_links as L3
L3.install()
import j1_bridge as JB                 # sets JB.M.JointC52Bridge = race_safe(J1 launcher)
if __name__ == '__main__':
    print('J3_BRIDGE class', JB.M.JointC52Bridge.__name__, flush=True)
    JB.M.main()
