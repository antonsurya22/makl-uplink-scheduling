#!/usr/bin/env python3
"""J4 final hop. Usage: j4_guard.py V. The frozen safety_chain.py SafetyChain node with MAX_LIN = max(0.18, V) (the
command cap must admit the speed level). Safety layer ON: every other constant frozen (0.70 m shield, 0.39 m latch,
0.5 s scan age, raw /scan). Same mechanism as E8's x2_guard.py (layer ON)."""
import sys
from pathlib import Path
E3B = Path.home() / 'ros2_ws/SCI_E3B_EDGE_NAV_RUNNER_20261031'
V_SET = (0.10, 0.15, 0.20, 0.22)


def configure(G, v):
    G.MAX_LIN = max(0.18, float(v))
    return G


if __name__ == '__main__':
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    if v not in V_SET:
        raise SystemExit('J4_GUARD_V_REQUIRED')
    sys.path.insert(0, str(E3B)); import safety_chain as G
    configure(G, v)
    print('J4_GUARD V=%s SAFE_STOP=%s EMERGENCY=%s SCAN_MAX_AGE=%s MAX_LIN=%s' % (v, G.SAFE_STOP, G.EMERGENCY, G.SCAN_MAX_AGE,
                                                                               G.MAX_LIN), flush=True)
    sys.argv = [str(E3B / 'safety_chain.py')]
    G.main()
