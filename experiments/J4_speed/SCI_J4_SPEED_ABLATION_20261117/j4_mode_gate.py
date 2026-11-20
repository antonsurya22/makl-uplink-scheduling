#!/usr/bin/env python3
"""J4 mode gate. Usage: j4_mode_gate.py V. The frozen e2_mode_gate.py with the NORMAL cap set to V (SLOW 0.08, STOP 0,
angular cap and fail-closed ages unchanged). Same mechanism as E8's x2_mode_gate.py; only the accepted set of V differs."""
import sys
from pathlib import Path
E3B = Path.home() / 'ros2_ws/SCI_E3B_EDGE_NAV_RUNNER_20261031'
V_SET = (0.10, 0.15, 0.20, 0.22)


def configure(G, v):
    G.LIN_CAP = (float(v), G.LIN_CAP[1], G.LIN_CAP[2])
    return G


if __name__ == '__main__':
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    if v not in V_SET:
        raise SystemExit('J4_MODE_GATE_V_REQUIRED')
    sys.path.insert(0, str(E3B)); import e2_mode_gate as G
    configure(G, v); print('J4_MODE_GATE LIN_CAP=%s' % (G.LIN_CAP,), flush=True)
    sys.argv = [str(E3B / 'e2_mode_gate.py')]; G.main()
