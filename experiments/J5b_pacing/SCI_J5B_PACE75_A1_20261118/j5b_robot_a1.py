#!/usr/bin/env python3
"""J5b sender entry point with amendment A1 (BSSID reader, j5b_sysops_a1.py). Usage: j5b_robot_a1.py RESULTS_DIR"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5b_common  # noqa: F401  (sets the pace and the matrix before the J5 modules are used)
from j5b_robot_core import J5BRunner
from j5_robot_core import Halt
from j5b_sysops_a1 import SysOpsA1
from j5_ros import RosTransport

if __name__ == '__main__':
    tp = RosTransport()
    try:
        J5BRunner(tp, SysOpsA1(), sys.argv[1]).run()
    except Halt as h:
        print(str(h), flush=True); sys.exit(3)
    finally:
        tp.close()
