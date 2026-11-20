#!/usr/bin/env python3
"""J5b sender entry point (Raspberry Pi). Usage: j5b_robot.py RESULTS_DIR"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5b_common  # noqa: F401  (sets the pace and the matrix before the J5 modules are used)
from j5b_robot_core import J5BRunner
from j5_robot_core import Halt
from j5_sysops import SysOps
from j5_ros import RosTransport

if __name__ == '__main__':
    tp = RosTransport()
    try:
        J5BRunner(tp, SysOps(), sys.argv[1]).run()
    except Halt as h:
        print(str(h), flush=True); sys.exit(3)
    finally:
        tp.close()
