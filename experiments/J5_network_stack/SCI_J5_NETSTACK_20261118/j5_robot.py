#!/usr/bin/env python3
"""J5 sender entry point (Raspberry Pi). Usage: j5_robot.py RESULTS_DIR"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5_common as C
from j5_robot_core import RobotRunner, Halt
from j5_sysops import SysOps
from j5_ros import RosTransport

if __name__ == '__main__':
    tp = RosTransport()
    try:
        RobotRunner(tp, SysOps(), sys.argv[1]).run()
    except Halt as h:
        print(str(h), flush=True); sys.exit(3)
    finally:
        tp.close()
