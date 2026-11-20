#!/usr/bin/env python3
"""J5 edge entry point (Ubuntu VM). Usage: j5_edge.py RESULTS_DIR. Runs until the sender reports all_done (the analysis
then runs here) or until Ctrl+C. It can be restarted at any time; the sender waits for it."""
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from j5_edge_core import EdgeCore
from j5_ros import EdgeNode

if __name__ == '__main__':
    rclpy.init()
    core = EdgeCore(sys.argv[1]); node = EdgeNode(core)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    print('J5_EDGE_READY', sys.argv[1], flush=True)
    try:
        while rclpy.ok() and not core.done:
            ex.spin_once(timeout_sec=0.5)
        t = time.monotonic()
        while time.monotonic() - t < 5:          # let the last acknowledgement go out
            ex.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
    print('J5_EDGE_EXIT done=%s' % core.done, flush=True)
