#!/usr/bin/env python3
"""J5b edge entry point (Ubuntu VM). Usage: j5b_edge.py RESULTS_DIR. Runs until the sender reports all_done."""
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import j5b_common  # noqa: F401
import rclpy
from rclpy.executors import SingleThreadedExecutor
from j5b_edge_core import J5BEdgeCore
from j5_ros import EdgeNode

if __name__ == '__main__':
    rclpy.init()
    core = J5BEdgeCore(sys.argv[1]); node = EdgeNode(core)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    print('J5B_EDGE_READY', sys.argv[1], flush=True)
    try:
        while rclpy.ok() and not core.done:
            ex.spin_once(timeout_sec=0.5)
        t = time.monotonic()
        while time.monotonic() - t < 5:
            ex.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
    print('J5B_EDGE_EXIT done=%s' % core.done, flush=True)
