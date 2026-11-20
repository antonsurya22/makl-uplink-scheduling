#!/usr/bin/env python3
"""
preflight.py -- verify the ROS 2 graph before spending two hours on Stage 0.

stage0_runner.py assumes a set of topics, an action, a service, and controller
parameter names. If any of them is wrong the run fails silently or produces
garbage, and you find out 120 episodes later. Run this first; it takes seconds.

    # terminal 1: your simulation bringup
    # terminal 2:
    python3 preflight.py

Every failed check prints the command that tells you the correct name, so you
can patch stage0_runner.py before running anything.

Requires: a sourced ROS 2 Humble environment and a running stack.
No rclpy import: uses the ros2 CLI so it works from any Python.
"""

import re
import shutil
import subprocess
import sys

OK, WARN, FAIL = "PASS", "WARN", "FAIL"
results = []


def run(cmd, timeout=15):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.stdout
    except Exception:
        return ""


def check(name, status, detail="", fix=""):
    results.append((name, status, detail, fix))


def has(items, target):
    return any(i.strip() == target for i in items)


# ---------------------------------------------------------------------------
def main():
    if not shutil.which("ros2"):
        print("ros2 not on PATH. Source your ROS 2 Humble setup first.")
        return 2

    topics = run(["ros2", "topic", "list"]).splitlines()
    actions = run(["ros2", "action", "list"]).splitlines()
    services = run(["ros2", "service", "list"]).splitlines()
    nodes = run(["ros2", "node", "list"]).splitlines()

    if not topics:
        print("No topics visible. Is the simulation actually running, and is\n"
              "ROS_DOMAIN_ID the same in both terminals?")
        return 2

    # --- topics the runner subscribes to --------------------------------
    for t in ["/scan", "/odom", "/clock"]:
        check(f"topic {t}", OK if has(topics, t) else FAIL,
              fix="ros2 topic list")

    # --- command chain ---------------------------------------------------
    for t, req in [("/cmd_vel_remote", False), ("/cmd_vel_safe", False),
                   ("/cmd_vel", False)]:
        present = has(topics, t)
        check(f"topic {t}", OK if present else WARN,
              "" if present else "not present (fine at Stage 0 if unused)",
              "ros2 topic list | grep cmd_vel")

    # --- Nav2 action -----------------------------------------------------
    nav_ok = has(actions, "/navigate_to_pose") or has(actions, "navigate_to_pose")
    check("action navigate_to_pose", OK if nav_ok else FAIL,
          fix="ros2 action list | grep navigate")

    # --- Gazebo reset service -------------------------------------------
    gz = [s for s in services if "set_entity_state" in s]
    check("service set_entity_state", OK if gz else FAIL,
          gz[0].strip() if gz else "missing: gazebo_ros_state plugin not loaded",
          "add <plugin name='gazebo_ros_state' "
          "filename='libgazebo_ros_state.so'/> to the world, or use "
          "/gazebo/set_model_state")

    # --- AMCL initial pose ----------------------------------------------
    check("topic /initialpose", OK if has(topics, "/initialpose") else WARN,
          fix="ros2 topic list | grep initialpose")

    # --- controller velocity parameter ----------------------------------
    ctrl = [n for n in nodes if "controller_server" in n]
    if not ctrl:
        check("controller_server", FAIL, "node not found",
              "ros2 node list | grep controller")
    else:
        params = run(["ros2", "param", "list", ctrl[0].strip()]).splitlines()
        vel = [p.strip() for p in params
               if re.search(r"(vx_max|max_vel_x|max_speed)", p)]
        if vel:
            check("controller velocity param", OK, ", ".join(vel[:3]),
                  "set CONSERVATIVE_PARAMS / NORMAL_PARAMS in "
                  "stage0_runner.py to one of these")
        else:
            check("controller velocity param", FAIL, "none matched",
                  f"ros2 param list {ctrl[0].strip()} | grep -i vel")

    # --- network emulator ------------------------------------------------
    emu = [n for n in nodes if "network_emulator" in n]
    if emu:
        params = run(["ros2", "param", "list", emu[0].strip()]).splitlines()
        has_profile = any("profile" in p for p in params)
        check("network_emulator profile param", OK if has_profile else WARN,
              "" if has_profile else "no 'profile' parameter found",
              "adjust PROFILE_CMD in stage0_runner.py to your actual interface")
    else:
        check("network_emulator node", FAIL, "not running",
              "Stage 0 needs the Severe condition; start the emulator, or "
              "Stage 0 measures nothing")

    # --- sim time --------------------------------------------------------
    check("simulated time", OK if has(topics, "/clock") else FAIL,
          fix="all nodes must run with use_sim_time:=true")

    # --- report ----------------------------------------------------------
    width = max(len(n) for n, _, _, _ in results) + 2
    print()
    for name, status, detail, fix in results:
        line = f"  [{status}] {name:<{width}}{detail}"
        print(line)
        if status == FAIL and fix:
            print(f"         -> {fix}")
    n_fail = sum(1 for _, s, _, _ in results if s == FAIL)
    n_warn = sum(1 for _, s, _, _ in results if s == WARN)
    print(f"\n  {len(results) - n_fail - n_warn} pass, {n_warn} warn, "
          f"{n_fail} fail")
    if n_fail:
        print("\n  Fix the failures before running stage0_runner.py.\n")
        return 1
    print("\n  Ready. Run:  ros2 run <your_pkg> stage0_runner.py "
          "--episodes 40 --out stage0.csv\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
