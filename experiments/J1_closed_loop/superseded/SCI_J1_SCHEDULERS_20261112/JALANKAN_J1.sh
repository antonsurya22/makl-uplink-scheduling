#!/usr/bin/env bash
# ONE command: offline tests -> J1 batch (lock -> 2 gate runs -> 240 counted runs) -> analysis -> results ZIP.
# Simulation only. Resumable: if interrupted, run the same command again (finished runs are never repeated).
# Expected time: about 9.5-10 hours.
set +u
source "$HOME/ros2_ws/env_naw.bash"
cd "$HOME/ros2_ws" || exit 1
export PYTHONNOUSERSITE=1
P=SCI_J1_SCHEDULERS_20261112
mkdir -p "$HOME/Downloads"
LOG="$HOME/Downloads/SCI_J1_SCHEDULERS_CONSOLE_$(date -u +%Y%m%dT%H%M%SZ).log"
{
echo "=== PRE-CHECK ==="
if pgrep -af 'gzserver|controller_server|collision_monitor|safety_chain.py|joint_c52_bridge|p0_bridge|e5_bridge|e7_bridge|j1_bridge|x1_guard|tf_age_logger' ; then
  echo "BLOCKED: Gazebo/Nav2/bridge process already running. Close it first:"
  echo "  pkill -f gzserver; pkill -f controller_server; pkill -f planner_server; pkill -f collision_monitor; pkill -f safety_chain.py; pkill -f j1_bridge; pkill -f e7_bridge; pkill -9 -f tf_age_logger"
  exit 2; fi
echo "=== OFFLINE TESTS ==="
python3 SCI_E3B_EDGE_NAV_RUNNER_20261031/verify.py || { echo "E3B_PACKAGE_CHANGED"; exit 3; }
TLOG="$HOME/Downloads/J1_OFFLINE_TESTS_$(date -u +%Y%m%dT%H%M%SZ).log"
/usr/bin/python3 "$P/tests/test_j1_offline.py" "$HOME/ros2_ws" > "$TLOG" 2>&1; RC=$?
grep -v '^J1_BRIDGE\|^E7_BRIDGE' "$TLOG"
[ "$RC" = "0" ] || { echo "J1_OFFLINE_TESTS_FAILED"; exit 4; }
echo "=== J1 BATCH ==="
SCI_SIM_ONLY=YES SCI_E2_NO_PHYSICAL_ROBOT=YES /usr/bin/python3 -u "$P/j1_run_all.py"
echo "J1_BATCH_EXIT_CODE=$?"
} 2>&1 | tee "$LOG"
echo "Console log: $LOG"
