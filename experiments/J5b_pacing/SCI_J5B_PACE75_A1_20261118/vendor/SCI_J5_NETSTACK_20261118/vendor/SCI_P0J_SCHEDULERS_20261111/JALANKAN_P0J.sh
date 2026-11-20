#!/usr/bin/env bash
# P0-J: new schedulers + unit tests + trace export + J0 replay. OFFLINE ONLY: no ROS, no Gazebo, no robot.
# Usage (from ~/ros2_ws):  bash SCI_P0J_SCHEDULERS_20261111/JALANKAN_P0J.sh
# Expected time: about 5-15 minutes. Output: ~/Downloads/SCI_P0J_RESULTS_<UTC>.zip and the console log.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTC="$(date -u +%Y%m%dT%H%M%SZ)"
LOG="$HOME/Downloads/SCI_P0J_CONSOLE_${UTC}.log"
mkdir -p "$HOME/Downloads"
PY=/usr/bin/python3
{
  echo "P0J start ${UTC} host=$(hostname) python=$($PY --version 2>&1)"
  if pgrep -f "gzserver|run_all.py|_run_trial.py" >/dev/null 2>&1; then
    echo "P0J NOTE: a simulation or batch seems to be running. P0-J is offline and does not touch it,"
    echo "P0J NOTE: but it will compete for CPU. Wait for the other batch to finish if it is a counted run."
    pgrep -af "gzserver|run_all.py|_run_trial.py" | head -5 || true
  fi
  "$PY" -I "$HERE/run_p0j.py"
  echo "P0J end $(date -u +%Y%m%dT%H%M%SZ)"
} 2>&1 | tee "$LOG"
echo "Console log: $LOG"
