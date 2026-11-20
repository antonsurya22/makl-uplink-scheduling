#!/usr/bin/env bash
# Offline checks (no ROS): freeze files, P0-J unit tests, J5/J5b offline tests.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
export PYTHONDONTWRITEBYTECODE=1   # the J5 tests start the analysis in a subprocess

echo "== freeze files"
python3 "$ROOT/tools/verify_freezes.py"

echo "== P0-J unit tests"
(cd "$TMP" && python3 -I -B "$ROOT/experiments/P0J_schedulers/SCI_P0J_SCHEDULERS_20261111/tests/test_p0j.py" | tail -1)

echo "== J5 / J5b offline tests"
python3 -I -B "$ROOT/experiments/J5_network_stack/SCI_J5_NETSTACK_20261118/tests/test_j5_offline.py" "$TMP/ws" | tail -1
python3 -I -B "$ROOT/experiments/J5b_pacing/SCI_J5B_PACE75_A1_20261118/tests/test_j5b_offline.py" "$TMP/ws" | tail -1
python3 -I -B "$ROOT/experiments/J5b_pacing/SCI_J5B_PACE75_A1_20261118/tests/test_j5b_a1.py" | tail -1
