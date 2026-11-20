#!/usr/bin/env bash
# J5b edge WITH AMENDMENT A1 (new results folder J5B_MAIN_A1). Start this FIRST, then JALANKAN_J5B_A1_ROBOT.sh on the Pi.
# J5b = the frozen J5 package (vendored, unchanged) with the NP pace 0.75 c instead of 0.9 c (J5B_PROTOCOL_FREEZE.md).
# Steps: environment -> freeze checks (J5b and vendored J5) -> J5 offline tests + J5b offline tests -> chrony server
# (as J5) -> firewall and iperf3 server -> edge receiver (until the Pi reports all_done; the J5b analysis then runs
# here) -> ZIP to ~/Downloads. It can be stopped (Ctrl+C) and started again.
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PKG"
source "$PKG/j5b_env.sh" edge
mkdir -p "$J5_RES"
LOG="$J5_RES/EDGE_CONSOLE_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
say() { echo "[J5b edge $(date +%H:%M:%S)] $*"; }
fail() { echo "J5B_EDGE_STOP: $*"; exit 2; }

say "1/8 host checks"
ip -4 addr show "$J5_IF" 2>/dev/null | grep -q "inet $J5_SELF/" || fail "interface $J5_IF does not have $J5_SELF (bridged adapter? DHCP reservation?)"
for c in python3 chronyc iperf3 tc sha256sum; do command -v $c >/dev/null || fail "missing command $c"; done
python3 -c "import rclpy, std_msgs" || fail "rclpy not found (ROS 2 Humble)"
ping -c 3 -W 2 "$J5_PEER" >/dev/null || say "WARNING: the Pi ($J5_PEER) does not answer ping yet (fine if it is still off)"

say "2/8 package freeze (J5b A1; the original J5b files; the vendored J5 package)"
python3 -I "$PKG/freeze_j5b_a1.py" --check || fail "package files differ from J5B_A1_FREEZE_HASHES.json"
python3 -I "$PKG/vendor/SCI_J5_NETSTACK_20261118/freeze_j5.py" --check || fail "vendored J5 differs from its freeze"

say "3/8 + 4/8 offline tests (vendored code = frozen packages here; traces rebuilt from J1 results; loopback run)"
WS="$HOME/ros2_ws"
[ -d "$WS/SCI_P0J_SCHEDULERS_20261111" ] || fail "frozen P0-J package not found in $WS"
[ -d "$WS/results/SCI_J1_SCHEDULERS_V2_20261113/J1_MAIN/trials" ] || fail "J1 results not found in $WS/results"
TLOG="$J5_RES/J5_OFFLINE_TESTS_EDGE.log"
python3 -I "$PKG/vendor/SCI_J5_NETSTACK_20261118/tests/test_j5_offline.py" "$WS" > "$TLOG" 2>&1 || { tail -30 "$TLOG"; fail "J5 offline tests failed (log $TLOG)"; }
if grep -q '^SKIP' "$TLOG"; then grep '^SKIP' "$TLOG"; fail "on the VM no offline test may be skipped"; fi
tail -1 "$TLOG"
[ -d "$WS/SCI_J5_NETSTACK_20261118" ] || fail "the J5 package is not installed in $WS (it was used for J5)"
TLOG2="$J5_RES/J5B_OFFLINE_TESTS_EDGE.log"
python3 -I "$PKG/tests/test_j5b_offline.py" "$WS" > "$TLOG2" 2>&1 || { tail -30 "$TLOG2"; fail "J5b offline tests failed (log $TLOG2)"; }
if grep -q '^SKIP' "$TLOG2"; then grep '^SKIP' "$TLOG2"; fail "on the VM no J5b offline test may be skipped"; fi
tail -1 "$TLOG2"
TLOG3="$J5_RES/J5B_A1_TESTS_EDGE.log"
python3 -I "$PKG/tests/test_j5b_a1.py" > "$TLOG3" 2>&1 || { tail -30 "$TLOG3"; fail "A1 tests failed (log $TLOG3)"; }
tail -1 "$TLOG3"
[ -f "$HOME/ros2_ws/results/SCI_J5_NETSTACK_20261118/J5_MAIN/final/J5_CELLS.csv" ] || say "note: J5 results not found here; the J5b analysis skips the reported comparison with J5"

say "5/8 chrony server for the Pi (sudo password may be asked once)"
grep -Eq '^\s*confdir\s+/etc/chrony/conf.d' /etc/chrony/chrony.conf || fail "/etc/chrony/chrony.conf has no 'confdir /etc/chrony/conf.d'"
WANT=$'# J5: serve time to the J5 sender on the lab LAN\nallow 192.168.200.0/24\nlocal stratum 10\n'
if [ "$(cat /etc/chrony/conf.d/j5.conf 2>/dev/null || true)" != "$(printf '%s' "$WANT")" ]; then
  printf '%s' "$WANT" | sudo tee /etc/chrony/conf.d/j5.conf >/dev/null
  sudo systemctl restart chrony
fi
chronyc -n tracking | sed -n '1,3p'

say "6/8 firewall and iperf3 server"
if command -v ufw >/dev/null && sudo ufw status | grep -q '^Status: active'; then
  sudo ufw allow from "$J5_PEER" >/dev/null && say "ufw: allowed all traffic from $J5_PEER"
fi
pgrep -x iperf3 >/dev/null || iperf3 -s -D
pgrep -x iperf3 >/dev/null || fail "iperf3 server did not start"

say "7/8 versions (recorded; no apt upgrade until J5 ends)"
{ uname -a; dpkg-query -W -f='${Package} ${Version}\n' ros-humble-rclpy ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds chrony iproute2 iperf3 2>/dev/null; } > "$J5_RES/EDGE_VERSIONS.txt"
cat "$J5_RES/EDGE_VERSIONS.txt"

say "8/8 edge receiver: now start JALANKAN_J5B_A1_ROBOT.sh on the Pi. This window must stay open (about 50 min)."
set +e
python3 "$PKG/j5b_edge.py" "$J5_RES/J5B_MAIN_A1"
RC=$?
set -e
if [ -f "$J5_RES/J5B_MAIN_A1/final/J5B_VERDICTS.json" ]; then
  Z="$HOME/Downloads/J5B_A1_RESULTS_$(date +%Y%m%d_%H%M%S).zip"; mkdir -p "$HOME/Downloads"
  (cd "$J5_RES/.." && python3 -I -m zipfile -c "$Z" "$(basename "$J5_RES")")
  say "J5b finished. Results: $Z  (send this ZIP)"
  head -40 "$J5_RES/J5B_MAIN_A1/final/J5B_VERDICTS.json"
else
  say "edge stopped before the analysis (rc=$RC). Run this script again to continue; it keeps what it has."
fi
