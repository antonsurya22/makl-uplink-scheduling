#!/usr/bin/env bash
# J5 edge (Ubuntu VM, ens33 192.168.200.235). Start this FIRST, then JALANKAN_J5_ROBOT.sh on the Pi.
# Steps: environment -> freeze check -> vendored code = frozen packages here -> traces rebuilt from the J1 results ->
# offline tests -> chrony server for the Pi -> firewall and iperf3 server -> edge receiver (until the Pi reports
# all_done; the analysis then runs here) -> ZIP to ~/Downloads. It can be stopped (Ctrl+C) and started again.
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PKG"
source "$PKG/j5_env.sh" edge
mkdir -p "$J5_RES"
LOG="$J5_RES/EDGE_CONSOLE_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
say() { echo "[J5 edge $(date +%H:%M:%S)] $*"; }
fail() { echo "J5_EDGE_STOP: $*"; exit 2; }

say "1/8 host checks"
ip -4 addr show "$J5_IF" 2>/dev/null | grep -q "inet $J5_SELF/" || fail "interface $J5_IF does not have $J5_SELF (bridged adapter? DHCP reservation?)"
for c in python3 chronyc iperf3 tc sha256sum; do command -v $c >/dev/null || fail "missing command $c"; done
python3 -c "import rclpy, std_msgs" || fail "rclpy not found (ROS 2 Humble)"
ping -c 3 -W 2 "$J5_PEER" >/dev/null || say "WARNING: the Pi ($J5_PEER) does not answer ping yet (fine if it is still off)"

say "2/8 package freeze"
python3 -I "$PKG/freeze_j5.py" --check || fail "package files differ from J5_FREEZE_HASHES.json"

say "3/8 + 4/8 offline tests (vendored code = frozen packages here; traces rebuilt from J1 results; loopback run)"
WS="$HOME/ros2_ws"
[ -d "$WS/SCI_P0J_SCHEDULERS_20261111" ] || fail "frozen P0-J package not found in $WS"
[ -d "$WS/results/SCI_J1_SCHEDULERS_V2_20261113/J1_MAIN/trials" ] || fail "J1 results not found in $WS/results"
TLOG="$J5_RES/J5_OFFLINE_TESTS_EDGE.log"
python3 -I "$PKG/tests/test_j5_offline.py" "$WS" > "$TLOG" 2>&1 || { tail -30 "$TLOG"; fail "offline tests failed (log $TLOG)"; }
if grep -q '^SKIP' "$TLOG"; then grep '^SKIP' "$TLOG"; fail "on the VM no offline test may be skipped"; fi
tail -1 "$TLOG"

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

say "8/8 edge receiver: now start JALANKAN_J5_ROBOT.sh on the Pi. This window must stay open (about 1.5-2 h)."
set +e
python3 "$PKG/j5_edge.py" "$J5_RES/J5_MAIN"
RC=$?
set -e
if [ -f "$J5_RES/J5_MAIN/final/J5_VERDICTS.json" ]; then
  Z="$HOME/Downloads/J5_RESULTS_$(date +%Y%m%d_%H%M%S).zip"; mkdir -p "$HOME/Downloads"
  (cd "$J5_RES/.." && python3 -I -m zipfile -c "$Z" "$(basename "$J5_RES")")
  say "J5 finished. Results: $Z  (send this ZIP and the console log)"
  cat "$J5_RES/J5_MAIN/final/J5_VERDICTS.json" | head -40
else
  say "edge stopped before the analysis (rc=$RC). Run this script again to continue; it keeps what it has."
fi
