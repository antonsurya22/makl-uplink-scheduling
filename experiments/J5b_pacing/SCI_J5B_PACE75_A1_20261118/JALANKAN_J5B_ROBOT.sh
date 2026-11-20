#!/usr/bin/env bash
# J5b sender (Raspberry Pi 3B+, wlan0 192.168.200.159). Start JALANKAN_J5B_EDGE.sh on the VM FIRST.
# J5b = the frozen J5 package (vendored, unchanged) with the NP pace 0.75 c instead of 0.9 c (J5B_PROTOCOL_FREEZE.md).
# Open loop: the robot does not move; bring-up, LiDAR, camera and motors stay off.
# Steps: environment -> freeze check -> sudo rule for tc and iw only -> chrony client of the VM -> Wi-Fi power save off ->
# offline tests -> versions = J5 -> lock, J5 pilots, background pilot, 15 blocks (40 counted + 20 demo replays).
# It can be stopped (Ctrl+C) and started again;
# it continues where it stopped. A halt after the pilots needs review (send the console log).
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PKG"
source "$PKG/j5b_env.sh" robot
mkdir -p "$J5_RES"
LOG="$J5_RES/ROBOT_CONSOLE_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
say() { echo "[J5b robot $(date +%H:%M:%S)] $*"; }
fail() { echo "J5B_ROBOT_STOP: $*"; exit 2; }

say "1/7 host checks"
ip -4 addr show "$J5_IF" 2>/dev/null | grep -q "inet $J5_SELF/" || fail "$J5_IF does not have $J5_SELF (DHCP reservation?)"
for c in python3 chronyc iperf3 ping sha256sum /usr/sbin/tc /usr/sbin/iw; do command -v $c >/dev/null || fail "missing command $c"; done
python3 -c "import rclpy, std_msgs" || fail "rclpy not found (ROS 2 Humble)"
ping -c 3 -W 2 "$J5_PEER" >/dev/null || fail "the VM ($J5_PEER) does not answer ping"
if pgrep -f 'turtlebot3_bringup|hlds_laser|ld08|robot_state_publisher' >/dev/null; then fail "TurtleBot bring-up is running; stop it (J5 sends no robot data)"; fi

say "2/7 package freeze (J5b, and the vendored J5 package)"
python3 -I "$PKG/freeze_j5b.py" --check || fail "package files differ from J5B_FREEZE_HASHES.json"
python3 -I "$PKG/vendor/SCI_J5_NETSTACK_20261118/freeze_j5.py" --check || fail "vendored J5 differs from its freeze"

say "3/7 sudo rule: $(id -un) may run only /usr/sbin/tc and /usr/sbin/iw without a password (sudo password asked once)"
RULE="$(id -un) ALL=(root) NOPASSWD: /usr/sbin/tc, /usr/sbin/iw"
if [ "$(sudo cat /etc/sudoers.d/j5 2>/dev/null || true)" != "$RULE" ]; then
  TMP="$(mktemp)"; printf '%s\n' "$RULE" > "$TMP"
  sudo visudo -cf "$TMP" >/dev/null || { rm -f "$TMP"; fail "sudo rule did not validate"; }
  sudo install -m 0440 -o root -g root "$TMP" /etc/sudoers.d/j5; rm -f "$TMP"
fi
sudo -n /usr/sbin/tc qdisc show dev wlan0 >/dev/null || fail "sudo -n tc does not work"

say "4/7 clock: chrony follows the VM ($J5_PEER)"
grep -Eq '^\s*sourcedir\s+/etc/chrony/sources.d' /etc/chrony/chrony.conf || fail "/etc/chrony/chrony.conf has no 'sourcedir /etc/chrony/sources.d'"
WANT="server $J5_PEER iburst minpoll 0 maxpoll 2 prefer trust"
if [ "$(cat /etc/chrony/sources.d/j5.sources 2>/dev/null || true)" != "$WANT" ]; then
  echo "$WANT" | sudo tee /etc/chrony/sources.d/j5.sources >/dev/null
  sudo chronyc reload sources >/dev/null
fi
OK=0
for i in $(seq 1 60); do
  L="$(chronyc -n -c tracking || true)"; REF="$(echo "$L" | cut -d, -f2)"; OFF="$(echo "$L" | cut -d, -f5)"
  if [ "$REF" = "$J5_PEER" ] && python3 -c "import sys; sys.exit(0 if abs(float('$OFF')) <= 0.002 else 1)" 2>/dev/null; then OK=1; break; fi
  [ $((i % 6)) -eq 1 ] && say "   waiting for chrony: reference=$REF offset=${OFF}s"
  sleep 5
done
[ $OK = 1 ] || { chronyc -n sources; fail "clock not locked to the VM within 2 ms after 5 min"; }
say "   chrony: reference $REF, offset ${OFF} s"

say "5/7 Wi-Fi power save off"
sudo -n /usr/sbin/iw dev wlan0 set power_save off
/usr/sbin/iw dev wlan0 get power_save | grep -q 'off' || fail "power save is still on"
/usr/sbin/iw dev wlan0 link | sed -n '1,5p'
command -v vcgencmd >/dev/null && vcgencmd get_throttled || true

say "6/7 offline tests"
TLOG="$J5_RES/J5_OFFLINE_TESTS_ROBOT.log"
python3 -I "$PKG/vendor/SCI_J5_NETSTACK_20261118/tests/test_j5_offline.py" "$HOME/ros2_ws" > "$TLOG" 2>&1 || { tail -30 "$TLOG"; fail "J5 offline tests failed (log $TLOG)"; }
tail -1 "$TLOG"
TLOG2="$J5_RES/J5B_OFFLINE_TESTS_ROBOT.log"
python3 -I "$PKG/tests/test_j5b_offline.py" "$HOME/ros2_ws" > "$TLOG2" 2>&1 || { tail -30 "$TLOG2"; fail "J5b offline tests failed (log $TLOG2)"; }
tail -1 "$TLOG2"
[ -f "$HOME/ros2_ws/results/SCI_J5_NETSTACK_20261118/J5_MAIN/STATE.json" ] || fail "J5 state not found: the versions check against J5 needs it"
{ uname -a; dpkg-query -W -f='${Package} ${Version}\n' ros-humble-rclpy ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds chrony iproute2 iperf3 iw 2>/dev/null; } > "$J5_RES/ROBOT_VERSIONS.txt"

say "7/7 J5b run: lock, 4 pilots + 60 s background (uncounted), 15 blocks = 40 counted + 20 demo replays (about 50 min)."
say "   Tip: detach tmux now (Ctrl+b then d) so that no screen output travels over Wi-Fi; check later with: tmux attach -t j5b"
set +e
python3 "$PKG/j5b_robot.py" "$J5_RES/J5B_MAIN"
RC=$?
set -e
Z="$HOME/J5B_ROBOT_SIDE_$(date +%Y%m%d_%H%M%S).zip"
(cd "$J5_RES/.." && python3 -I -m zipfile -c "$Z" "$(basename "$J5_RES")") || true
DONE=$(python3 -I -c "import json,sys; s=json.load(open(sys.argv[1])); print(1 if s.get('completed') else 0)" "$J5_RES/J5B_MAIN/STATE.json" 2>/dev/null || echo 0)
if [ "$DONE" = 1 ]; then
  say "J5b sender finished (an abort on exit, rc 134, is known from J5 and harmless). Do NOT run again."
  say "The analysis runs on the VM; the robot-side copy is $Z"; RC=0
else case $RC in
  3) say "J5b HALTED (rc=3). Do not delete anything. Send $LOG and $Z for review." ;;
  *) say "J5b sender stopped (rc=$RC) before the end. Run this script again to continue where it stopped." ;;
esac; fi
exit $RC
