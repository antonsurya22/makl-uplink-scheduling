#!/usr/bin/env python3
"""J1 bridge launcher = the E7 (article E6) launcher pattern: the unchanged E3-B joint C5.2 bridge + the E5 race fix, with
ONE change: after the unchanged constructor has built its uplink, `self.link` is replaced, before the first /clock, by
j1_links.make_link(SCI_J1_CONFIG) with the same capacity, queue limit and delay. Per-message ledger -> SCI_J1_LEDGER.
For the new arms (B4, B5, B7, MAKL) the bridge passes each message's capture stamp (header stamp) to the link just
before its unchanged offer call. For MAKL only, the bridge also gives the scheduler the robot state, all onboard:
  d     = minimum valid range of the raw /scan message being offered - 0.22 m (range_max - 0.22 m if no valid return);
  v_odom= |linear.x| of /odom;   v_cmd = |linear.x| of /e2_nav_cmd_policy (Nav2 output BEFORE the mode gate and the
          safety layer).
MAKL decisions and CPU times -> SCI_J1_DECISIONS. A 'J1_STATE' line with input counts is printed every 100 scans.
Downlink, probes, camera encoding, profiles, episode engine and capacity schedule are untouched. Same CLI as E7."""
import math, os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
WS = Path.home() / 'ros2_ws'
E3B = WS / 'SCI_E3B_EDGE_NAV_RUNNER_20261031'
E5 = WS / 'SCI_E5_PED_SPEED_20261101'
sys.path.insert(0, str(E3B / 'bridge_optional'))
sys.path.insert(0, str(E5))
sys.path.insert(0, str(HERE))
import joint_c52_bridge_aoi as M
from bridge_fix.c52_race_fix import race_safe        # E5 package, hash-checked by j1_run_all.py
import j1_links as JL


def _stamp_us(msg):
    return int(msg.header.stamp.sec) * 1_000_000 + int(msg.header.stamp.nanosec) // 1000


def make_j1(base):
    class J1Bridge(base):
        def __init__(self, args):
            super().__init__(args)
            cfg = os.environ.get('SCI_J1_CONFIG')
            led = os.environ.get('SCI_J1_LEDGER')
            dec = os.environ.get('SCI_J1_DECISIONS')
            if cfg not in JL.J1_CONFIGS or not led or not dec:
                raise RuntimeError('J1_CONFIG_LEDGER_DECISIONS_REQUIRED')
            if self.clock_us is not None or self.link.records:
                raise RuntimeError('J1_LINK_SWAP_AFTER_TRAFFIC')
            old = self.link
            self.link = JL.make_link(cfg, capacity_bps=old.capacity_bps, queue_limit_bytes=old.queue_limit_bytes,
                                     delay_us=old.delay_us, seed=old.seed, jitter_us=old.jitter_us,
                                     packet_loss=old.packet_loss, ledger_path=led, decisions_path=dec)
            self.j1_new = isinstance(self.link, JL.J1Link)
            self.j1_makl = JL.is_makl(self.link)
            self.j1_n = {'odom': 0, 'cmd': 0, 'scan_d': 0}
            if self.j1_makl:
                from rclpy.qos import qos_profile_sensor_data
                from nav_msgs.msg import Odometry
                from geometry_msgs.msg import Twist
                self.create_subscription(Odometry, '/odom', self.j1_on_odom, qos_profile_sensor_data)
                self.create_subscription(Twist, '/e2_nav_cmd_policy', self.j1_on_cmd, 10)
            print('J1_BRIDGE config=%s link=%s jdisc=%s capacity_bps=%d queue_limit=%d delay_us=%d makl_inputs=%s' % (
                cfg, type(self.link).__name__, getattr(self.link, 'jdisc', 'frozen_E6'), old.capacity_bps,
                old.queue_limit_bytes, old.delay_us, 'odom,/e2_nav_cmd_policy,raw_scan' if self.j1_makl else 'none'),
                flush=True)

        # ---------------- MAKL inputs (onboard only) ----------------
        def j1_on_odom(self, msg):
            self.j1_n['odom'] += 1
            self.link.set_state(v_odom=abs(float(msg.twist.twist.linear.x)), t_us=self.clock_us)

        def j1_on_cmd(self, msg):
            self.j1_n['cmd'] += 1
            self.link.set_state(v_cmd=abs(float(msg.linear.x)), t_us=self.clock_us)

        # ---------------- capture stamps (new arms) and d (MAKL) ----------------
        def _offer_scan(self, msg):
            if self.j1_makl:
                vals = [float(x) for x in msg.ranges if math.isfinite(x) and msg.range_min <= x <= msg.range_max]
                d = (min(vals) if vals else float(msg.range_max)) - JL.ROBOT_R
                self.link.set_state(d=d, t_us=self.clock_us)
                self.j1_n['scan_d'] += 1
                if self.j1_n['scan_d'] % 100 == 0:
                    print('J1_STATE scans=%d odom=%d cmd=%d v_odom=%.3f v_cmd=%.3f d=%.3f decisions=%d' % (
                        self.j1_n['scan_d'], self.j1_n['odom'], self.j1_n['cmd'], self.link.state['v_odom'],
                        self.link.state['v_cmd'], self.link.state['d'], getattr(self.link.dlog, 'n', -1)), flush=True)
            if self.j1_new:
                self.link.next_capture_us = _stamp_us(msg)
            try:
                super()._offer_scan(msg)
            finally:
                if self.j1_new:
                    self.link.next_capture_us = None

        def _offer_camera(self, msg):
            if self.j1_new:
                self.link.next_capture_us = _stamp_us(msg)
            try:
                super()._offer_camera(msg)
            finally:
                if self.j1_new:
                    self.link.next_capture_us = None

        def _prune_completed(self):
            if isinstance(self.link, (JL.J.K.QdiscLink, JL.J.JQdiscLink)):
                self.link.prune(set(self.scan_cache) | set(self.camera_cache))
            else:
                super()._prune_completed()

    J1Bridge.__name__ = J1Bridge.__qualname__ = 'J1' + base.__name__
    return J1Bridge


M.JointC52Bridge = race_safe(make_j1(M.JointC52Bridge))
if __name__ == '__main__':
    print('J1_BRIDGE class', M.JointC52Bridge.__name__, flush=True)
    M.main()
