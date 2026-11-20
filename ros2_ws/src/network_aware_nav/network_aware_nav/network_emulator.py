#!/usr/bin/env python3

import random
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


PROFILES = {
    "normal": {
        "delay_min": 0.020,
        "delay_max": 0.050,
        "loss_min": 0.00,
        "loss_max": 0.01,
    },
    "mild": {
        "delay_min": 0.080,
        "delay_max": 0.150,
        "loss_min": 0.02,
        "loss_max": 0.05,
    },
    "moderate": {
        "delay_min": 0.180,
        "delay_max": 0.300,
        "loss_min": 0.08,
        "loss_max": 0.15,
    },
    "severe": {
        "delay_min": 0.450,
        "delay_max": 0.700,
        "loss_min": 0.20,
        "loss_max": 0.35,
    },
}


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class NetworkEmulator(Node):

    def __init__(self):
        super().__init__("network_emulator")

        self.declare_parameter("profile", "normal")
        self.declare_parameter("seed", 3)

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        self.active_profile = self._read_profile()

        # FIFO queue:
        # (requested_release_time_ns, LaserScan, sampled_delay)
        #
        # IMPORTANT:
        # We intentionally do NOT use a priority queue sorted by release
        # time. Sorting by release time permits a later scan with a shorter
        # random delay to overtake an earlier scan.
        #
        # With FIFO, later packets may experience head-of-line waiting, but
        # timestamps remain monotonic exactly as expected from an ordered
        # transport stream.
        self.pending = deque()

        self.last_sampled_delay = 0.0
        self.last_scan_stamp = None

        self.window_received = 0
        self.window_dropped = 0

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_degraded",
            qos_profile_sensor_data,
        )

        self.age_pub = self.create_publisher(
            Float32MultiArray,
            "/topic_age",
            10,
        )

        self.profile_pub = self.create_publisher(
            String,
            "/network_profile",
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # 100 Hz in ROS/simulation time.
        self.flush_timer = self.create_timer(
            0.01,
            self.flush_queue,
        )

        self.diag_timer = self.create_timer(
            0.5,
            self.publish_diagnostics,
        )

        self.get_logger().info(
            f"network emulator started: "
            f"profile={self.active_profile}, seed={seed}, "
            f"FIFO ordering enabled"
        )

    def _read_profile(self):
        name = str(self.get_parameter("profile").value).lower()

        if name not in PROFILES:
            self.get_logger().warning(
                f"unknown profile '{name}', using normal"
            )
            name = "normal"

        return name

    def _sync_profile(self):
        new_profile = self._read_profile()

        if new_profile == self.active_profile:
            return

        old_profile = self.active_profile
        self.active_profile = new_profile

        # Prevent packets from the previous experimental condition from
        # leaking into the next condition.
        discarded = len(self.pending)
        self.pending.clear()

        self.window_received = 0
        self.window_dropped = 0
        self.last_scan_stamp = None
        self.last_sampled_delay = 0.0

        self.get_logger().info(
            f"profile changed: {old_profile} -> {new_profile}; "
            f"cleared {discarded} queued scans"
        )

    def scan_callback(self, msg):
        self._sync_profile()

        cfg = PROFILES[self.active_profile]

        self.window_received += 1

        loss_probability = self.rng.uniform(
            cfg["loss_min"],
            cfg["loss_max"],
        )

        if self.rng.random() < loss_probability:
            self.window_dropped += 1
            return

        delay = self.rng.uniform(
            cfg["delay_min"],
            cfg["delay_max"],
        )

        now_ns = self.get_clock().now().nanoseconds
        release_ns = now_ns + int(delay * 1e9)

        self.pending.append(
            (release_ns, msg, delay)
        )

    def flush_queue(self):
        self._sync_profile()

        now_ns = self.get_clock().now().nanoseconds

        # FIFO is deliberate.
        #
        # Only inspect the oldest accepted scan. If it is not ready yet,
        # later scans are not allowed to overtake it.
        #
        # Once the oldest is ready, multiple already-due messages may be
        # released during the same timer callback, but always in input order.
        while self.pending:

            release_ns, msg, sampled_delay = self.pending[0]

            if release_ns > now_ns:
                break

            self.pending.popleft()

            # Preserve ORIGINAL acquisition timestamp.
            # Do not restamp delayed sensor information.
            self.scan_pub.publish(msg)

            self.last_sampled_delay = float(sampled_delay)
            self.last_scan_stamp = stamp_to_sec(
                msg.header.stamp
            )

    def publish_diagnostics(self):
        self._sync_profile()

        now_sec = (
            self.get_clock().now().nanoseconds / 1e9
        )

        if self.window_received > 0:
            actual_loss = (
                self.window_dropped /
                self.window_received
            )
        else:
            actual_loss = 0.0

        if self.last_scan_stamp is None:
            scan_age = 0.0
        else:
            scan_age = max(
                0.0,
                now_sec - self.last_scan_stamp
            )

        msg = Float32MultiArray()

        # [sampled latency,
        #  actual loss in current diagnostic window,
        #  actual age of most recently published degraded scan,
        #  command age (unused for Stage 0)]
        msg.data = [
            float(self.last_sampled_delay),
            float(actual_loss),
            float(scan_age),
            0.0,
        ]

        self.age_pub.publish(msg)

        profile_msg = String()
        profile_msg.data = self.active_profile
        self.profile_pub.publish(profile_msg)

        self.window_received = 0
        self.window_dropped = 0


def main(args=None):
    rclpy.init(args=args)

    node = NetworkEmulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
