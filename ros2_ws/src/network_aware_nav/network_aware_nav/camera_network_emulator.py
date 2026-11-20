#!/usr/bin/env python3

import random
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import Image
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    String,
    UInt8,
)


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


QOS_NORMAL = 0
QOS_FRESHNESS = 1
QOS_LOW_BANDWIDTH = 2


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CameraNetworkEmulator(Node):

    def __init__(self):
        super().__init__('camera_network_emulator')

        self.declare_parameter('seed', 13)

        # B1.2a:
        # Application-level approximation of additional delivery
        # attempts for the RELIABLE communication profile.
        self.declare_parameter(
            'reliable_max_retries',
            2,
        )

        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(seed)

        self.reliable_max_retries = int(
            self.get_parameter(
                'reliable_max_retries'
            ).value
        )

        self.reliable_max_retries = max(
            0,
            self.reliable_max_retries,
        )

        # Channel condition comes from network_emulator.
        self.active_profile = 'normal'

        # Communication profile comes from QoSProfileManager.
        self.active_qos_profile = QOS_NORMAL

        # FIFO queue:
        # (release_time_ns, Image, sampled_delay)
        self.pending = deque()

        self.window_received = 0
        self.window_dropped = 0

        self.last_sampled_delay = 0.0
        self.last_published_stamp = None

        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_degraded',
            qos_profile_sensor_data,
        )

        self.age_pub = self.create_publisher(
            Float32,
            '/camera_age',
            10,
        )

        # Diagnostic vector:
        # [0] sampled/effective delay [s]
        # [1] window loss ratio
        # [2] camera AoI [s]
        # [3] frames received in diagnostic window
        # [4] frames dropped in diagnostic window
        #
        # Fields 0..2 are kept unchanged for backward compatibility.
        self.diag_pub = self.create_publisher(
            Float32MultiArray,
            '/camera_network_diag',
            10,
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/tx_bw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.profile_sub = self.create_subscription(
            String,
            '/network_profile',
            self.profile_callback,
            10,
        )

        qos_state_qos = QoSProfile(depth=1)
        qos_state_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )
        qos_state_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.qos_profile_sub = self.create_subscription(
            UInt8,
            '/qos_profile',
            self.qos_profile_callback,
            qos_state_qos,
        )

        # Queue release check at 100 Hz simulation time.
        self.flush_timer = self.create_timer(
            0.01,
            self.flush_queue,
        )

        # Diagnostics @ 2 Hz.
        self.diag_timer = self.create_timer(
            0.5,
            self.publish_diagnostics,
        )

        self.get_logger().info(
            f'camera network emulator started: '
            f'profile={self.active_profile}, seed={seed}, '
            f'reliable_max_retries={self.reliable_max_retries}, '
            f'FIFO ordering enabled'
        )

    def profile_callback(self, msg):
        new_profile = str(msg.data).lower()

        if new_profile not in PROFILES:
            self.get_logger().warning(
                f"unknown network profile '{new_profile}', ignoring"
            )
            return

        if new_profile == self.active_profile:
            return

        old_profile = self.active_profile
        self.active_profile = new_profile

        # Prevent samples from previous experimental condition
        # leaking into the next one.
        discarded = len(self.pending)
        self.pending.clear()

        self.window_received = 0
        self.window_dropped = 0
        self.last_sampled_delay = 0.0
        self.last_published_stamp = None

        self.get_logger().info(
            f'profile changed: {old_profile} -> {new_profile}; '
            f'cleared {discarded} queued images'
        )

    def qos_profile_callback(self, msg):
        requested = int(msg.data)

        if requested not in (
            QOS_NORMAL,
            QOS_FRESHNESS,
            QOS_LOW_BANDWIDTH,
        ):
            self.get_logger().warning(
                f'invalid QoS profile {requested}; ignored'
            )
            return

        if requested == self.active_qos_profile:
            return

        old = self.active_qos_profile
        self.active_qos_profile = requested

        # Do not clear the channel queue here.
        # Frames already in flight remain part of the previous
        # transport decision.
        self.get_logger().info(
            f'QoS-aware channel mode changed: '
            f'{old} -> {requested}'
        )

    def image_callback(self, msg):
        cfg = PROFILES[self.active_profile]

        self.window_received += 1

        # BEST_EFFORT profiles receive one transmission attempt.
        #
        # Q0 / RELIABLE receives additional retransmission-like
        # attempts. This is an application-level emulation of the
        # reliability consequence, not an RTPS implementation.
        if self.active_qos_profile == QOS_NORMAL:
            max_attempts = (
                1 + self.reliable_max_retries
            )
        else:
            max_attempts = 1

        total_delay = 0.0
        delivered = False

        for _attempt in range(max_attempts):

            # Every transmission attempt experiences the
            # currently selected channel delay.
            attempt_delay = self.rng.uniform(
                cfg['delay_min'],
                cfg['delay_max'],
            )

            total_delay += attempt_delay

            loss_probability = self.rng.uniform(
                cfg['loss_min'],
                cfg['loss_max'],
            )

            lost = (
                self.rng.random()
                < loss_probability
            )

            if not lost:
                delivered = True
                break

        if not delivered:
            self.window_dropped += 1
            return

        now_ns = self.get_clock().now().nanoseconds

        release_ns = (
            now_ns
            + int(total_delay * 1e9)
        )

        # FIFO ordering intentionally remains enabled.
        #
        # For RELIABLE mode this also permits additional retry
        # delay to create head-of-line freshness cost.
        self.pending.append(
            (
                release_ns,
                msg,
                total_delay,
            )
        )

    def flush_queue(self):
        now_ns = self.get_clock().now().nanoseconds

        while self.pending:
            release_ns, msg, sampled_delay = self.pending[0]

            if release_ns > now_ns:
                break

            self.pending.popleft()

            # IMPORTANT:
            # Preserve original sensor timestamp.
            # Do not restamp delayed information.
            self.image_pub.publish(msg)

            self.last_sampled_delay = float(sampled_delay)
            self.last_published_stamp = stamp_to_sec(
                msg.header.stamp
            )

    def get_camera_age(self):
        if self.last_published_stamp is None:
            return 0.0

        now_sec = (
            self.get_clock().now().nanoseconds / 1e9
        )

        return max(
            0.0,
            now_sec - self.last_published_stamp
        )

    def publish_diagnostics(self):
        if self.window_received > 0:
            actual_loss = (
                self.window_dropped /
                self.window_received
            )
        else:
            actual_loss = 0.0

        camera_age = self.get_camera_age()

        age_msg = Float32()
        age_msg.data = float(camera_age)
        self.age_pub.publish(age_msg)

        diag = Float32MultiArray()
        diag.data = [
            float(self.last_sampled_delay),
            float(actual_loss),
            float(camera_age),
            float(self.window_received),
            float(self.window_dropped),
        ]
        self.diag_pub.publish(diag)

        self.window_received = 0
        self.window_dropped = 0


def main(args=None):
    rclpy.init(args=args)

    node = CameraNetworkEmulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
