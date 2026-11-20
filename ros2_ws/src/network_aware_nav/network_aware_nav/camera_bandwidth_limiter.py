#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message

from sensor_msgs.msg import Image
from std_msgs.msg import (
    String,
    Float32,
    Float32MultiArray,
)


CAP_MBPS = {
    'normal': None,
    'mild': 10.0,
    'moderate': 5.0,
    'severe': 1.0,
}


class CameraBandwidthLimiter(Node):

    def __init__(self):
        super().__init__('camera_bandwidth_limiter')

        self.declare_parameter(
            'bucket_burst_s',
            1.0,
        )

        self.bucket_burst_s = float(
            self.get_parameter(
                'bucket_burst_s'
            ).value
        )

        if self.bucket_burst_s <= 0.0:
            raise ValueError(
                'bucket_burst_s must be > 0'
            )

        self.active_profile = 'normal'

        # Token-bucket state.
        self.tokens_bits = 0.0
        self.last_refill_ns = None

        # Diagnostic window.
        self.window_received = 0
        self.window_forwarded = 0

        self.image_pub = self.create_publisher(
            Image,
            '/camera/tx_bw',
            qos_profile_sensor_data,
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/tx',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.profile_sub = self.create_subscription(
            String,
            '/network_profile',
            self.profile_callback,
            10,
        )

        self.cap_pub = self.create_publisher(
            Float32,
            '/camera_bw_cap_mbps',
            10,
        )

        self.drop_pub = self.create_publisher(
            Float32,
            '/camera_bw_drop_ratio',
            10,
        )

        # [cap_mbps,
        #  drop_ratio,
        #  received_count,
        #  forwarded_count]
        self.diag_pub = self.create_publisher(
            Float32MultiArray,
            '/camera_bw_diag',
            10,
        )

        self.create_timer(
            0.5,
            self.publish_diagnostics,
        )

        self.get_logger().info(
            'CameraBandwidthLimiter started: '
            'token-bucket model, '
            f'bucket_burst_s={self.bucket_burst_s:.3f}, '
            '/camera/tx -> /camera/tx_bw'
        )

    def reset_bucket(self):
        self.tokens_bits = 0.0
        self.last_refill_ns = None

        self.window_received = 0
        self.window_forwarded = 0

    def profile_callback(self, msg):

        profile = str(msg.data).lower()

        if profile not in CAP_MBPS:
            self.get_logger().warning(
                f"Unknown network profile "
                f"'{profile}', ignored"
            )
            return

        if profile == self.active_profile:
            return

        old = self.active_profile
        self.active_profile = profile

        self.reset_bucket()

        cap = CAP_MBPS[profile]

        self.get_logger().info(
            f'Bandwidth condition changed: '
            f'{old} -> {profile}, '
            f'cap='
            f'{"unlimited" if cap is None else str(cap) + " Mbps"}'
        )

    @staticmethod
    def message_size_bits(msg):

        try:
            message_bytes = len(
                serialize_message(msg)
            )
        except Exception:
            message_bytes = (
                len(msg.data) + 256
            )

        return float(message_bytes) * 8.0

    def image_callback(self, msg):

        self.window_received += 1

        cap_mbps = CAP_MBPS[
            self.active_profile
        ]

        # Normal profile: unlimited.
        if cap_mbps is None:
            self.image_pub.publish(msg)

            self.window_forwarded += 1

            return

        message_bits = (
            self.message_size_bits(msg)
        )

        cap_bps = (
            float(cap_mbps) * 1e6
        )

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        # Capacity permits accumulation while avoiding
        # an unrealistically unlimited burst.
        #
        # Always allow enough capacity for at least
        # two complete frames.
        bucket_capacity_bits = max(
            cap_bps * self.bucket_burst_s,
            2.0 * message_bits,
        )

        if self.last_refill_ns is None:

            self.last_refill_ns = now_ns

            # Initial credit of exactly one frame:
            # avoids a large startup burst while
            # permitting immediate first delivery.
            self.tokens_bits = min(
                message_bits,
                bucket_capacity_bits,
            )

        else:

            dt = max(
                0.0,
                (
                    now_ns
                    - self.last_refill_ns
                ) * 1e-9,
            )

            self.last_refill_ns = now_ns

            self.tokens_bits = min(
                bucket_capacity_bits,
                self.tokens_bits
                + cap_bps * dt,
            )

        if self.tokens_bits >= message_bits:

            self.image_pub.publish(msg)

            self.window_forwarded += 1

            self.tokens_bits -= (
                message_bits
            )

        # Otherwise:
        # frame is intentionally dropped because
        # the configured application-channel budget
        # is not currently sufficient.

    def publish_diagnostics(self):

        cap = CAP_MBPS[
            self.active_profile
        ]

        cap_msg = Float32()

        cap_msg.data = (
            -1.0
            if cap is None
            else float(cap)
        )

        self.cap_pub.publish(
            cap_msg
        )

        if self.window_received > 0:

            drop_ratio = (
                1.0
                - (
                    self.window_forwarded
                    / self.window_received
                )
            )

        else:
            drop_ratio = 0.0

        drop_msg = Float32()
        drop_msg.data = float(drop_ratio)

        self.drop_pub.publish(
            drop_msg
        )

        diag = Float32MultiArray()

        diag.data = [
            -1.0
            if cap is None
            else float(cap),

            float(drop_ratio),

            float(
                self.window_received
            ),

            float(
                self.window_forwarded
            ),
        ]

        self.diag_pub.publish(
            diag
        )

        self.window_received = 0
        self.window_forwarded = 0


def main(args=None):

    rclpy.init(args=args)

    node = CameraBandwidthLimiter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
