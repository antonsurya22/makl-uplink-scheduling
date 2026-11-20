#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8


PROFILE_NORMAL = 0
PROFILE_FRESHNESS = 1
PROFILE_LOW_BANDWIDTH = 2


class AdaptiveCameraRelay(Node):

    def __init__(self):
        super().__init__('adaptive_camera_relay')

        self.declare_parameter('low_bandwidth_rate', 2.0)

        self.low_bandwidth_rate = float(
            self.get_parameter('low_bandwidth_rate').value
        )

        self.current_profile = PROFILE_NORMAL
        self.last_tx_ns = None
        self.tx_pub = None

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.profile_sub = self.create_subscription(
            UInt8,
            '/qos_profile',
            self.profile_callback,
            mode_qos,
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.create_tx_publisher()

        self.get_logger().info(
            'AdaptiveCameraRelay started: '
            'input=/camera/image_raw, output=/camera/tx'
        )

    def qos_for_profile(self):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        qos.durability = DurabilityPolicy.VOLATILE

        if self.current_profile == PROFILE_NORMAL:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.depth = 10

        elif self.current_profile == PROFILE_FRESHNESS:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            qos.depth = 1

        else:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            qos.depth = 1

        return qos

    def create_tx_publisher(self):
        if self.tx_pub is not None:
            self.destroy_publisher(self.tx_pub)

        self.tx_pub = self.create_publisher(
            Image,
            '/camera/tx',
            self.qos_for_profile(),
        )

    def profile_callback(self, msg):
        requested = int(msg.data)

        if requested not in (
            PROFILE_NORMAL,
            PROFILE_FRESHNESS,
            PROFILE_LOW_BANDWIDTH,
        ):
            self.get_logger().warning(
                f'Invalid QoS profile {requested}; ignored'
            )
            return

        if requested == self.current_profile:
            return

        old = self.current_profile
        self.current_profile = requested
        self.last_tx_ns = None

        # DDS QoS cannot simply be mutated on an existing publisher,
        # therefore recreate the publisher with the selected QoS.
        self.create_tx_publisher()

        self.get_logger().info(
            f'Camera communication profile changed: '
            f'{old} -> {requested}'
        )

    def image_callback(self, msg):
        if self.current_profile in (
            PROFILE_NORMAL,
            PROFILE_FRESHNESS,
        ):
            # Preserve original timestamp.
            self.tx_pub.publish(msg)
            return

        # LOW-BANDWIDTH:
        # retain full image format but reduce transmission rate.
        now_ns = self.get_clock().now().nanoseconds

        interval_ns = int(
            (1.0 / max(0.1, self.low_bandwidth_rate))
            * 1e9
        )

        if self.last_tx_ns is None:
            self.tx_pub.publish(msg)
            self.last_tx_ns = now_ns
            return

        if (now_ns - self.last_tx_ns) >= interval_ns:
            self.tx_pub.publish(msg)
            self.last_tx_ns = now_ns


def main(args=None):
    rclpy.init(args=args)

    node = AdaptiveCameraRelay()

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
