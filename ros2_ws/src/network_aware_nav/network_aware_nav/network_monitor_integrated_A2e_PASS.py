#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Float32MultiArray
from naw_interfaces.msg import NetworkState


class NetworkMonitorIntegrated(Node):

    def __init__(self):
        super().__init__('network_monitor_integrated')

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('jitter_window', 20)
        self.declare_parameter('source_timeout', 2.0)

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        self.jitter_window = int(
            self.get_parameter('jitter_window').value
        )

        self.source_timeout = float(
            self.get_parameter('source_timeout').value
        )

        self.latency = None
        self.packet_loss = None
        self.bandwidth = None

        self.topic_age_rx_ns = None
        self.bandwidth_rx_ns = None

        self.latency_history = deque(
            maxlen=max(2, self.jitter_window)
        )

        self.topic_age_sub = self.create_subscription(
            Float32MultiArray,
            '/topic_age',
            self.topic_age_callback,
            10
        )

        self.bandwidth_sub = self.create_subscription(
            Float32,
            '/observed_bandwidth_mbps',
            self.bandwidth_callback,
            10
        )

        self.pub = self.create_publisher(
            NetworkState,
            '/network_state',
            10
        )

        self.create_timer(
            1.0 / max(0.1, self.publish_rate),
            self.publish_state
        )

        self.get_logger().info(
            'Integrated network monitor started: '
            '/topic_age + /observed_bandwidth_mbps '
            '-> /network_state_candidate'
        )

    def topic_age_callback(self, msg):
        if len(msg.data) < 2:
            return

        self.latency = float(msg.data[0])
        self.packet_loss = float(msg.data[1])

        self.latency_history.append(
            self.latency
        )

        self.topic_age_rx_ns = (
            self.get_clock().now().nanoseconds
        )

    def bandwidth_callback(self, msg):
        self.bandwidth = float(msg.data)

        self.bandwidth_rx_ns = (
            self.get_clock().now().nanoseconds
        )

    def compute_jitter(self):
        if len(self.latency_history) < 2:
            return 0.0

        diffs = []

        values = list(self.latency_history)

        for i in range(1, len(values)):
            diffs.append(
                abs(values[i] - values[i - 1])
            )

        if not diffs:
            return 0.0

        return sum(diffs) / len(diffs)

    def source_fresh(self, rx_ns):
        if rx_ns is None:
            return False

        now_ns = self.get_clock().now().nanoseconds

        age_sec = (
            now_ns - rx_ns
        ) * 1e-9

        return age_sec <= self.source_timeout

    def publish_state(self):
        if self.latency is None:
            return

        if self.packet_loss is None:
            return

        if self.bandwidth is None:
            return

        if not self.source_fresh(self.topic_age_rx_ns):
            return

        if not self.source_fresh(self.bandwidth_rx_ns):
            return

        msg = NetworkState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            'network_monitor_integrated'
        )

        msg.latency = float(self.latency)
        msg.packet_loss = float(
            self.packet_loss
        )
        msg.bandwidth = float(
            self.bandwidth
        )
        msg.jitter = float(
            self.compute_jitter()
        )

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = NetworkMonitorIntegrated()

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
