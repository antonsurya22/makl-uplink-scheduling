#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

from naw_interfaces.msg import NetworkState


class NetworkMonitor(Node):

    def __init__(self):
        super().__init__('network_monitor')

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('jitter_window', 20)

        publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        jitter_window = int(
            self.get_parameter('jitter_window').value
        )

        # Latest measured network values
        self.latency = 0.0
        self.packet_loss = 0.0

        # Diagnostic latency history for jitter calculation
        self.latency_history = deque(
            maxlen=max(2, jitter_window)
        )

        # Application-level throughput measurement
        self.byte_count = 0
        self.last_bw_time = self.get_clock().now()

        self.bandwidth_mbps = 0.0

        self.create_subscription(
            Float32MultiArray,
            '/topic_age',
            self.topic_age_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan_degraded',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            NetworkState,
            '/network_state',
            10
        )

        period = 1.0 / max(0.1, publish_rate)

        self.create_timer(
            period,
            self.publish_state
        )

        self.get_logger().info(
            f'NetworkMonitor started at {publish_rate:.2f} Hz'
        )

    def topic_age_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                '/topic_age contains fewer than 2 elements'
            )
            return

        latency = max(0.0, float(msg.data[0]))
        loss = min(
            1.0,
            max(0.0, float(msg.data[1]))
        )

        self.latency = latency
        self.packet_loss = loss

        self.latency_history.append(latency)

    def scan_callback(self, msg):
        """
        Estimate observed application throughput from serialized
        /scan_degraded traffic.

        This is stream throughput, not physical PHY/link capacity.
        """
        try:
            self.byte_count += len(serialize_message(msg))
        except Exception as exc:
            self.get_logger().warning(
                f'Could not serialize LaserScan: {exc}'
            )

    def calculate_jitter(self):
        """
        Mean absolute difference between consecutive latency samples.
        Unit: seconds.
        """
        if len(self.latency_history) < 2:
            return 0.0

        values = list(self.latency_history)

        diffs = [
            abs(values[i] - values[i - 1])
            for i in range(1, len(values))
        ]

        if not diffs:
            return 0.0

        return sum(diffs) / len(diffs)

    def update_bandwidth(self):
        now = self.get_clock().now()

        dt = (
            now.nanoseconds -
            self.last_bw_time.nanoseconds
        ) / 1e9

        if dt <= 0.0:
            return

        # bytes -> bits -> megabits / second
        self.bandwidth_mbps = (
            self.byte_count * 8.0
        ) / dt / 1e6

        self.byte_count = 0
        self.last_bw_time = now

    def publish_state(self):
        self.update_bandwidth()

        msg = NetworkState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'network_monitor'

        # Units:
        # latency     : seconds
        # packet_loss : ratio [0,1]
        # bandwidth   : Mbps
        # jitter      : seconds
        msg.latency = float(self.latency)
        msg.packet_loss = float(self.packet_loss)
        msg.bandwidth = float(self.bandwidth_mbps)
        msg.jitter = float(self.calculate_jitter())

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = NetworkMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
