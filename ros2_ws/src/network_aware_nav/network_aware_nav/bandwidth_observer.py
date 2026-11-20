#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message

from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class BandwidthObserver(Node):

    def __init__(self):
        super().__init__('bandwidth_observer')

        self.declare_parameter('window_size', 5)

        self.window_size = int(
            self.get_parameter('window_size').value
        )

        self.last_rx_ns = None

        self.samples = deque(
            maxlen=max(1, self.window_size)
        )

        self.last_bandwidth = 0.0

        self.sub = self.create_subscription(
            Image,
            '/camera/tx_bw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Float32,
            '/observed_bandwidth_mbps',
            10
        )

        # Publish current estimate at 2 Hz so it can later align
        # with the RL/state update period.
        self.create_timer(
            0.5,
            self.publish_estimate
        )

        self.get_logger().info(
            'BandwidthObserver started: '
            '/camera/tx_bw -> /observed_bandwidth_mbps'
        )

    def image_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds

        try:
            size_bytes = len(serialize_message(msg))
        except Exception:
            size_bytes = len(msg.data) + 256

        if self.last_rx_ns is not None:
            dt = (now_ns - self.last_rx_ns) * 1e-9

            if dt > 0.0:
                mbps = (
                    size_bytes * 8.0 /
                    dt /
                    1e6
                )

                self.samples.append(mbps)

                self.last_bandwidth = (
                    sum(self.samples) /
                    len(self.samples)
                )

        self.last_rx_ns = now_ns

    def publish_estimate(self):
        msg = Float32()
        msg.data = float(self.last_bandwidth)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BandwidthObserver()

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
