#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import UInt8, String


PROFILE_NORMAL = 0
PROFILE_FRESHNESS = 1
PROFILE_LOW_BANDWIDTH = 2


PROFILE_NAMES = {
    PROFILE_NORMAL: 'normal',
    PROFILE_FRESHNESS: 'freshness',
    PROFILE_LOW_BANDWIDTH: 'low_bandwidth',
}


class QoSProfileManager(Node):

    def __init__(self):
        super().__init__('qos_profile_manager')

        self.declare_parameter(
            'initial_profile',
            PROFILE_NORMAL,
        )

        initial = int(
            self.get_parameter('initial_profile').value
        )

        if initial not in PROFILE_NAMES:
            self.get_logger().warning(
                f'Invalid initial profile {initial}; using NORMAL'
            )
            initial = PROFILE_NORMAL

        self.current_profile = initial

        # Late subscribers such as state_aggregator and relay nodes
        # must immediately know the currently active profile.
        profile_qos = QoSProfile(depth=1)
        profile_qos.reliability = ReliabilityPolicy.RELIABLE
        profile_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.profile_pub = self.create_publisher(
            UInt8,
            '/qos_profile',
            profile_qos,
        )

        self.profile_name_pub = self.create_publisher(
            String,
            '/qos_profile_name',
            profile_qos,
        )

        self.cmd_sub = self.create_subscription(
            UInt8,
            '/qos_profile_cmd',
            self.command_callback,
            10,
        )

        # Periodic publication is useful for logging/rosbag,
        # while transient-local preserves the latest state.
        self.create_timer(
            0.5,
            self.publish_profile,
        )

        self.publish_profile()

        self.get_logger().info(
            'QoSProfileManager started: '
            f'profile={self.current_profile} '
            f'({PROFILE_NAMES[self.current_profile]})'
        )

    def command_callback(self, msg):
        requested = int(msg.data)

        if requested not in PROFILE_NAMES:
            self.get_logger().warning(
                f'Invalid QoS profile {requested}; ignored'
            )
            return

        if requested == self.current_profile:
            self.publish_profile()
            return

        old = self.current_profile
        self.current_profile = requested

        self.get_logger().info(
            f'QoS profile changed: '
            f'{old} ({PROFILE_NAMES[old]}) -> '
            f'{requested} ({PROFILE_NAMES[requested]})'
        )

        self.publish_profile()

    def publish_profile(self):
        profile_msg = UInt8()
        profile_msg.data = int(self.current_profile)
        self.profile_pub.publish(profile_msg)

        name_msg = String()
        name_msg.data = PROFILE_NAMES[self.current_profile]
        self.profile_name_pub.publish(name_msg)


def main(args=None):
    rclpy.init(args=args)

    node = QoSProfileManager()

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
