#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class ActionDispatcher(Node):

    NUM_NAV_MODES = 3
    NUM_QOS_PROFILES = 3
    NUM_ACTIONS = NUM_NAV_MODES * NUM_QOS_PROFILES

    def __init__(self):
        super().__init__('action_dispatcher')

        # Commands are explicit control messages: reliable delivery.
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Diagnostic state of the last successfully dispatched action.
        # Transient local lets a late subscriber immediately see it.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.nav_pub = self.create_publisher(
            UInt8,
            '/nav_mode_cmd',
            command_qos,
        )

        self.qos_pub = self.create_publisher(
            UInt8,
            '/qos_profile_cmd',
            command_qos,
        )

        self.action_state_pub = self.create_publisher(
            UInt8,
            '/action_dispatched',
            state_qos,
        )

        self.action_sub = self.create_subscription(
            UInt8,
            '/action_cmd',
            self.action_callback,
            command_qos,
        )

        self.last_action = None

        self.get_logger().info(
            'Action Dispatcher ready: '
            'action 0..8 -> nav=action//3, qos=action%3'
        )

    def action_callback(self, msg: UInt8):
        action = int(msg.data)

        if action < 0 or action >= self.NUM_ACTIONS:
            self.get_logger().error(
                f'Rejected invalid action={action}; '
                f'valid range is 0..{self.NUM_ACTIONS - 1}'
            )
            return

        nav_mode = action // self.NUM_QOS_PROFILES
        qos_profile = action % self.NUM_QOS_PROFILES

        nav_msg = UInt8()
        nav_msg.data = nav_mode

        qos_msg = UInt8()
        qos_msg.data = qos_profile

        self.nav_pub.publish(nav_msg)
        self.qos_pub.publish(qos_msg)

        dispatched_msg = UInt8()
        dispatched_msg.data = action
        self.action_state_pub.publish(dispatched_msg)

        self.last_action = action

        self.get_logger().info(
            f'Dispatched action={action} '
            f'-> nav_mode={nav_mode}, qos_profile={qos_profile}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = ActionDispatcher()

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
