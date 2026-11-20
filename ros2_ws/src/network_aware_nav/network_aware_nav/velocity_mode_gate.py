#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


MODE_NORMAL = 0
MODE_SLOW = 1
MODE_SAFE_STOP = 2


class VelocityModeGate(Node):

    def __init__(self):
        super().__init__('velocity_mode_gate')

        self.declare_parameter('mode_timeout', 1.0)
        self.declare_parameter('stop_publish_rate', 20.0)

        self.mode_timeout = float(
            self.get_parameter('mode_timeout').value
        )

        stop_rate = float(
            self.get_parameter('stop_publish_rate').value
        )

        # Fail-safe startup: STOP until a valid mode is received.
        self.current_mode = MODE_SAFE_STOP
        self.last_mode_time = None

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mode_sub = self.create_subscription(
            UInt8,
            '/nav_mode',
            self.mode_callback,
            mode_qos,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            self.cmd_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_mode',
            10,
        )

        self.create_timer(
            1.0 / max(1.0, stop_rate),
            self.stop_timer_callback,
        )

        self.get_logger().info(
            'VelocityModeGate started: '
            'input=/cmd_vel_smoothed, '
            'output=/cmd_vel_mode, '
            'fail-safe default=SAFE_STOP'
        )

    def mode_callback(self, msg):
        mode = int(msg.data)

        if mode not in (
            MODE_NORMAL,
            MODE_SLOW,
            MODE_SAFE_STOP,
        ):
            self.get_logger().warning(
                f'Invalid nav mode {mode}; forcing SAFE_STOP'
            )
            mode = MODE_SAFE_STOP

        previous = self.current_mode
        self.current_mode = mode
        self.last_mode_time = self.get_clock().now()

        if mode != previous:
            self.get_logger().info(
                f'Navigation mode changed: {previous} -> {mode}'
            )

        # Immediately issue zero on transition to safe-stop.
        if mode == MODE_SAFE_STOP:
            self.publish_zero()

    def mode_is_fresh(self):
        if self.last_mode_time is None:
            return False

        age = (
            self.get_clock().now().nanoseconds -
            self.last_mode_time.nanoseconds
        ) / 1e9

        return age <= self.mode_timeout

    def cmd_callback(self, msg):
        # Normal and Slow both pass through.
        # Their speed limit is enforced upstream by controller_server.
        if (
            self.current_mode in (MODE_NORMAL, MODE_SLOW)
            and self.mode_is_fresh()
        ):
            self.cmd_pub.publish(msg)
        else:
            self.publish_zero()

    def stop_timer_callback(self):
        # Keep publishing explicit zero while stopped or
        # when mode information becomes stale.
        if (
            self.current_mode == MODE_SAFE_STOP
            or not self.mode_is_fresh()
        ):
            self.publish_zero()

    def publish_zero(self):
        zero = Twist()
        self.cmd_pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityModeGate()

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
