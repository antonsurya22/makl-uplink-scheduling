#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8, Bool, String
from naw_interfaces.msg import StateVector

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class SafetySupervisor(Node):
    """
    A3d version.

    AUDIT MODE ONLY:
      /action_requested
              |
              v
      safety_supervisor
              |
              +--> /action_cmd
              +--> /action_executed
              +--> /safety_override
              +--> /safety_reason

    No safety threshold is enforced yet.

    This stage exists to validate the distinction between:
      requested action  a_t
      executed action   a_exec_t

    Real enforcement is added only after d_hard and age_crit
    have been experimentally calibrated.
    """

    NUM_ACTIONS = 9

    def __init__(self):
        super().__init__('safety_supervisor')

        self.declare_parameter('audit_only', True)

        audit_only = bool(
            self.get_parameter('audit_only').value
        )

        # Refuse to pretend that enforcement exists.
        if not audit_only:
            raise RuntimeError(
                'Safety enforcement is not calibrated yet. '
                'Run with audit_only:=true.'
            )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Input from policy / manual test.
        self.action_sub = self.create_subscription(
            UInt8,
            '/action_requested',
            self.action_callback,
            command_qos,
        )

        # State is subscribed now so that A3e can add
        # calibrated safety logic without changing architecture.
        self.state_sub = self.create_subscription(
            StateVector,
            '/state_vector',
            self.state_callback,
            10,
        )

        # Executed command toward Action Dispatcher.
        self.action_cmd_pub = self.create_publisher(
            UInt8,
            '/action_cmd',
            command_qos,
        )

        # Diagnostics / logging.
        self.executed_pub = self.create_publisher(
            UInt8,
            '/action_executed',
            state_qos,
        )

        self.override_pub = self.create_publisher(
            Bool,
            '/safety_override',
            state_qos,
        )

        self.reason_pub = self.create_publisher(
            String,
            '/safety_reason',
            state_qos,
        )

        self.latest_state = None
        self.last_requested = None
        self.last_executed = None

        self.get_logger().warning(
            'Safety Supervisor started in AUDIT ONLY mode. '
            'No safety threshold is enforced yet.'
        )

    def state_callback(self, msg: StateVector):
        self.latest_state = msg

    def action_callback(self, msg: UInt8):
        requested = int(msg.data)

        if requested < 0 or requested >= self.NUM_ACTIONS:
            self.get_logger().error(
                f'Rejected invalid requested action={requested}; '
                'valid range is 0..8'
            )
            return

        #
        # A3d:
        # pass-through only.
        #
        # A3e will replace this section with calibrated:
        #
        #     executed = g(requested, state)
        #
        executed = requested
        overridden = False
        reason = 'audit_pass_through'

        cmd = UInt8()
        cmd.data = executed
        self.action_cmd_pub.publish(cmd)

        executed_msg = UInt8()
        executed_msg.data = executed
        self.executed_pub.publish(executed_msg)

        override_msg = Bool()
        override_msg.data = overridden
        self.override_pub.publish(override_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

        self.last_requested = requested
        self.last_executed = executed

        self.get_logger().info(
            f'requested={requested} '
            f'executed={executed} '
            f'override={overridden} '
            f'reason={reason}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SafetySupervisor()

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
