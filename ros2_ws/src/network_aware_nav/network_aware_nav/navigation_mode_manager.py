#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import UInt8
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


MODE_NORMAL = 0
MODE_SLOW = 1
MODE_SAFE_STOP = 2


class NavigationModeManager(Node):

    def __init__(self):
        super().__init__('navigation_mode_manager')

        self.declare_parameter('normal_speed', 0.22)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('initial_mode', MODE_NORMAL)

        self.normal_speed = float(
            self.get_parameter('normal_speed').value
        )
        self.slow_speed = float(
            self.get_parameter('slow_speed').value
        )

        self.current_mode = int(
            self.get_parameter('initial_mode').value
        )

        # Late subscribers should still know the current mode.
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mode_pub = self.create_publisher(
            UInt8,
            '/nav_mode',
            mode_qos,
        )

        self.mode_cmd_sub = self.create_subscription(
            UInt8,
            '/nav_mode_cmd',
            self.mode_cmd_callback,
            10,
        )

        self.param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )

        self.request_in_progress = False
        self.pending_mode = None
        self.initialized = False

        # Publish current mode periodically as well.
        self.create_timer(0.5, self.publish_mode)

        # Retry initialization until controller_server exists.
        self.create_timer(1.0, self.initialize_controller)

        self.get_logger().info(
            'NavigationModeManager started: '
            f'Normal={self.normal_speed:.2f} m/s, '
            f'Slow={self.slow_speed:.2f} m/s, '
            'Safe-stop=mode 2'
        )

    def make_double_parameter(self, name, value):
        p = Parameter()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        )
        return p

    def publish_mode(self):
        msg = UInt8()
        msg.data = int(self.current_mode)
        self.mode_pub.publish(msg)

    def initialize_controller(self):
        if self.initialized or self.request_in_progress:
            return

        if not self.param_client.service_is_ready():
            self.get_logger().info(
                'Waiting for /controller_server/set_parameters...'
            )
            return

        initial = self.current_mode

        if initial not in (MODE_NORMAL, MODE_SLOW):
            # Start controller in known safe low-speed configuration.
            initial = MODE_SLOW

        self.apply_speed_mode(initial, initialization=True)

    def mode_cmd_callback(self, msg):
        requested = int(msg.data)

        if requested not in (
            MODE_NORMAL,
            MODE_SLOW,
            MODE_SAFE_STOP,
        ):
            self.get_logger().warning(
                f'Invalid navigation mode {requested}; ignored'
            )
            return

        if requested == MODE_SAFE_STOP:
            # Publish the logical mode now.
            # Actual zero-velocity gating is added in the next A2b2 step.
            self.current_mode = MODE_SAFE_STOP
            self.publish_mode()

            self.get_logger().warning(
                'Mode 2 SAFE_STOP selected. '
                'Mode state updated; velocity gate must enforce zero motion.'
            )
            return

        self.apply_speed_mode(requested)

    def apply_speed_mode(self, mode, initialization=False):
        if self.request_in_progress:
            self.pending_mode = mode
            return

        if not self.param_client.service_is_ready():
            self.get_logger().warning(
                'controller_server parameter service unavailable'
            )
            self.pending_mode = mode
            return

        if mode == MODE_NORMAL:
            speed = self.normal_speed
            label = 'NORMAL'
        elif mode == MODE_SLOW:
            speed = self.slow_speed
            label = 'SLOW'
        else:
            return

        request = SetParameters.Request()

        request.parameters = [
            self.make_double_parameter(
                'FollowPath.max_vel_x',
                speed,
            ),
            self.make_double_parameter(
                'FollowPath.max_speed_xy',
                speed,
            ),
        ]

        self.request_in_progress = True

        future = self.param_client.call_async(request)

        future.add_done_callback(
            lambda f: self.parameter_response(
                f,
                mode,
                speed,
                label,
                initialization,
            )
        )

    def parameter_response(
        self,
        future,
        mode,
        speed,
        label,
        initialization,
    ):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Parameter update failed: {exc}'
            )
            return

        success = (
            response is not None
            and len(response.results) == 2
            and all(r.successful for r in response.results)
        )

        if not success:
            reasons = []

            if response is not None:
                reasons = [
                    r.reason
                    for r in response.results
                    if not r.successful
                ]

            self.get_logger().error(
                f'Could not activate {label}: {reasons}'
            )
            return

        self.current_mode = mode
        self.initialized = True
        self.publish_mode()

        prefix = 'Initialized' if initialization else 'Switched'

        self.get_logger().info(
            f'{prefix} navigation mode -> '
            f'{label} ({mode}), speed={speed:.2f} m/s'
        )

        if self.pending_mode is not None:
            pending = self.pending_mode
            self.pending_mode = None

            if pending != self.current_mode:
                self.apply_speed_mode(pending)


def main(args=None):
    rclpy.init(args=args)

    node = NavigationModeManager()

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
