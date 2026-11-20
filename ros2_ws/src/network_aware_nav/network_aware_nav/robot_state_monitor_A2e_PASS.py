#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Float32, Float32MultiArray, UInt8
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from nav2_msgs.action._navigate_to_pose import (
    NavigateToPose_FeedbackMessage,
)

from naw_interfaces.msg import RobotState


class RobotStateMonitor(Node):

    def __init__(self):
        super().__init__('robot_state_monitor')

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('source_timeout', 2.0)
        self.declare_parameter('goal_feedback_timeout', 2.0)

        publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        self.source_timeout = float(
            self.get_parameter('source_timeout').value
        )

        self.goal_feedback_timeout = float(
            self.get_parameter('goal_feedback_timeout').value
        )

        # Latest values
        self.scan_age = None
        self.camera_age = None
        self.dist_obstacle = None
        self.dist_goal = None
        self.speed = None
        self.nav_status = None

        # Receipt timestamps
        self.scan_age_rx = None
        self.camera_age_rx = None
        self.scan_rx = None
        self.odom_rx = None
        self.nav_mode_rx = None
        self.goal_feedback_rx = None

        # /topic_age:
        # [0] latency
        # [1] loss
        # [2] degraded scan age
        # [3] command age
        self.create_subscription(
            Float32MultiArray,
            '/topic_age',
            self.topic_age_callback,
            10,
        )

        self.create_subscription(
            Float32,
            '/camera_age',
            self.camera_age_callback,
            10,
        )

        # IMPORTANT:
        # raw scan is used for physical/local obstacle distance.
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # /nav_mode is transient-local.
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            UInt8,
            '/nav_mode',
            self.nav_mode_callback,
            mode_qos,
        )

        self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.goal_feedback_callback,
            10,
        )

        self.publisher = self.create_publisher(
            RobotState,
            '/robot_state',
            10,
        )

        period = 1.0 / max(0.1, publish_rate)

        self.create_timer(
            period,
            self.publish_state,
        )

        self.last_not_ready_log = None

        self.get_logger().info(
            f'RobotStateMonitor started at {publish_rate:.2f} Hz'
        )

    def now(self):
        return self.get_clock().now()

    def topic_age_callback(self, msg):
        if len(msg.data) < 3:
            self.get_logger().warning(
                '/topic_age contains fewer than 3 elements'
            )
            return

        self.scan_age = max(
            0.0,
            float(msg.data[2])
        )

        self.scan_age_rx = self.now()

    def camera_age_callback(self, msg):
        self.camera_age = max(
            0.0,
            float(msg.data)
        )

        self.camera_age_rx = self.now()

    def scan_callback(self, msg):
        valid = []

        for r in msg.ranges:
            if not math.isfinite(r):
                continue

            if r < msg.range_min:
                continue

            if r > msg.range_max:
                continue

            valid.append(float(r))

        if valid:
            self.dist_obstacle = min(valid)
        else:
            # No obstacle detected within valid sensor range.
            # Use sensor maximum range rather than a fake zero.
            self.dist_obstacle = float(msg.range_max)

        self.scan_rx = self.now()

    def odom_callback(self, msg):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)

        self.speed = math.sqrt(
            vx * vx + vy * vy
        )

        self.odom_rx = self.now()

    def nav_mode_callback(self, msg):
        mode = int(msg.data)

        if mode not in (0, 1, 2):
            self.get_logger().warning(
                f'Invalid /nav_mode value: {mode}'
            )
            return

        self.nav_status = mode
        self.nav_mode_rx = self.now()

    def goal_feedback_callback(self, msg):
        distance = float(
            msg.feedback.distance_remaining
        )

        if not math.isfinite(distance):
            return

        self.dist_goal = max(
            0.0,
            distance
        )

        self.goal_feedback_rx = self.now()

    def age_of(self, timestamp):
        if timestamp is None:
            return float('inf')

        return (
            self.now().nanoseconds -
            timestamp.nanoseconds
        ) / 1e9

    def sources_ready(self):
        required_values = [
            self.scan_age,
            self.camera_age,
            self.dist_obstacle,
            self.dist_goal,
            self.speed,
            self.nav_status,
        ]

        if any(v is None for v in required_values):
            return False

        # General sensor/mode freshness.
        timestamps = [
            self.scan_age_rx,
            self.camera_age_rx,
            self.scan_rx,
            self.odom_rx,
            self.nav_mode_rx,
        ]

        for timestamp in timestamps:
            if self.age_of(timestamp) > self.source_timeout:
                return False

        # Goal feedback is intentionally treated separately.
        # We do not publish stale goal distance between episodes.
        if (
            self.age_of(self.goal_feedback_rx)
            > self.goal_feedback_timeout
        ):
            return False

        return True

    def publish_state(self):
        if not self.sources_ready():
            # Throttled informational warning, approximately every 5 s.
            now = self.now()

            if (
                self.last_not_ready_log is None
                or (
                    now.nanoseconds -
                    self.last_not_ready_log.nanoseconds
                ) / 1e9 > 5.0
            ):
                self.get_logger().info(
                    'RobotState waiting for fresh sensor/mode/'
                    'NavigateToPose feedback'
                )

                self.last_not_ready_log = now

            return

        msg = RobotState()

        msg.header.stamp = self.now().to_msg()
        msg.header.frame_id = 'robot_state_monitor'

        msg.scan_age = float(self.scan_age)
        msg.camera_age = float(self.camera_age)
        msg.dist_obstacle = float(self.dist_obstacle)
        msg.dist_goal = float(self.dist_goal)
        msg.speed = float(self.speed)
        msg.nav_status = int(self.nav_status)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RobotStateMonitor()

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
