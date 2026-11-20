#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger


class DynamicObstacle(Node):

    def __init__(self):
        super().__init__("dynamic_obstacle")

        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------
        self.declare_parameter("model_name", "dynamic_pedestrian")

        self.declare_parameter("cross_x", 0.45)
        self.declare_parameter("start_y", -0.20)
        self.declare_parameter("end_y", 0.20)

        self.declare_parameter("z", 0.30)
        self.declare_parameter("speed", 0.06)

        # Hold briefly after crossing before parking outside arena.
        self.declare_parameter("hold_time", 1.0)

        self.declare_parameter("park_x", 5.0)
        self.declare_parameter("park_y", 5.0)

        self.model_name = str(
            self.get_parameter("model_name").value
        )

        self.cross_x = float(
            self.get_parameter("cross_x").value
        )

        self.start_y = float(
            self.get_parameter("start_y").value
        )

        self.end_y = float(
            self.get_parameter("end_y").value
        )

        self.z = float(
            self.get_parameter("z").value
        )

        self.speed = abs(float(
            self.get_parameter("speed").value
        ))

        self.hold_time = float(
            self.get_parameter("hold_time").value
        )

        self.park_x = float(
            self.get_parameter("park_x").value
        )

        self.park_y = float(
            self.get_parameter("park_y").value
        )

        if self.speed <= 0.0:
            raise RuntimeError("speed must be > 0")

        self.direction = (
            1.0 if self.end_y >= self.start_y else -1.0
        )

        self.travel_distance = abs(
            self.end_y - self.start_y
        )

        self.travel_time = (
            self.travel_distance / self.speed
        )

        # -------------------------------------------------------------
        # Gazebo
        # -------------------------------------------------------------
        self.state_cli = self.create_client(
            SetEntityState,
            "/set_entity_state"
        )

        # -------------------------------------------------------------
        # External control services
        # -------------------------------------------------------------
        self.create_service(
            Trigger,
            "/dynamic_obstacle/start",
            self.start_callback
        )

        self.create_service(
            Trigger,
            "/dynamic_obstacle/reset",
            self.reset_callback
        )

        # State machine:
        # parked -> moving -> holding -> parked
        self.phase = "parked"
        self.phase_start_ns = None

        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

        self.get_logger().info(
            "Dynamic obstacle ready: "
            f"x={self.cross_x:.2f}, "
            f"y={self.start_y:.2f}->{self.end_y:.2f}, "
            f"speed={self.speed:.2f} m/s, "
            f"travel_time={self.travel_time:.2f} s"
        )

    # -----------------------------------------------------------------
    # Gazebo helper
    # -----------------------------------------------------------------
    def set_state(self, x, y, vy=0.0):

        if not self.state_cli.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                "/set_entity_state unavailable"
            )
            return False

        state = EntityState()

        state.name = self.model_name
        state.reference_frame = "world"

        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = self.z

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0

        state.twist.linear.x = 0.0
        state.twist.linear.y = float(vy)
        state.twist.linear.z = 0.0

        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = state

        self.state_cli.call_async(req)

        return True

    # -----------------------------------------------------------------
    # Reset
    # -----------------------------------------------------------------
    def reset_callback(self, request, response):

        del request

        ok = self.set_state(
            self.park_x,
            self.park_y,
            0.0
        )

        self.phase = "parked"
        self.phase_start_ns = None

        response.success = ok

        if ok:
            response.message = (
                "dynamic obstacle parked"
            )

            self.get_logger().info(
                "RESET -> obstacle parked"
            )
        else:
            response.message = (
                "failed to park obstacle"
            )

        return response

    # -----------------------------------------------------------------
    # Start crossing
    # -----------------------------------------------------------------
    def start_callback(self, request, response):

        del request

        if self.phase != "parked":
            response.success = False
            response.message = (
                f"obstacle currently {self.phase}"
            )
            return response

        velocity_y = (
            self.direction * self.speed
        )

        ok = self.set_state(
            self.cross_x,
            self.start_y,
            velocity_y
        )

        if not ok:
            response.success = False
            response.message = (
                "failed to start crossing"
            )
            return response

        self.phase = "moving"

        self.phase_start_ns = (
            self.get_clock().now().nanoseconds
        )

        response.success = True
        response.message = (
            "dynamic obstacle crossing started"
        )

        self.get_logger().info(
            "START crossing: "
            f"({self.cross_x:.2f},"
            f"{self.start_y:.2f}) -> "
            f"({self.cross_x:.2f},"
            f"{self.end_y:.2f}), "
            f"speed={self.speed:.2f} m/s"
        )

        return response

    # -----------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------
    def timer_callback(self):

        if self.phase == "parked":
            return

        now_ns = self.get_clock().now().nanoseconds

        if self.phase_start_ns is None:
            return

        elapsed = (
            now_ns - self.phase_start_ns
        ) / 1e9

        # -------------------------------------------------------------
        # Moving -> Holding
        # -------------------------------------------------------------
        if self.phase == "moving":

            if elapsed >= self.travel_time:

                # Snap exactly to final crossing position and stop.
                self.set_state(
                    self.cross_x,
                    self.end_y,
                    0.0
                )

                self.phase = "holding"
                self.phase_start_ns = now_ns

                self.get_logger().info(
                    "CROSSING COMPLETE -> "
                    f"holding at "
                    f"({self.cross_x:.2f},"
                    f"{self.end_y:.2f})"
                )

        # -------------------------------------------------------------
        # Holding -> Parked
        # -------------------------------------------------------------
        elif self.phase == "holding":

            if elapsed >= self.hold_time:

                self.set_state(
                    self.park_x,
                    self.park_y,
                    0.0
                )

                self.phase = "parked"
                self.phase_start_ns = None

                self.get_logger().info(
                    "obstacle parked after crossing"
                )


def main(args=None):

    rclpy.init(args=args)

    node = DynamicObstacle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
