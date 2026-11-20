#!/usr/bin/env python3

import math
import subprocess
import time
from pathlib import Path
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile,
    ReliabilityPolicy, DurabilityPolicy,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Bool, Float64, Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv, Trigger

from gazebo_msgs.msg import ContactsState, EntityState
from gazebo_msgs.srv import SetEntityState

from nav2_msgs.action import NavigateToPose
from unique_identifier_msgs.msg import UUID
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener, TransformException

from naw_interfaces.msg import StateVector, RobotState

from network_aware_nav.reward_calculator import (
    RewardConfig,
    calculate_reward,
)

from network_aware_nav.spatial_adapter import (
    SpatialAdapter,
)


# --------------------------------------------------------------------------
# B2 environment constants
# --------------------------------------------------------------------------

ROBOT_NAME = "burger"

RL_START = (-0.80, -0.82, 0.0)

# --------------------------------------------------------------------------
# B2 arena centerline route
#
# Patched arena free-space centerlines:
#   bottom corridor : y = -0.8225
#   middle corridor : y =  0.0000
#   top corridor    : y = +0.8225
#   right opening   : x = +0.9550
#   left opening    : x = -0.9550
#
# Each 90-degree corner is split into:
#   ENTRY -> TURN
#
# ENTRY preserves the incoming heading while reaching the corridor
# centerline. TURN uses the same x/y and changes only orientation.
# Therefore ENTRY->TURN has zero geometric route length and does not
# create artificial translational progress.
# --------------------------------------------------------------------------

WAYPOINTS = [
    # Bottom corridor -> right opening.
    ("P1_ENTRY",  0.9550, -0.8225, 0.0000),
    ("P1",        0.9550, -0.8225, 1.5708),

    # Right opening -> middle corridor.
    ("P2_ENTRY",  0.9550,  0.0000, 1.5708),
    ("P2",        0.9550,  0.0000, 3.1416),

    # Middle corridor -> left opening.
    ("P3_ENTRY", -0.9550,  0.0000, 3.1416),
    ("P3",       -0.9550,  0.0000, 1.5708),

    # Left opening -> top corridor.
    ("P4_ENTRY", -0.9550,  0.8225, 1.5708),
    ("P4",       -0.9550,  0.8225, 0.0000),

    # Final top-corridor goal.
    ("G",         0.9000,  0.8225, 0.0000),
]

DECISION_PERIOD = 0.5
MAX_STEPS = 360
EPISODE_TIMEOUT = 180.0

# For translational waypoint changes, >= 0.30 m distinguishes fresh
# feedback for the newly dispatched Nav2 goal from the previous goal's
# residual distance.
#
# ENTRY->TURN waypoints intentionally have zero translational distance.
# While such a turn is active, route progress is therefore held at the
# previous value. Once the following translational waypoint is dispatched,
# its larger distance clears this feedback hold.
NEW_GOAL_FEEDBACK_MIN = 0.30

SETTLE_TIME = 4.0


class NetworkAwareEnv:
    """
    ROS2 environment for network-aware high-level navigation control.

    RL action:
        0 NORMAL    Q0
        1 NORMAL    Q1
        2 NORMAL    Q2
        3 SLOW      Q0
        4 SLOW      Q1
        5 SLOW      Q2
        6 SAFE_STOP Q0
        7 SAFE_STOP Q1
        8 SAFE_STOP Q2

    Returned API follows Gymnasium convention:

        obs, info = reset(...)
        obs, reward, terminated, truncated, info = step(action)

    This class intentionally does not depend on gymnasium yet.
    DDQN training can call reset()/step() directly.
    """

    def __init__(
        self,
        use_dynamic_obstacle: bool = True,
        goal_reward: float = 20.0,
        collision_penalty: float = -25.0,
        truncation_penalty: float = -10.0,
        environment_name: Optional[str] = None,
        spatial_root: Optional[str] = None,
    ):
        self._owns_rclpy = False

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self.node = Node(
            "network_aware_env",
            parameter_overrides=[
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    True,
                )
            ],
            automatically_declare_parameters_from_overrides=True,
        )

        self.log = self.node.get_logger()

        # --------------------------------------------------------------
        # Provisional B2 terminal rewards.
        # B1c non-terminal reward remains frozen in reward_calculator.py.
        # --------------------------------------------------------------

        self.goal_reward = float(goal_reward)
        self.collision_penalty = float(collision_penalty)
        self.truncation_penalty = float(truncation_penalty)

        self.reward_config = RewardConfig()

        self.use_dynamic_obstacle = bool(use_dynamic_obstacle)

        # --------------------------------------------------------------
        # B2 spatial configuration
        # --------------------------------------------------------------

        self.environment_name = environment_name
        self.spatial_adapter = None
        self.last_progress_snap = 0.0

        if environment_name is None:
            # Legacy arena-compatible mode.
            self.rl_start = tuple(RL_START)
            self.waypoints = list(WAYPOINTS)

        else:
            if self.use_dynamic_obstacle:
                raise ValueError(
                    "Map-aware dynamic crossing is not integrated yet. "
                    "Use use_dynamic_obstacle=False until the crossing "
                    "adapter is installed."
                )

            if spatial_root is None:
                spatial_root = str(
                    Path.home()
                    / "ros2_ws"
                    / "results"
                    / "B2_SPATIAL_FINAL"
                    / "generated_envs"
                )

            self.spatial_adapter = SpatialAdapter(
                spatial_root,
                environment_name,
            )

            sx, sy, syaw = (
                self.spatial_adapter.start
            )

            gx, gy, gyaw = (
                self.spatial_adapter.goal
            )

            self.rl_start = (
                float(sx),
                float(sy),
                float(syaw),
            )

            # Sparse Nav2 route extracted automatically from the
            # 0.45 m safety-feasible reference path. Consecutive
            # waypoints have direct safety-free line-of-sight.
            self.waypoints = (
                self.spatial_adapter
                .navigation_waypoints()
            )

            self.log.info(
                "B2 spatial environment loaded: "
                f"{environment_name}; "
                f"start=({sx:.2f},{sy:.2f}); "
                f"goal=({gx:.2f},{gy:.2f}); "
                f"geodesic="
                f"{self.spatial_adapter.initial_progress_distance:.2f} m"
            )

        # --------------------------------------------------------------
        # Runtime data
        # --------------------------------------------------------------

        self.latest_state: Optional[StateVector] = None
        self.state_seq = 0

        self.latest_robot: Optional[RobotState] = None
        self.robot_seq = 0

        self.latest_odom_pose = None
        self.odom_seq = 0

        self.latest_action_executed = None

        self.override_since_step = False

        self.latest_contact = False
        self.contact_since_step = False
        self.contact_episode = False

        self.physics_paused = False

        # --------------------------------------------------------------
        # Route state
        # --------------------------------------------------------------

        self.waypoint_index = 0

        self.goal_send_future = None
        self.goal_handle = None
        self.goal_result_future = None
        self.active_goal_id = None
        self.active_goal_feedback = None
        self.active_goal_feedback_seq = 0
        self._p1_feedback_rejected = False

        self.route_success = False
        self.nav_failure_reason = None

        self.waypoint_transition_this_step = False
        self.transition_waiting_feedback = False

        self.initial_route_distance = None
        self.previous_route_remaining = None

        self.previous_executed_action = None

        self.step_count = 0
        self.episode_start_sim_ns = None
        self.episode_active = False

        self.current_network_profile = "normal"

        # --------------------------------------------------------------
        # Route geometry
        # --------------------------------------------------------------

        self.segment_lengths = self._compute_segment_lengths()
        self.future_tail = self._compute_future_tails()

        self.geometric_route_length = sum(self.segment_lengths)

        self.log.info(
            "B2 route geometric length: "
            f"{self.geometric_route_length:.3f} m"
        )

        # --------------------------------------------------------------
        # Publishers
        # --------------------------------------------------------------

        self.action_pub = self.node.create_publisher(
            UInt8,
            "/action_requested",
            10,
        )

        self.episode_reset_pub = self.node.create_publisher(
            EmptyMsg,
            "/episode_reset",
            10,
        )

        # Active-goal identity, latched so RobotStateMonitor can
        # reject feedback from previous waypoints/episodes.
        goal_scope_qos = QoSProfile(depth=1)
        goal_scope_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_scope_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_scope_pub = self.node.create_publisher(
            UUID, "/active_nav_goal_id", goal_scope_qos,
        )

        # Verified P1 distance: StateAggregator must not infer dg0
        # from an arbitrary fresh RobotState message.
        self.verified_initial_goal_pub = self.node.create_publisher(
            Float64, "/verified_initial_goal_distance", 10,
        )

        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        # --------------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------------

        self.state_sub = self.node.create_subscription(
            StateVector,
            "/state_vector",
            self._state_cb,
            10,
        )

        self.robot_sub = self.node.create_subscription(
            RobotState,
            "/robot_state",
            self._robot_cb,
            10,
        )

        self.action_executed_sub = self.node.create_subscription(
            UInt8,
            "/action_executed",
            self._action_executed_cb,
            10,
        )

        self.override_sub = self.node.create_subscription(
            Bool,
            "/safety_override",
            self._override_cb,
            10,
        )

        self.contact_sub = self.node.create_subscription(
            ContactsState,
            "/robot/contacts",
            self._contact_cb,
            10,
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/odom",
            self._odom_cb,
            qos_profile_sensor_data,
        )

        # --------------------------------------------------------------
        # TF
        # --------------------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

        # --------------------------------------------------------------
        # Services
        # --------------------------------------------------------------

        self.set_entity_cli = self.node.create_client(
            SetEntityState,
            "/set_entity_state",
        )

        self.pause_cli = self.node.create_client(
            EmptySrv,
            "/pause_physics",
        )

        self.unpause_cli = self.node.create_client(
            EmptySrv,
            "/unpause_physics",
        )

        self.clear_local_cli = self.node.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )

        self.clear_global_cli = self.node.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )

        self.obstacle_reset_cli = self.node.create_client(
            Trigger,
            "/dynamic_obstacle/reset",
        )

        self.obstacle_start_cli = self.node.create_client(
            Trigger,
            "/dynamic_obstacle/start",
        )

        # --------------------------------------------------------------
        # Nav2
        # --------------------------------------------------------------

        self.nav = ActionClient(
            self.node,
            NavigateToPose,
            "/navigate_to_pose",
        )

    # ======================================================================
    # ROS callbacks
    # ======================================================================

    def _state_cb(self, msg):
        self.latest_state = msg
        self.state_seq += 1

    def _robot_cb(self, msg):
        self.latest_robot = msg
        self.robot_seq += 1

    def _action_executed_cb(self, msg):
        self.latest_action_executed = int(msg.data)

    def _override_cb(self, msg):
        if bool(msg.data):
            self.override_since_step = True

    def _contact_cb(self, msg):
        contact = len(msg.states) > 0

        self.latest_contact = contact

        if contact:
            self.contact_since_step = True
            self.contact_episode = True

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.latest_odom_pose = (
            float(p.x),
            float(p.y),
            float(yaw),
        )

        self.odom_seq += 1

    # ======================================================================
    # Basic helpers
    # ======================================================================

    def _spin_once(self, timeout=0.05):
        rclpy.spin_once(
            self.node,
            timeout_sec=float(timeout),
        )

    def _spin_for_wall(self, seconds):
        deadline = time.monotonic() + float(seconds)

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            self._spin_once(0.05)

    def _wait_future(self, future, timeout=5.0):
        deadline = time.monotonic() + float(timeout)

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            self._spin_once(0.05)

        if not future.done():
            return None

        return future.result()

    @staticmethod
    def _stamp_ns(msg):
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    # ======================================================================
    # Gazebo pause / unpause
    # ======================================================================

    def _pause(self):
        if not self.pause_cli.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "/pause_physics unavailable"
            )

        fut = self.pause_cli.call_async(
            EmptySrv.Request()
        )

        if self._wait_future(fut, 5.0) is None:
            raise RuntimeError(
                "pause_physics failed"
            )

        self.physics_paused = True

    def _unpause(self):
        if not self.unpause_cli.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "/unpause_physics unavailable"
            )

        fut = self.unpause_cli.call_async(
            EmptySrv.Request()
        )

        if self._wait_future(fut, 5.0) is None:
            raise RuntimeError(
                "unpause_physics failed"
            )

        self.physics_paused = False

    # ======================================================================
    # Network profile
    # ======================================================================

    def _set_network_profile(self, profile):
        profile = str(profile).lower()

        valid = {
            "normal",
            "mild",
            "moderate",
            "severe",
        }

        if profile not in valid:
            raise ValueError(
                f"invalid network profile: {profile}"
            )

        result = subprocess.run(
            [
                "ros2",
                "param",
                "set",
                "/network_emulator",
                "profile",
                profile,
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        if result.returncode != 0:
            raise RuntimeError(
                "Could not set network profile "
                f"{profile}: {result.stderr.strip()}"
            )

        self.current_network_profile = profile

    # ======================================================================
    # Dynamic obstacle
    # ======================================================================

    def _call_obstacle(self, command, required=True):
        if command == "reset":
            cli = self.obstacle_reset_cli
        elif command == "start":
            cli = self.obstacle_start_cli
        else:
            raise ValueError(command)

        if not cli.wait_for_service(timeout_sec=2.0):
            if required:
                raise RuntimeError(
                    "dynamic obstacle service unavailable: "
                    f"{command}"
                )

            self.log.warning(
                "dynamic obstacle service unavailable: "
                f"{command}"
            )

            return False

        fut = cli.call_async(
            Trigger.Request()
        )

        result = self._wait_future(
            fut,
            timeout=5.0,
        )

        if result is None:
            if required:
                raise RuntimeError(
                    f"dynamic obstacle {command} timeout"
                )
            return False

        if not result.success:
            if required:
                raise RuntimeError(
                    f"dynamic obstacle {command} failed: "
                    f"{result.message}"
                )
            return False

        self.log.info(
            f"DYNAMIC OBSTACLE {command.upper()}: "
            f"{result.message}"
        )

        return True

    # ======================================================================
    # Costmap / reset helpers
    # ======================================================================

    def _clear_costmap(self, cli, name):
        if not cli.wait_for_service(
            timeout_sec=3.0
        ):
            self.log.warning(
                f"{name} unavailable"
            )
            return False

        fut = cli.call_async(
            ClearEntireCostmap.Request()
        )

        result = self._wait_future(
            fut,
            timeout=5.0,
        )

        if result is None:
            self.log.warning(
                f"{name} did not respond"
            )
            return False

        return True

    def _wait_for_odom_stable(
        self,
        timeout=8.0,
        stable_samples=8,
        position_step_tolerance=0.01,
        yaw_step_tolerance=0.02,
    ):
        deadline = time.monotonic() + timeout
        start_seq = self.odom_seq

        previous = None
        stable = 0

        while (
            time.monotonic() < deadline
            and rclpy.ok()
        ):
            self._spin_once(0.10)

            if self.odom_seq <= start_seq:
                continue

            current = self.latest_odom_pose

            if current is None:
                continue

            if previous is None:
                previous = current
                continue

            dx = current[0] - previous[0]
            dy = current[1] - previous[1]

            dyaw = abs(
                math.atan2(
                    math.sin(
                        current[2] - previous[2]
                    ),
                    math.cos(
                        current[2] - previous[2]
                    ),
                )
            )

            step = math.hypot(dx, dy)

            if (
                step <= position_step_tolerance
                and dyaw <= yaw_step_tolerance
            ):
                stable += 1
            else:
                stable = 0

            previous = current

            if stable >= stable_samples:
                self.log.info(
                    "ODOM STABLE: "
                    f"({current[0]:.3f}, "
                    f"{current[1]:.3f}), "
                    f"yaw="
                    f"{math.degrees(current[2]):.1f} deg"
                )

                return True

        return False

    def _wait_for_start_ready(
        self,
        x,
        y,
        yaw,
        timeout=20.0,
        position_tolerance=0.15,
        yaw_tolerance=0.35,
        max_tf_age=0.50,
        stable_samples=5,
        min_tf_stamp_sec=None,
    ):
        """
        Verify reset localization without requiring AMCL to continuously
        republish a transform while the robot is stationary.

        For an episode reset we require:
          1. map->base_footprint TF generated after the reset marker;
          2. pose within reset tolerances;
          3. simulation clock is still advancing;
          4. several stable observations.

        Legacy callers without min_tf_stamp_sec retain the old tf-age check.
        """

        deadline = time.monotonic() + timeout
        stable = 0
        last_reason = "no TF"
        last_clock_sec = None

        while (
            time.monotonic() < deadline
            and rclpy.ok()
        ):
            self._spin_once(0.10)

            try:
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    "base_footprint",
                    Time(),
                )
            except TransformException as exc:
                stable = 0
                last_reason = str(exc)
                continue

            px = float(
                tf.transform.translation.x
            )
            py = float(
                tf.transform.translation.y
            )

            q = tf.transform.rotation

            current_yaw = math.atan2(
                2.0 * (
                    q.w * q.z
                    + q.x * q.y
                ),
                1.0 - 2.0 * (
                    q.y * q.y
                    + q.z * q.z
                ),
            )

            position_error = math.hypot(
                px - x,
                py - y,
            )

            yaw_error = abs(
                math.atan2(
                    math.sin(
                        current_yaw - yaw
                    ),
                    math.cos(
                        current_yaw - yaw
                    ),
                )
            )

            stamp_sec = (
                float(tf.header.stamp.sec)
                + float(
                    tf.header.stamp.nanosec
                ) * 1e-9
            )

            now_sec = (
                self.node.get_clock()
                .now()
                .nanoseconds
                / 1e9
            )

            tf_age = now_sec - stamp_sec

            # A frozen Gazebo clock must never satisfy reset readiness.
            clock_advanced = (
                last_clock_sec is None
                or now_sec > last_clock_sec + 1e-9
            )

            if not clock_advanced:
                last_reason = (
                    "simulation clock not advancing; "
                    f"now={now_sec:.3f}"
                )
                continue

            last_clock_sec = now_sec

            position_ok = (
                position_error
                <= position_tolerance
            )

            yaw_ok = (
                yaw_error
                <= yaw_tolerance
            )

            if min_tf_stamp_sec is None:
                # Preserve legacy behaviour for non-reset callers.
                freshness_ok = (
                    -0.20
                    <= tf_age
                    <= max_tf_age
                )
            else:
                # Episode reset:
                # require a TF generated after the reset marker.
                freshness_ok = (
                    tf_age >= -0.20
                    and stamp_sec + 1e-6
                    >= min_tf_stamp_sec
                )

            good = (
                position_ok
                and yaw_ok
                and freshness_ok
            )

            if good:
                stable += 1

                if stable >= stable_samples:
                    self.log.info(
                        "RESET READY: "
                        f"pose=({px:.3f},"
                        f"{py:.3f}), "
                        f"pos_err="
                        f"{position_error:.3f} m, "
                        f"yaw_err="
                        f"{math.degrees(yaw_error):.1f} deg, "
                        f"tf_age={tf_age:.3f} s"
                    )

                    return True

            else:
                stable = 0

                last_reason = (
                    f"pos_err="
                    f"{position_error:.3f}, "
                    f"yaw_err="
                    f"{math.degrees(yaw_error):.1f}, "
                    f"tf_age={tf_age:.3f}, "
                    f"tf_stamp={stamp_sec:.3f}, "
                    f"reset_stamp="
                    f"{min_tf_stamp_sec}"
                )

        self.log.error(
            "RESET NOT READY: "
            f"{last_reason}"
        )

        return False

    def _teleport(
        self,
        x,
        y,
        yaw,
    ):
        """
        Deterministic episode reset.

        Gazebo physics is paused only around SetEntityState.  It is
        immediately resumed before waiting for odometry/AMCL.
        """

        if not self.set_entity_cli.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "/set_entity_state unavailable"
            )

        st = EntityState()
        st.name = ROBOT_NAME
        st.reference_frame = "world"

        st.pose.position.x = float(x)
        st.pose.position.y = float(y)
        st.pose.position.z = 0.01

        st.pose.orientation.x = 0.0
        st.pose.orientation.y = 0.0
        st.pose.orientation.z = math.sin(
            yaw / 2.0
        )
        st.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        st.twist.linear.x = 0.0
        st.twist.linear.y = 0.0
        st.twist.linear.z = 0.0

        st.twist.angular.x = 0.0
        st.twist.angular.y = 0.0
        st.twist.angular.z = 0.0

        # ----------------------------------------------------------
        # Critical reset change:
        # do not teleport an actively simulated differential-drive
        # model while physics is integrating.
        # ----------------------------------------------------------

        self._pause()

        try:
            req = SetEntityState.Request()
            req.state = st

            fut = self.set_entity_cli.call_async(
                req
            )

            rclpy.spin_until_future_complete(
                self.node,
                fut,
                timeout_sec=5.0,
            )

            result = fut.result()

            if result is None:
                raise RuntimeError(
                    "Gazebo SetEntityState returned "
                    "no response"
                )

            if not result.success:
                raise RuntimeError(
                    "Gazebo SetEntityState reported "
                    "failure"
                )

        finally:
            # Physics must always resume after the atomic teleport.
            self._unpause()

        # Give Gazebo plugins a short opportunity to publish from
        # the new physical state.
        self._spin_for_wall(0.25)

        if not self._wait_for_odom_stable():
            raise RuntimeError(
                "Odometry did not settle after "
                "Gazebo teleport"
            )

        # Marker used to reject TF left over from the previous
        # episode.
        reset_tf_not_before = (
            self.node.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)

        msg.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )
        msg.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03

        for _ in range(3):
            msg.header.stamp = (
                self.node.get_clock()
                .now()
                .to_msg()
            )

            self.initial_pose_pub.publish(msg)

            self._spin_for_wall(0.20)

        self._spin_for_wall(SETTLE_TIME)

        # ----------------------------------------------------------
        # Clear Nav2 costmaps through the existing ROS clients.
        # Do not spawn nested `ros2 service call` CLI processes
        # during an episode reset.
        # ----------------------------------------------------------

        for label, cli in (
            ("local", self.clear_local_cli),
            ("global", self.clear_global_cli),
        ):
            if not cli.wait_for_service(
                timeout_sec=2.0
            ):
                self.log.warn(
                    f"{label} costmap clear "
                    "service unavailable"
                )
                continue

            fut = cli.call_async(
                ClearEntireCostmap.Request()
            )

            rclpy.spin_until_future_complete(
                self.node,
                fut,
                timeout_sec=3.0,
            )

            if fut.result() is None:
                self.log.warn(
                    f"{label} costmap clear "
                    "did not return"
                )

        self._spin_for_wall(1.0)

        if not self._wait_for_start_ready(
            x,
            y,
            yaw,
            min_tf_stamp_sec=reset_tf_not_before,
        ):
            raise RuntimeError(
                "Localization failed to stabilize "
                "at self.rl_start"
            )

    def _compute_segment_lengths(self):
        points = [
            (
                float(self.rl_start[0]),
                float(self.rl_start[1]),
            )
        ]

        points += [
            (
                float(wp[1]),
                float(wp[2]),
            )
            for wp in self.waypoints
        ]

        lengths = []

        for i in range(
            len(points) - 1
        ):
            lengths.append(
                math.dist(
                    points[i],
                    points[i + 1],
                )
            )

        return lengths

    def _compute_future_tails(self):
        """
        For active waypoint i:
            route_remaining =
                Nav2 distance_remaining to waypoint i
                + future_tail[i]
        """

        tails = []

        for i in range(len(self.waypoints)):
            # segment_lengths[i] is the segment
            # ending at current waypoint i.
            future = sum(
                self.segment_lengths[
                    i + 1:
                ]
            )

            tails.append(float(future))

        return tails

    # ======================================================================
    # Nav2 goal management
    # ======================================================================

    def _publish_goal_scope(self, goal_id=None):
        msg = UUID()
        if goal_id is not None:
            msg.uuid = [int(x) for x in goal_id]
        self.goal_scope_pub.publish(msg)

    def _goal_feedback_cb(self, feedback_msg):
        # ActionClient callback carries goal_id, unlike RobotState.
        if self.active_goal_id is None or self.goal_handle is None:
            return
        if bytes(feedback_msg.goal_id.uuid) != self.active_goal_id:
            return
        distance = float(feedback_msg.feedback.distance_remaining)
        if not math.isfinite(distance) or distance < 0.0:
            return
        self.active_goal_feedback = distance
        self.active_goal_feedback_seq += 1

    def _dispatch_goal(self, index):
        if not (
            0 <= index < len(self.waypoints)
        ):
            raise IndexError(index)

        name, gx, gy, gyaw = (
            self.waypoints[index]
        )

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = (
            self.node
            .get_clock()
            .now()
            .to_msg()
        )

        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = math.sin(
            gyaw / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            gyaw / 2.0
        )

        self.log.info(
            f"Dispatch {name}: "
            f"({gx:.2f}, {gy:.2f})"
        )

        # Invalidate the old identity before dispatching a new goal.
        self.active_goal_id = None
        self.active_goal_feedback = None
        self.active_goal_feedback_seq = 0
        self._p1_feedback_rejected = False
        self._publish_goal_scope()

        self.goal_handle = None
        self.goal_result_future = None

        self.goal_send_future = self.nav.send_goal_async(
            goal, feedback_callback=self._goal_feedback_cb,
        )

    def _poll_goal_manager(self):
        # Goal request not accepted/rejected yet.
        if (
            self.goal_send_future is not None
            and self.goal_handle is None
        ):
            if self.goal_send_future.done():
                handle = (
                    self.goal_send_future.result()
                )

                if (
                    handle is None
                    or not handle.accepted
                ):
                    name = self.waypoints[
                        self.waypoint_index
                    ][0]

                    self.nav_failure_reason = (
                        f"goal_rejected_{name}"
                    )
                    return

                self.goal_handle = handle
                self.active_goal_id = bytes(handle.goal_id.uuid)
                if len(self.active_goal_id) != 16 or not any(self.active_goal_id):
                    raise RuntimeError("Nav2 accepted goal without valid UUID")
                self._publish_goal_scope(self.active_goal_id)

                self.goal_result_future = (
                    handle.get_result_async()
                )

        if self.goal_result_future is None:
            return

        if not self.goal_result_future.done():
            return

        wrapped = (
            self.goal_result_future.result()
        )

        if wrapped is None:
            name = self.waypoints[
                self.waypoint_index
            ][0]

            self.nav_failure_reason = (
                f"goal_no_result_{name}"
            )
            return

        status = wrapped.status

        name = self.waypoints[
            self.waypoint_index
        ][0]

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            self.log.info(
                f"{name} reached"
            )

            # Final goal.
            if (
                self.waypoint_index
                == len(self.waypoints) - 1
            ):
                self.route_success = True
                return

            # Deterministic dynamic crossing:
            # trigger after P2 and before P3.
            if (
                name == "P2"
                and self.use_dynamic_obstacle
            ):
                self._call_obstacle(
                    "start",
                    required=True,
                )

            self.waypoint_index += 1

            self.waypoint_transition_this_step = True
            self.transition_waiting_feedback = True

            self._dispatch_goal(
                self.waypoint_index
            )

            return

        if status == GoalStatus.STATUS_ABORTED:
            self.nav_failure_reason = (
                f"nav2_aborted_{name}"
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.nav_failure_reason = (
                f"nav2_canceled_{name}"
            )

        else:
            self.nav_failure_reason = (
                f"nav2_status_{status}_{name}"
            )

    def _wait_goal_accepted(
        self,
        timeout=15.0,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            self._spin_once(0.05)

            self._poll_goal_manager()

            if self.nav_failure_reason:
                raise RuntimeError(
                    self.nav_failure_reason
                )

            if self.goal_handle is not None:
                return True

        raise RuntimeError(
            "Nav2 goal acceptance timeout"
        )

    def _cancel_active_goal(self):
        if self.goal_handle is None:
            return

        if (
            self.goal_result_future is not None
            and self.goal_result_future.done()
        ):
            return

        try:
            fut = (
                self.goal_handle
                .cancel_goal_async()
            )

            self._wait_future(
                fut,
                timeout=3.0,
            )

        except Exception as exc:
            self.log.warning(
                f"goal cancel failed: {exc}"
            )

    # ======================================================================
    # State / progress helpers
    # ======================================================================

    @staticmethod
    def _finite_state(msg):
        if msg is None:
            return False

        if (
            len(msg.state) != 10
            or len(msg.raw_state) != 10
        ):
            return False

        values = (
            list(msg.state)
            + list(msg.raw_state)
            + [
                msg.initial_goal_distance
            ]
        )

        return all(
            math.isfinite(float(v))
            for v in values
        )

    def _wait_robot_goal_feedback(self, _start_seq, timeout=10.0):
        """Wait for *this* accepted Nav2 goal's feedback, not RobotState age."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)
            if (self.goal_result_future is not None
                    and self.goal_result_future.done()):
                wrapped = self.goal_result_future.result()
                status = wrapped.status if wrapped is not None else -1
                raise RuntimeError(
                    "Nav2 goal ended before goal-scoped feedback; "
                    f"status={status}"
                )

            dg = self.active_goal_feedback
            if dg is None or not math.isfinite(dg) or dg <= 1e-3:
                continue

            # A UUID match is necessary, but validate initial P1 against
            # the physical pose as well (reject e.g. 0.035 m at 1.08 m).
            if self.waypoint_index == 0:
                x, y = self._map_xy()
                _, gx, gy, _ = self.waypoints[0]
                euclid = math.hypot(float(gx) - x, float(gy) - y)
                if euclid >= 0.3 and dg < 0.60 * euclid:
                    if not self._p1_feedback_rejected:
                        self.log.warning(
                            "T6F rejected implausible goal-scoped P1 feedback: "
                            f"Nav2={dg:.3f} m, Euclidean={euclid:.3f} m"
                        )
                        self._p1_feedback_rejected = True
                    continue
            return dg

        raise RuntimeError(
            "No UUID-matched, geometrically plausible Nav2 P1 feedback "
            "after goal dispatch"
        )

    def _wait_state_ready(
        self,
        start_seq,
        timeout=10.0,
        expected_initial_goal=None,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        last_verified_pub = float('-inf')
        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            # Retry the post-reset value: reset and initial-data messages
            # use different ROS topics (no cross-topic delivery ordering).
            if expected_initial_goal is not None:
                wall_now = time.monotonic()
                if wall_now - last_verified_pub >= 0.20:
                    msg = Float64()
                    msg.data = float(expected_initial_goal)
                    self.verified_initial_goal_pub.publish(msg)
                    last_verified_pub = wall_now
            self._spin_once(0.05)

            if self.state_seq <= start_seq:
                continue

            if not self._finite_state(self.latest_state):
                continue
            if expected_initial_goal is not None:
                dg0 = float(self.latest_state.initial_goal_distance)
                if not math.isclose(
                    dg0, float(expected_initial_goal),
                    rel_tol=1e-5, abs_tol=1e-5,
                ):
                    continue
                # RobotState also needs plausible current P1 feedback;
                # rejected old-goal residuals cannot start a fresh episode.
                if self.latest_robot is None:
                    continue
                current_dg = float(self.latest_robot.dist_goal)
                if not math.isfinite(current_dg):
                    continue
                x, y = self._map_xy()
                _, gx, gy, _ = self.waypoints[0]
                dxy = math.hypot(float(gx)-x, float(gy)-y)
                if dxy >= 0.3 and current_dg < 0.60*dxy:
                    continue
            return True

        raise RuntimeError(
            "No fresh finite /state_vector"
        )

    def _current_executed_action(self):
        if self.latest_state is None:
            return None

        raw = self.latest_state.raw_state

        nav_mode = int(
            round(float(raw[8]))
        )

        qos_mode = int(
            round(float(raw[9]))
        )

        if nav_mode not in (0, 1, 2):
            return None

        if qos_mode not in (0, 1, 2):
            return None

        return (
            nav_mode * 3
            + qos_mode
        )

    def _map_xy(self):
        """
        Current robot position for B2 route-progress evaluation.

        Prefer the Nav2 map-frame TF.  Immediately after a Gazebo
        teleport, however, AMCL map->odom and Gazebo odom->base may
        temporarily have non-overlapping timestamp histories.  In the
        generated B2 simulation worlds, Gazebo odometry/world
        coordinates use the same spatial coordinates as the generated
        map metadata, so odometry is a deterministic fallback for this
        transient TF condition.

        Hardware evaluation continues to use the TF path whenever it is
        available.
        """

        tf_error = None

        if hasattr(self, "tf_buffer"):
            try:
                transform = (
                    self.tf_buffer.lookup_transform(
                        "map",
                        "base_footprint",
                        Time(),
                    )
                )

                self._progress_pose_source = "map_tf"

                return (
                    float(
                        transform.transform.translation.x
                    ),
                    float(
                        transform.transform.translation.y
                    ),
                )

            except TransformException as exc:
                tf_error = exc

        # ----------------------------------------------------------
        # B2 Gazebo fallback
        # ----------------------------------------------------------

        pose = self.latest_odom_pose

        if pose is not None:

            x = None
            y = None

            # Current environment normally stores odometry as a tuple.
            if isinstance(
                pose,
                (tuple, list, np.ndarray),
            ):
                if len(pose) >= 2:
                    x = float(pose[0])
                    y = float(pose[1])

            # Also support Pose / Odometry-like representations so the
            # fallback remains robust if the callback representation
            # changes later.
            elif hasattr(pose, "position"):
                x = float(pose.position.x)
                y = float(pose.position.y)

            elif hasattr(pose, "pose"):
                obj = pose.pose

                if hasattr(obj, "pose"):
                    obj = obj.pose

                if hasattr(obj, "position"):
                    x = float(obj.position.x)
                    y = float(obj.position.y)

            if (
                x is not None
                and y is not None
                and math.isfinite(x)
                and math.isfinite(y)
            ):
                self._progress_pose_source = (
                    "gazebo_odom_fallback"
                )

                if not getattr(
                    self,
                    "_odom_progress_fallback_warned",
                    False,
                ):
                    self.log.warn(
                        "B2 progress: map TF temporarily "
                        "unavailable after reset; using "
                        "Gazebo odometry in the aligned "
                        "generated-map coordinate system. "
                        f"TF error: {tf_error}"
                    )

                    self._odom_progress_fallback_warned = True

                return x, y

        raise RuntimeError(
            "No valid pose available for B2 route progress; "
            f"map TF error: {tf_error}"
        )

    def _route_remaining(self):
        if self.route_success:
            return 0.0

        # ----------------------------------------------------------
        # B2 map-aware mode:
        #
        # Progress is measured continuously on the currently active
        # sparse Nav2 route leg. The sparse route itself is derived
        # from the validated 0.45 m safety-feasible reference path.
        #
        # Nav2 distance_remaining is never used for B2 reward.
        # ----------------------------------------------------------
        if self.spatial_adapter is not None:

            x, y = self._map_xy()

            (
                remaining,
                lateral_error,
                leg_fraction,
            ) = (
                self.spatial_adapter
                .active_sparse_leg_distance(
                    x,
                    y,
                    self.waypoint_index,
                )
            )

            # Historical field name retained for compatibility.
            # In B2 map-aware mode this stores lateral error to the
            # active sparse leg, not discrete grid snap distance.
            self.last_progress_snap = float(
                lateral_error
            )

            return max(
                0.0,
                float(remaining),
            )

        # ----------------------------------------------------------
        # Legacy arena behavior retained only for backward
        # compatibility / comparison.
        # ----------------------------------------------------------
        if self.latest_state is None:
            raise RuntimeError(
                "No state for route progress"
            )

        raw_distance = float(
            self.latest_state.raw_state[7]
        )

        if not math.isfinite(raw_distance):
            raise RuntimeError(
                "dist_goal is not finite"
            )

        if self.transition_waiting_feedback:
            if (
                raw_distance
                >= NEW_GOAL_FEEDBACK_MIN
            ):
                self.transition_waiting_feedback = False

            elif (
                self.previous_route_remaining
                is not None
            ):
                return float(
                    self.previous_route_remaining
                )

        tail = self.future_tail[
            self.waypoint_index
        ]

        return max(
            0.0,
            raw_distance + tail,
        )

    def _observation(self):
        if not self._finite_state(
            self.latest_state
        ):
            raise RuntimeError(
                "Invalid state_vector"
            )

        obs = np.asarray(
            self.latest_state.state,
            dtype=np.float32,
        ).copy()

        remaining = (
            self._route_remaining()
        )

        denom = max(
            float(
                self.initial_route_distance
            ),
            1e-6,
        )

        obs[7] = np.clip(
            remaining / denom,
            0.0,
            1.0,
        )

        return obs

    # ======================================================================
    # 0.5-s simulation-time stepping
    # ======================================================================

    def _wait_for_decision_boundary(
        self,
        previous_stamp_ns,
        previous_seq,
        timeout_wall=20.0,
        stop_on_terminal=True,
    ):
        target_ns = (
            int(previous_stamp_ns)
            + int(
                DECISION_PERIOD
                * 1e9
            )
        )

        deadline = (
            time.monotonic()
            + timeout_wall
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            self._spin_once(0.01)

            self._poll_goal_manager()

            if stop_on_terminal:
                if (
                    self.contact_since_step
                    or self.route_success
                    or self.nav_failure_reason
                ):
                    return

            if (
                self.latest_state is None
                or self.state_seq
                <= previous_seq
            ):
                continue

            stamp_ns = self._stamp_ns(
                self.latest_state
            )

            # Allow 1 ms numerical/timer tolerance.
            if stamp_ns >= (
                target_ns - 1_000_000
            ):
                return

        raise RuntimeError(
            "Timed out waiting for next "
            "0.5-s state boundary"
        )

    # ======================================================================
    # Reset
    # ======================================================================

    def reset(
        self,
        *,
        options=None,
    ):
        if options is None:
            options = {}

        profile = options.get(
            "network_profile",
            "normal",
        )

        self.log.info(
            "================================"
        )
        self.log.info(
            "B2 EPISODE RESET"
        )
        self.log.info(
            "================================"
        )

        # Cancel previous route if necessary.
        self._cancel_active_goal()

        # Reset local bookkeeping.
        self.waypoint_index = 0

        self.goal_send_future = None
        self.goal_handle = None
        self.goal_result_future = None

        self.route_success = False
        self.nav_failure_reason = None

        self.waypoint_transition_this_step = False
        self.transition_waiting_feedback = False

        self.initial_route_distance = None
        self.previous_route_remaining = None

        self.previous_executed_action = None

        self.step_count = 0
        self.episode_active = False

        self.override_since_step = False

        self.latest_contact = False
        self.contact_since_step = False
        self.contact_episode = False

        # Reset should happen under healthy communication.
        self._set_network_profile(
            "normal"
        )

        # Physics must run for Gazebo, odom and AMCL reset.
        self._unpause()

        # Force SAFE_STOP Q0 while preparing episode.
        msg = UInt8()
        msg.data = 6
        self.action_pub.publish(msg)

        self._spin_for_wall(0.5)

        # Reset deterministic crossing obstacle only when enabled.
        if self.use_dynamic_obstacle:
            self._call_obstacle(
                "reset",
                required=True,
            )

        # Gazebo + AMCL reset.
        self._teleport(
            *self.rl_start
        )

        # Old goal feedback should now be stale.
        if not self.nav.wait_for_server(
            timeout_sec=15.0
        ):
            raise RuntimeError(
                "navigate_to_pose action server "
                "unavailable"
            )

        robot_seq_before_goal = (
            self.robot_seq
        )

        # Dispatch P1 first.
        self._dispatch_goal(0)
        self._wait_goal_accepted()

        # Confirm that P1 actually generated fresh
        # NavigateToPose distance feedback.
        p1_distance = (
            self._wait_robot_goal_feedback(
                robot_seq_before_goal
            )
        )

        self.log.info(
            "Fresh P1 distance_remaining: "
            f"{p1_distance:.3f} m"
        )

        # NOW reset state aggregator so dg0 cannot be
        # captured from stale feedback from an old episode.
        state_seq_before_reset = (
            self.state_seq
        )

        self.episode_reset_pub.publish(
            EmptyMsg()
        )

        self._wait_state_ready(
            state_seq_before_reset,
            expected_initial_goal=p1_distance,
        )

        # Route-level distance for observation[7].
        self.initial_route_distance = (
            self._route_remaining()
        )

        self.previous_route_remaining = (
            self.initial_route_distance
        )

        self.log.info(
            "Initial route distance: "
            f"{self.initial_route_distance:.3f} m"
        )

        # Apply requested episode degradation only
        # after localization and goal initialization.
        self._set_network_profile(
            profile
        )

        # SAFE_STOP during network warm-up.
        warmup_stamp = self._stamp_ns(
            self.latest_state
        )
        warmup_seq = self.state_seq

        self._wait_for_decision_boundary(
            warmup_stamp,
            warmup_seq,
            stop_on_terminal=False,
        )

        # Freeze simulation while policy computes.
        self._pause()

        # Drain already-delivered DDS messages.
        for _ in range(3):
            self._spin_once(0.0)

        self.previous_route_remaining = (
            self._route_remaining()
        )

        self.episode_start_sim_ns = (
            self.node
            .get_clock()
            .now()
            .nanoseconds
        )

        # The actual configuration at the first policy boundary
        # is normally SAFE_STOP Q0 from reset preparation. Keep it
        # as the previous applied action so the first real mode change
        # is counted as a switch.
        self.previous_executed_action = (
            self._current_executed_action()
        )

        self.episode_active = True

        obs = self._observation()

        info = {
            "network_profile":
                self.current_network_profile,
            "waypoint":
                self.waypoints[
                    self.waypoint_index
                ][0],
            "initial_route_distance_m":
                float(
                    self.initial_route_distance
                ),
            "route_remaining_m":
                float(
                    self.previous_route_remaining
                ),
            "geometric_route_length_m":
                float(
                    self.geometric_route_length
                ),
            "physics_paused":
                bool(
                    self.physics_paused
                ),
        }

        return obs, info

    # ======================================================================
    # Step
    # ======================================================================

    def step(self, action):
        if not self.episode_active:
            raise RuntimeError(
                "step() called before reset() "
                "or after episode end"
            )

        action = int(action)

        if not 0 <= action <= 8:
            raise ValueError(
                f"action must be 0..8, got {action}"
            )

        if self.latest_state is None:
            raise RuntimeError(
                "No state available"
            )

        previous_stamp_ns = (
            self._stamp_ns(
                self.latest_state
            )
        )

        previous_seq = self.state_seq

        previous_route = float(
            self.previous_route_remaining
        )

        self.override_since_step = False
        self.contact_since_step = False
        self.waypoint_transition_this_step = False

        # --------------------------------------------------------------
        # Request high-level action while simulator is frozen.
        # --------------------------------------------------------------

        msg = UInt8()
        msg.data = action

        self.action_pub.publish(msg)

        # Allow DDS callback delivery while still paused.
        for _ in range(3):
            self._spin_once(0.0)

        # --------------------------------------------------------------
        # Run exactly to next simulation-time state boundary.
        # --------------------------------------------------------------

        self._unpause()

        try:
            self._wait_for_decision_boundary(
                previous_stamp_ns,
                previous_seq,
                stop_on_terminal=True,
            )
        finally:
            self._pause()

        # Drain callbacks already queued at pause boundary.
        for _ in range(5):
            self._spin_once(0.0)

        self._poll_goal_manager()

        self.step_count += 1

        current_stamp_ns = (
            self._stamp_ns(
                self.latest_state
            )
            if self.latest_state
            is not None
            else previous_stamp_ns
        )

        state_dt = max(
            0.0,
            (
                current_stamp_ns
                - previous_stamp_ns
            ) / 1e9,
        )

        # --------------------------------------------------------------
        # Executed action
        # --------------------------------------------------------------

        executed_action = (
            self._current_executed_action()
        )

        if executed_action is None:
            if (
                self.latest_action_executed
                is None
            ):
                raise RuntimeError(
                    "No executed action available"
                )

            executed_action = int(
                self.latest_action_executed
            )

        executed_nav = (
            executed_action // 3
        )

        executed_qos = (
            executed_action % 3
        )

        overridden = bool(
            self.override_since_step
            or executed_action != action
        )

        switched = bool(
            self.previous_executed_action
            is not None
            and executed_action
            != self.previous_executed_action
        )

        # --------------------------------------------------------------
        # Route-level progress
        # --------------------------------------------------------------

        if self.route_success:
            route_remaining = 0.0
        else:
            route_remaining = (
                self._route_remaining()
            )

        delta_route = (
            previous_route
            - route_remaining
        )

        # The active sparse-leg coordinate changes reference when
        # Nav2 advances to the next waypoint. Suppress progress reward
        # for exactly that transition step so changing the active leg
        # itself cannot create artificial reward or penalty.
        if self.waypoint_transition_this_step:
            delta_for_reward = 0.0
        else:
            delta_for_reward = delta_route

        raw = self.latest_state.raw_state

        # --------------------------------------------------------------
        # Frozen B1c reward
        # --------------------------------------------------------------

        reward, components = (
            calculate_reward(
                delta_goal=float(
                    delta_for_reward
                ),
                scan_age=float(raw[4]),
                camera_age=float(raw[5]),
                dist_obstacle=float(raw[6]),
                nav_mode=int(
                    executed_nav
                ),
                qos_mode=int(
                    executed_qos
                ),
                switched=switched,
                overridden=overridden,
                config=self.reward_config,
            )
        )

        reward = float(reward)

        # --------------------------------------------------------------
        # Episode status
        # --------------------------------------------------------------

        now_sim_ns = (
            self.node
            .get_clock()
            .now()
            .nanoseconds
        )

        elapsed_sim = max(
            0.0,
            (
                now_sim_ns
                - self.episode_start_sim_ns
            ) / 1e9,
        )

        collision = bool(
            self.contact_since_step
            or self.latest_contact
        )

        timeout = bool(
            self.step_count >= MAX_STEPS
            or elapsed_sim
            >= EPISODE_TIMEOUT
        )

        terminated = False
        truncated = False
        terminal_adjustment = 0.0
        termination_reason = ""

        # Physical collision takes precedence over
        # simultaneous goal completion.
        if collision:
            terminated = True

            terminal_adjustment = (
                self.collision_penalty
            )

            termination_reason = (
                "physical_collision"
            )

        elif self.route_success:
            terminated = True

            terminal_adjustment = (
                self.goal_reward
            )

            termination_reason = (
                "goal_success"
            )

        elif self.nav_failure_reason:
            truncated = True

            terminal_adjustment = (
                self.truncation_penalty
            )

            termination_reason = (
                self.nav_failure_reason
            )

        elif timeout:
            truncated = True

            terminal_adjustment = (
                self.truncation_penalty
            )

            termination_reason = (
                "timeout"
            )

        reward += terminal_adjustment

        # --------------------------------------------------------------
        # Corrected route-level observation
        # --------------------------------------------------------------

        self.previous_route_remaining = (
            route_remaining
        )

        self.previous_executed_action = (
            executed_action
        )

        obs = self._observation()

        components = dict(components)

        components[
            "terminal_adjustment"
        ] = float(
            terminal_adjustment
        )

        components[
            "reward_total"
        ] = float(reward)

        info = {
            "step":
                int(self.step_count),

            "state_dt_s":
                float(state_dt),

            "elapsed_sim_s":
                float(elapsed_sim),

            "requested_action":
                int(action),

            # Safety-supervisor filtered command.
            "filtered_action":
                (
                    int(self.latest_action_executed)
                    if self.latest_action_executed is not None
                    else None
                ),

            # Actual nav/QoS configuration observed in state_vector.
            "applied_action":
                int(executed_action),

            # Backward-compatible alias.
            "executed_action":
                int(executed_action),

            "executed_nav_mode":
                int(executed_nav),

            "executed_qos_mode":
                int(executed_qos),

            "switched":
                bool(switched),

            "overridden":
                bool(overridden),

            "waypoint":
                self.waypoints[
                    self.waypoint_index
                ][0],

            "waypoint_index":
                int(
                    self.waypoint_index
                ),

            "waypoint_transition":
                bool(
                    self.waypoint_transition_this_step
                ),

            "route_remaining_m":
                float(route_remaining),

            "delta_route_m":
                float(delta_route),

            "delta_reward_m":
                float(delta_for_reward),

            "physical_collision":
                bool(collision),

            "route_success":
                bool(
                    self.route_success
                ),

            "terminated":
                bool(terminated),

            "truncated":
                bool(truncated),

            "termination_reason":
                termination_reason,

            "network_profile":
                self.current_network_profile,

            "reward_components":
                components,

            "physics_paused":
                bool(
                    self.physics_paused
                ),
        }

        if terminated or truncated:
            self.episode_active = False

            if not self.route_success:
                self._cancel_active_goal()

        return (
            obs,
            float(reward),
            bool(terminated),
            bool(truncated),
            info,
        )

    # ======================================================================
    # Shutdown
    # ======================================================================

    def close(self):
        try:
            self._cancel_active_goal()
        except Exception:
            pass

        try:
            msg = UInt8()
            msg.data = 6
            self.action_pub.publish(msg)
        except Exception:
            pass

        try:
            if not self.physics_paused:
                self._pause()
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass

        if (
            self._owns_rclpy
            and rclpy.ok()
        ):
            rclpy.shutdown()
