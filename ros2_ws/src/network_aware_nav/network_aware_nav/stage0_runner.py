#!/usr/bin/env python3
"""
stage0_runner.py -- Stage 0 episode runner: does wireless degradation actually
break navigation in YOUR setup?

This is the first thing to run and the only thing that matters before any of the
rest is built. It uses Default Nav2 only: no DQN, no 9-D state, no supervisor.
If degradation does not measurably hurt navigation, and if a permanently
conservative policy does not measurably help, then there is nothing for an
adaptive method to exploit and the whole study needs rescoping. Better to learn
that in three days than in three months.

Runs three cells, writing one CSV row per episode:
    default      / normal      -- the reference          (40 episodes)
    default      / severe      -- does degradation bite?  (40 episodes)
    conservative / severe      -- how much headroom?      (40 episodes)

The conservative arm is not a supervisor and not an agent: it is Nav2 with a low
velocity limit pinned through controller parameters, optionally plus a fixed
freshness comms profile (--qos). Total 120 episodes; conservative is not run
under Normal because headroom is only meaningful under degradation.

Then:  python3 analyze.py gate0 --csv stage0.csv

--------------------------------------------------------------------------
VERIFY LOCALLY BEFORE TRUSTING. This has not been executed against a live
ROS 2 stack. The action, service, and topic names below are the ROS 2 Humble
defaults for Nav2 + gazebo_ros, but the controller parameter names in
CONSERVATIVE_PARAMS depend on your controller plugin id, and the degradation
hooks depend on your network_emulator interface. Check both first:

    ros2 param list /controller_server | grep -i vel
    ros2 action list | grep navigate
    ros2 service list | grep set_entity_state
--------------------------------------------------------------------------

USAGE
    # terminal 1: your usual simulation bringup on arena_lab
    # terminal 2:
    ros2 run <your_pkg> stage0_runner.py --episodes 40 --out stage0.csv
    python3 analyze.py gate0 --csv stage0.csv

Requires: rclpy, nav2_msgs, gazebo_msgs, geometry_msgs, nav_msgs, sensor_msgs
"""

import argparse
import csv
import math
import os
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    HAVE_GAZEBO_SRV = True
except ImportError:                                   # pragma: no cover
    HAVE_GAZEBO_SRV = False

# --- configuration ---------------------------------------------------------
ROBOT_NAME = "burger"
COLLISION_DIST = 0.08          # m, LiDAR min range treated as a collision
GOAL_TOLERANCE = 0.20          # m
EPISODE_TIMEOUT = 240.0        # s
SETTLE_TIME = 4.0              # s after teleport, before dispatching the goal

# Arena start/goal, from worlds/arena_lab.json
START = (-1.05, -0.77, 0.0)

# Fixed S-route. Each tuple is:
# (segment_name, x, y, yaw_rad)
WAYPOINTS = [
    ("P1",  0.90, -0.77,  1.5708),  # turn north
    ("P2",  0.90,  0.00,  3.1416),  # turn west
    ("P3", -0.90,  0.00,  1.5708),  # turn north
    ("P4", -0.90,  0.78,  0.0000),  # turn east
    ("G",   0.90,  0.78,  0.0000),
]

GOAL = (0.90, 0.78, 0.0)

# Adjust to your controller plugin id. For MPPI under Nav2 Humble the parameter
# is typically "FollowPath.vx_max"; for DWB it is "FollowPath.max_vel_x".
CONSERVATIVE_PARAMS = {
    "FollowPath.max_vel_x": 0.08,
    "FollowPath.max_speed_xy": 0.08,
}
NORMAL_PARAMS = {
    "FollowPath.max_vel_x": 0.22,
    "FollowPath.max_speed_xy": 0.22,
}

# Degradation hooks. Replace with however your network_emulator is driven.
PROFILE_CMD = {
    "normal": ["ros2", "param", "set", "/network_emulator", "profile", "normal"],
    "severe": ["ros2", "param", "set", "/network_emulator", "profile", "severe"],
}

# Optional: the conservative arm may also pin a freshness-prioritized comms
# profile, if the adaptive relay already exists. If it does not, run with
# --no-qos and read the headroom number as a LOWER BOUND: the communication axis
# was never exercised, so a narrow Gate-0 failure on headroom is not conclusive.
QOS_CMD = {
    "normal":    ["ros2", "param", "set", "/qos_manager", "profile", "normal"],
    "freshness": ["ros2", "param", "set", "/qos_manager", "profile", "freshness"],
}


class Stage0Runner(Node):
    def __init__(self, args):
        super().__init__(
            "stage0_runner",
            parameter_overrides=[
                RclpyParameter(
                    "use_sim_time",
                    RclpyParameter.Type.BOOL,
                    True
                )
            ]
        )
        self.args = args
        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, sensor_qos)

        self.param_cli = self.create_client(SetParameters,
                                            "/controller_server/set_parameters")
        self.state_cli = (self.create_client(SetEntityState, "/set_entity_state")
                          if HAVE_GAZEBO_SRV else None)

        # TF readiness gate used after every Gazebo/AMCL reset.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Used to ensure Gazebo odometry has settled after a physical
        # teleport before AMCL receives a new initial pose.
        self.latest_odom_pose = None
        self.odom_seq = 0

        self._reset_episode()

    # -- callbacks ----------------------------------------------------------
    def _on_scan(self, msg):
        finite = [r for r in msg.ranges
                  if math.isfinite(r) and msg.range_min < r < msg.range_max]
        if finite:
            self.min_scan = min(self.min_scan, min(finite))
            if min(finite) < COLLISION_DIST:
                self.collided = True

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.latest_odom_pose = (
            float(p.x),
            float(p.y),
            float(yaw),
        )
        self.odom_seq += 1

        if self.last_xy is not None:
            self.path_len += math.dist((p.x, p.y), self.last_xy)

        self.last_xy = (p.x, p.y)

    def _reset_episode(self):
        self.min_scan = float("inf")
        self.collided = False
        self.path_len = 0.0
        self.last_xy = None

    def wait_for_start_ready(
        self,
        x,
        y,
        yaw,
        timeout=20.0,
        position_tolerance=0.15,
        yaw_tolerance=0.35,
        max_tf_age=0.50,
        stable_samples=5,
    ):
        """Wait until localization is genuinely stable at the reset pose."""

        deadline = time.time() + timeout
        stable = 0
        last_reason = "no TF received"

        while time.time() < deadline and rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.10)

            try:
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    "base_footprint",
                    Time(),
                )
            except TransformException as exc:
                stable = 0
                last_reason = f"TF unavailable: {exc}"
                continue

            px = float(tf.transform.translation.x)
            py = float(tf.transform.translation.y)

            q = tf.transform.rotation

            # General quaternion -> planar yaw.
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)

            position_error = math.hypot(px - x, py - y)

            yaw_error = abs(
                math.atan2(
                    math.sin(current_yaw - yaw),
                    math.cos(current_yaw - yaw),
                )
            )

            stamp_sec = (
                float(tf.header.stamp.sec)
                + float(tf.header.stamp.nanosec) * 1e-9
            )

            now_sec = (
                self.get_clock().now().nanoseconds / 1e9
            )

            tf_age = now_sec - stamp_sec

            position_ok = position_error <= position_tolerance
            yaw_ok = yaw_error <= yaw_tolerance

            # AMCL may future-date map->odom slightly because of its
            # transform_tolerance, hence the small negative allowance.
            freshness_ok = -0.20 <= tf_age <= max_tf_age

            if position_ok and yaw_ok and freshness_ok:
                stable += 1

                if stable >= stable_samples:
                    self.get_logger().info(
                        "RESET READY: "
                        f"pose=({px:.3f}, {py:.3f}), "
                        f"position_error={position_error:.3f} m, "
                        f"yaw_error={math.degrees(yaw_error):.1f} deg, "
                        f"tf_age={tf_age:.3f} s"
                    )
                    return True
            else:
                stable = 0

                last_reason = (
                    f"pose=({px:.3f},{py:.3f}), "
                    f"position_error={position_error:.3f} m, "
                    f"yaw_error={math.degrees(yaw_error):.1f} deg, "
                    f"tf_age={tf_age:.3f} s"
                )

        self.get_logger().error(
            "RESET NOT READY after "
            f"{timeout:.1f}s: {last_reason}"
        )

        return False

    def wait_for_odom_stable(
        self,
        timeout=8.0,
        stable_samples=8,
        position_step_tolerance=0.01,
        yaw_step_tolerance=0.02,
    ):
        """Wait for fresh, stationary odometry after Gazebo teleport."""

        deadline = time.time() + timeout
        start_seq = self.odom_seq

        previous = None
        stable = 0

        while time.time() < deadline and rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.10)

            # Require an odom sample generated AFTER this function began.
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
                    math.sin(current[2] - previous[2]),
                    math.cos(current[2] - previous[2]),
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
                self.get_logger().info(
                    "ODOM STABLE after teleport: "
                    f"odom=({current[0]:.3f}, "
                    f"{current[1]:.3f}), "
                    f"yaw={math.degrees(current[2]):.1f} deg"
                )
                return True

        self.get_logger().error(
            "ODOM NOT STABLE after Gazebo teleport"
        )
        return False

    # -- helpers ------------------------------------------------------------
    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_hook(self, cmd):
        if not cmd:
            return
        try:
            subprocess.run(cmd, check=False, capture_output=True, timeout=10)
        except Exception as exc:                      # pragma: no cover
            self.get_logger().warn(f"hook failed: {cmd[0]}: {exc}")

    def set_profile(self, profile):
        self.run_hook(PROFILE_CMD.get(profile))

    def set_controller_params(self, params):
        if not self.param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("controller_server params unavailable")
            return
        req = SetParameters.Request()
        for name, val in params.items():
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                double_value=float(val))
            req.parameters.append(Parameter(name=name, value=pv))
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        if fut.result() is None:
            self.get_logger().error(
                "controller parameter update returned no result"
            )
            return

        for requested, result in zip(
            req.parameters,
            fut.result().results
        ):
            if result.successful:
                self.get_logger().info(
                    f"controller param set: "
                    f"{requested.name}"
                )
            else:
                self.get_logger().error(
                    f"controller param FAILED: "
                    f"{requested.name}: {result.reason}"
                )

    def teleport(self, x, y, yaw=0.0):
        """Reset the robot pose in Gazebo, then reseed AMCL with the same pose."""
        if self.state_cli and self.state_cli.wait_for_service(timeout_sec=5.0):
            st = EntityState()
            st.name = ROBOT_NAME
            st.reference_frame = "world"

            # Keep TurtleBot3 slightly above the ground plane.
            # Resetting at z=0 can cause the model to intersect the floor
            # and tip over under Gazebo physics.
            st.pose.position.x = float(x)
            st.pose.position.y = float(y)
            st.pose.position.z = 0.01

            # Explicitly restore an upright planar orientation.
            st.pose.orientation.x = 0.0
            st.pose.orientation.y = 0.0
            st.pose.orientation.z = math.sin(yaw / 2.0)
            st.pose.orientation.w = math.cos(yaw / 2.0)

            # Reset residual motion from the previous episode.
            # Without this, Gazebo preserves the previous linear/angular
            # velocity and the robot may immediately move or rotate after
            # being teleported.
            st.twist.linear.x = 0.0
            st.twist.linear.y = 0.0
            st.twist.linear.z = 0.0
            st.twist.angular.x = 0.0
            st.twist.angular.y = 0.0
            st.twist.angular.z = 0.0

            req = SetEntityState.Request()
            req.state = st
            fut = self.state_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

            if fut.result() is None:
                raise RuntimeError(
                    "Gazebo SetEntityState returned no response"
                )

            if not fut.result().success:
                raise RuntimeError(
                    "Gazebo SetEntityState reported failure"
                )

            # CRITICAL:
            # SetEntityState can complete before the odometry/TF publisher
            # has observed the teleported pose. AMCL compensates an
            # /initialpose according to odometric motion between the
            # message timestamp and the current time. Therefore do not
            # stamp /initialpose until odometry has settled.
            if not self.wait_for_odom_stable():
                raise RuntimeError(
                    "Odometry did not settle after Gazebo teleport"
                )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.03
        # Publish several times so AMCL reliably receives the reset pose.
        for _ in range(3):
            # Never reuse an old timestamp after a Gazebo teleport.
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(msg)
            self.spin_for(0.2)

        self.spin_for(SETTLE_TIME)

        # Teleport invalidates the previous local/global costmaps.
        for srv in (
            "/local_costmap/clear_entirely_local_costmap",
            "/global_costmap/clear_entirely_global_costmap",
        ):
            try:
                subprocess.run(
                    ["ros2", "service", "call", srv,
                     "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                    check=False, capture_output=True, timeout=5
                )
            except Exception as exc:
                self.get_logger().warn(f"costmap clear failed: {exc}")

        self.spin_for(1.0)

        # Do not start an experimental episode merely because a fixed
        # settling time elapsed. Require several consecutive valid TF
        # observations around the commanded reset pose.
        if not self.wait_for_start_ready(x, y, yaw):
            raise RuntimeError(
                "Episode reset failed: localization did not become "
                "stable at START"
            )

    # -- one episode --------------------------------------------------------
    def run_episode(self, policy, profile, index):
        # Every episode is initialized under a healthy network condition.
        # This prevents degraded sensing from corrupting the Gazebo/AMCL
        # reset itself and gives every experimental cell the same initial
        # localization condition.
        self.set_profile("normal")

        conservative = (policy == "conservative")

        self.set_controller_params(
            CONSERVATIVE_PARAMS if conservative else NORMAL_PARAMS
        )

        if self.args.qos:
            self.run_hook(
                QOS_CMD["freshness" if conservative else "normal"]
            )

        # -------------------------------------------------------------
        # Reset physical Gazebo pose + AMCL before every episode
        # -------------------------------------------------------------
        self.teleport(*START)
        self._reset_episode()

        # Network impairment starts only after reset/localization has
        # stabilized. From this point onward it is part of the episode.
        self.set_profile(profile)
        self.spin_for(0.5)

        if not self.nav.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                "navigate_to_pose action server missing"
            )
            return None

        # Total route time begins here, after reset/settling.
        route_t0_ros_ns = self.get_clock().now().nanoseconds
        route_t0_wall = time.monotonic()

        abort_reason = ""
        completed_segments = 0

        # -------------------------------------------------------------
        # Fixed S-route: P1 -> P2 -> P3 -> P4 -> G
        # -------------------------------------------------------------
        for segment_name, gx, gy, gyaw in WAYPOINTS:

            goal = NavigateToPose.Goal()

            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()

            goal.pose.pose.position.x = float(gx)
            goal.pose.pose.position.y = float(gy)
            goal.pose.pose.position.z = 0.0

            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)

            self.get_logger().info(
                f"[{policy}/{profile}] "
                f"sending {segment_name} -> "
                f"({gx:.2f}, {gy:.2f})"
            )

            send_fut = self.nav.send_goal_async(goal)

            rclpy.spin_until_future_complete(
                self,
                send_fut,
                timeout_sec=15.0
            )

            handle = send_fut.result()

            if handle is None or not handle.accepted:
                abort_reason = f"goal_rejected_{segment_name}"
                break

            result_fut = handle.get_result_async()

            # ---------------------------------------------------------
            # Wait for this segment, but enforce timeout on TOTAL route
            # ---------------------------------------------------------
            while rclpy.ok() and not result_fut.done():

                rclpy.spin_once(
                    self,
                    timeout_sec=0.05
                )

                now_ros_ns = self.get_clock().now().nanoseconds

                route_elapsed = max(
                    0.0,
                    (now_ros_ns - route_t0_ros_ns) / 1e9
                )

                if self.collided:
                    abort_reason = f"collision_{segment_name}"

                elif route_elapsed >= EPISODE_TIMEOUT:
                    abort_reason = f"timeout_{segment_name}"

                elif (
                    time.monotonic() - route_t0_wall
                    >= EPISODE_TIMEOUT * 2.0
                ):
                    abort_reason = f"wall_watchdog_{segment_name}"

                if abort_reason:

                    cancel_fut = handle.cancel_goal_async()

                    rclpy.spin_until_future_complete(
                        self,
                        cancel_fut,
                        timeout_sec=5.0
                    )

                    rclpy.spin_until_future_complete(
                        self,
                        result_fut,
                        timeout_sec=3.0
                    )

                    self.spin_for(0.5)
                    break

            if abort_reason:
                break

            status = None

            if result_fut.done():
                wrapped = result_fut.result()

                if wrapped is not None:
                    status = wrapped.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                completed_segments += 1

                self.get_logger().info(
                    f"[{policy}/{profile}] "
                    f"{segment_name} reached "
                    f"({completed_segments}/{len(WAYPOINTS)})"
                )

                # Brief deterministic settling between route segments.
                self.spin_for(0.25)

                continue

            if status == GoalStatus.STATUS_ABORTED:
                abort_reason = f"nav_aborted_{segment_name}"

            elif status == GoalStatus.STATUS_CANCELED:
                abort_reason = f"nav_canceled_{segment_name}"

            elif status == GoalStatus.STATUS_CANCELING:
                abort_reason = f"nav_canceling_{segment_name}"

            elif status == GoalStatus.STATUS_UNKNOWN:
                abort_reason = f"nav_unknown_{segment_name}"

            elif status is None:
                abort_reason = f"result_missing_{segment_name}"

            else:
                abort_reason = (
                    f"nav_status_{status}_{segment_name}"
                )

            break

        # -------------------------------------------------------------
        # Final route metrics
        # -------------------------------------------------------------
        route_now_ros_ns = self.get_clock().now().nanoseconds

        elapsed = max(
            0.0,
            (route_now_ros_ns - route_t0_ros_ns) / 1e9
        )

        reached = (
            completed_segments == len(WAYPOINTS)
            and not self.collided
            and abort_reason == ""
        )

        return dict(
            policy=policy,
            profile=profile,
            episode=index,
            success=int(reached),
            collision=int(self.collided),
            nav_time=round(elapsed, 2) if reached else "",
            path_len=round(self.path_len, 3),
            min_obs_dist=(
                round(self.min_scan, 3)
                if math.isfinite(self.min_scan)
                else ""
            ),
            note=abort_reason
        )


CELLS = [("default", "normal"), ("default", "severe"), ("conservative", "severe")]
FIELDS = ["policy", "profile", "episode", "success", "collision",
          "nav_time", "path_len", "min_obs_dist", "note"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=40,
                    help="episodes per cell; 40 keeps the Wilson interval "
                         "narrower than the 15-point headroom threshold")
    ap.add_argument("--out", default="stage0.csv")
    ap.add_argument("--qos", action="store_true",
                    help="also pin a freshness comms profile on the conservative "
                         "arm; requires the adaptive relay. Without it the "
                         "measured headroom is a lower bound.")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Stage0Runner(args)
    resume = os.path.exists(args.out)
    f = open(args.out, "a" if resume else "w", newline="")
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    if not resume:
        writer.writeheader()

    try:
        for policy, profile in CELLS:
            for i in range(args.episodes):
                row = node.run_episode(policy, profile, i)
                if row is None:
                    continue
                writer.writerow(row)
                f.flush()
                node.get_logger().info(
                    f"[{policy}/{profile}] {i+1}/{args.episodes} "
                    f"success={row['success']} t={row['nav_time']} "
                    f"{row['note']}")
    except KeyboardInterrupt:
        node.get_logger().info("interrupted; partial results kept (append-safe)")
    finally:
        f.close()
        node.destroy_node()
        rclpy.shutdown()

    print(f"\nwrote {args.out}\nnow run:  python3 analyze.py gate0 --csv {args.out}")


if __name__ == "__main__":
    main()
