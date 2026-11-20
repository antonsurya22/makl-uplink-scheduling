#!/usr/bin/env python3

"""
Final B1c non-terminal reward definition.

This module is deliberately ROS-independent so that the same reward
implementation can be used by:
  - offline analysis,
  - training environment,
  - evaluation environment,
  - unit tests.

Terminal rewards are handled separately by the RL environment.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RewardConfig:
    # Reward weights
    w_progress: float = 1.0
    w_proximity: float = 1.0
    w_freshness: float = 1.0
    w_qos: float = 0.05
    w_switch: float = 0.10
    w_override: float = 0.50
    time_cost: float = 0.01

    # Component constants
    progress_scale: float = 0.11
    progress_deadband: float = 0.003

    scan_age_scale: float = 3.0
    camera_age_scale: float = 10.0

    d_safe: float = 0.40

    safe_stop_mode: int = 2


QOS_COST = {
    0: 1.0,   # Q0 normal / reliability-oriented
    1: 1.0,   # Q1 freshness-oriented
    2: 0.2,   # Q2 low-bandwidth
}


def clip01(x):
    return max(0.0, min(1.0, float(x)))


def clip11(x):
    return max(-1.0, min(1.0, float(x)))


def _require_finite(**values):
    for name, value in values.items():
        if not math.isfinite(float(value)):
            raise ValueError(
                f"Reward input {name} is non-finite: {value}"
            )


def calculate_reward(
    *,
    delta_goal,
    scan_age,
    camera_age,
    dist_obstacle,
    nav_mode,
    qos_mode,
    switched=False,
    overridden=False,
    config=None,
):
    """
    Calculate one non-terminal reward step.

    `nav_mode` and `qos_mode` should represent the EXECUTED/current
    configuration, not merely the requested action.

    `switched` should represent an executed configuration transition.
    An inadmissible requested action is handled by `overridden`.
    """

    cfg = config or RewardConfig()

    _require_finite(
        delta_goal=delta_goal,
        scan_age=scan_age,
        camera_age=camera_age,
        dist_obstacle=dist_obstacle,
    )

    nav_mode = int(nav_mode)
    qos_mode = int(qos_mode)

    if qos_mode not in QOS_COST:
        raise ValueError(f"Invalid qos_mode: {qos_mode}")

    # ---------------------------------------------------------
    # Progress
    # ---------------------------------------------------------
    if nav_mode == cfg.safe_stop_mode:
        delta_effective = 0.0

    elif abs(delta_goal) <= cfg.progress_deadband:
        delta_effective = 0.0

    else:
        delta_effective = float(delta_goal)

    progress = clip11(
        delta_effective / cfg.progress_scale
    )

    # ---------------------------------------------------------
    # Sensor freshness
    # ---------------------------------------------------------
    scan_age_norm = clip01(
        scan_age / cfg.scan_age_scale
    )

    camera_age_norm = clip01(
        camera_age / cfg.camera_age_scale
    )

    freshness = max(
        scan_age_norm,
        camera_age_norm,
    )

    # ---------------------------------------------------------
    # Proximity
    # ---------------------------------------------------------
    proximity = max(
        0.0,
        (cfg.d_safe - float(dist_obstacle))
        / cfg.d_safe,
    )

    proximity = clip01(proximity)

    # ---------------------------------------------------------
    # Communication resource cost
    # ---------------------------------------------------------
    qos_cost = QOS_COST[qos_mode]

    switch_i = 1.0 if switched else 0.0
    override_i = 1.0 if overridden else 0.0

    # ---------------------------------------------------------
    # Scalar non-terminal reward
    # ---------------------------------------------------------
    reward = (
        cfg.w_progress * progress
        - cfg.w_proximity * proximity
        - cfg.w_freshness * freshness
        - cfg.w_qos * qos_cost
        - cfg.w_switch * switch_i
        - cfg.w_override * override_i
        - cfg.time_cost
    )

    components = {
        "progress": progress,
        "delta_goal_effective": delta_effective,

        "scan_age_norm": scan_age_norm,
        "camera_age_norm": camera_age_norm,
        "freshness": freshness,

        "proximity": proximity,

        "qos_cost": qos_cost,

        "switch": switch_i,
        "override": override_i,

        "reward_nonterminal": reward,
    }

    return reward, components
