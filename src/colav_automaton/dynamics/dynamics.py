import math
import numpy as np
from hybrid_automaton import RuntimeContext
from hybrid_automaton.definition import continuous_dynamics

@continuous_dynamics
def constant_heading_dynamics(
    ctx: RuntimeContext
) -> np.ndarray:
    """
    State x: [px, py, theta, v, yaw_rate]
    """

    px, py, theta, v, yaw_rate = ctx.continuous_state.latest()
    v_des = ctx.configuration.get("constant_velocity", 2.0)  # target speed
    k_v   = ctx.configuration.get("k_v", 1.0)                # velocity gain

    # kinematics
    px_dot = v * math.cos(theta)
    py_dot = v * math.sin(theta)
    theta_dot = 0.0

    # drive v -> v_des
    v_dot = k_v * (v_des - v)

    yaw_rate_dot = 0.0

    return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

@continuous_dynamics
def flow_los_heading(
    ctx: RuntimeContext
) -> np.ndarray:
    """
    LOS heading control with velocity driven toward constant_velocity.
    State x: [px, py, theta, v, yaw_rate]
    """

    px, py, theta, v, yaw_rate = ctx.continuous_state.latest()

    # Waypoint
    waypoint = ctx.auxiliary_states['waypoints'].latest()
    x_wp = waypoint[0]
    y_wp = waypoint[1]
    # LOS desired heading
    desired_heading = math.atan2(y_wp - py, x_wp - px)

    # Heading error in [-pi, pi]
    e_theta = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi

    # Heading gain
    k_theta = ctx.configuration.get("k_theta", 1.0)

    # Heading dynamics
    theta_dot = k_theta * e_theta

    # Translational dynamics using current speed
    px_dot = v * math.cos(theta)
    py_dot = v * math.sin(theta)

    # Velocity control toward target
    v_des = ctx.configuration.get("constant_velocity", 2.0)
    k_v   = ctx.configuration.get("k_v", 1.0)
    v_dot = k_v * (v_des - v)

    yaw_rate_dot = 0.0

    return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

@continuous_dynamics
def hold_position_dynamics(
    ctx: RuntimeContext
) -> np.ndarray:
    """
    Brakes to a stop and holds position/heading - used by Fallback so a
    degraded-safety state doesn't keep advancing on whatever heading it
    happened to have when it was entered. Unlike constant_heading_dynamics
    (drives v toward constant_velocity, i.e. keeps cruising), this drives
    v toward 0 with the same k_v gain, so the agent decelerates rather than
    continuing to close on - or drive through - whatever unsafe region
    triggered unsafe_conditions_guard in the first place.
    State x: [px, py, theta, v, yaw_rate]
    """

    px, py, theta, v, yaw_rate = ctx.continuous_state.latest()
    k_v = ctx.configuration.get("k_v", 1.0)  # velocity gain

    # kinematics - still integrates from current v so this is a
    # deceleration, not a teleport to a dead stop
    px_dot = v * math.cos(theta)
    py_dot = v * math.sin(theta)
    theta_dot = 0.0

    # drive v -> 0 (brake), not v -> constant_velocity
    v_dot = k_v * (0.0 - v)

    yaw_rate_dot = 0.0

    return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

