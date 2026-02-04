
from typing import Dict, List
import math
import numpy as np
from hybrid_automaton import Automaton
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

# def constant_heading_dynamics(
#     x: Automaton.Runtime.ContinousState,
#     aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
#     u: Dict[str, Automaton.Runtime.ControlInput],
#     cfg: Dict,
#     clk: Automaton.Runtime.Clock
# ):
#     """
#     State x: [px, py, theta, v, yaw_rate]
#     Slowly converges heading toward the waypoint during cruise
#     """
#     px, py, theta, v, yaw_rate = x.get_continous_state()
    
#     v_des = cfg.get("constant_velocity", 2.0)  # target speed
#     k_v   = cfg.get("k_v", 1.0)                # velocity gain
    
#     # Get waypoint for heading reference
#     x_wp, y_wp = aux_x["waypoints"].state[0]
    
#     # Compute desired heading toward waypoint
#     desired_heading = math.atan2(y_wp - py, x_wp - px)
    
#     # Heading error in [-pi, pi]
#     e_theta = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi
    
#     # Very slow heading convergence gain (much smaller than LOS)
#     k_theta_cruise = cfg.get("k_theta_cruise", 0.1)  # slow convergence
    
#     # kinematics
#     px_dot = v * math.cos(theta)
#     py_dot = v * math.sin(theta)
    
#     # Slow heading convergence
#     theta_dot = k_theta_cruise * e_theta
    
#     # drive v -> v_des
#     v_dot = k_v * (v_des - v)
    
#     yaw_rate_dot = 0.0
    
#     return np.array([px_dot, py_dot, theta_dot, v_dot, yaw_rate_dot], dtype=float)

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
    x_wp, y_wp = ctx.auxiliary_states.get["waypoints"].latest()

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

