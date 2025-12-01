
from typing import Dict, List
import math
import numpy as np
from hybrid_automaton import Automaton

CONSTANT_VELOCITY = 2.0

def constant_heading_dynamics(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],  u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock): 
    """ 
    returns the continous dynamics for x

    Output: 
        Tuple: [float, float, float, float, float]
        x., y., theta., velocity., yaw_rate. in this case
        we are outputting 0
    """
    dx = CONSTANT_VELOCITY
    dy = 0.0
    dtheta = 0.0

    return np.array([dx, dy, dtheta, 0.0, 0.0], dtype=float)

def flow_los_heading(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> Dict:
    """Continuous dynamics for Turn-to-LOS mode.

    x:      [px, py, theta]
    aux_x:  contains 'waypoints': [(x_wp, y_wp), ...]
    cfg:    may contain tuning parameters (k_theta, etc)
    dt:     integration step (if needed externally)

    Returns:
        {
            "xdot": [dx, dy, dtheta],
            "desired_heading": LOS heading
        }
    """

    px = x[0]
    py = x[1]
    theta = x[2]

    # Get final waypoint
    x_wp, y_wp = aux_x['waypoints'][-1]

    # Line-of-sight desired heading
    desired_heading = math.atan2(y_wp - py, x_wp - px)

    # Heading error
    e_theta = (desired_heading - theta + math.pi) % (2 * math.pi) - math.pi

    # Gain for turning behavior
    k_theta = cfg.get("k_theta", 1.0)

    # Desired angular rate
    dtheta = k_theta * e_theta

    # Turn-in-place (no translation)
    dx = 0.0
    dy = 0.0

    return np.array([dx, dy, dtheta, 0.0, 0.0], dtype=float)