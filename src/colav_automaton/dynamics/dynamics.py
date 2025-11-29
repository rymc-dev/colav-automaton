
from typing import Dict, List
import math

def constant_heading_dynamics(x: List, aux_x: Dict = None, ctx: Dict = None, u:Dict = None, dt: float = 0.1): 
    """ 
    returns the continous dynamics for x

    Output: 
        Tuple: [float, float, float, float, float]
        x., y., theta., velocity., yaw_rate. in this case
        we are outputting 0
    """
    dx = 0.0
    dy = 0.0
    dtheta = 0.0


    return [dx, dy, dtheta]

def flow_los_heading(x: List[float], aux_x: Dict, ctx: Dict, dt: float = 0.1) -> Dict:
    """Continuous dynamics for Turn-to-LOS mode.

    x:      [px, py, theta]
    aux_x:  contains 'waypoints': [(x_wp, y_wp), ...]
    ctx:    may contain tuning parameters (k_theta, etc)
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
    k_theta = ctx.get("k_theta", 1.0)

    # Desired angular rate
    dtheta = k_theta * e_theta

    # Turn-in-place (no translation)
    dx = 0.0
    dy = 0.0

    return [dx, dy, dtheta]