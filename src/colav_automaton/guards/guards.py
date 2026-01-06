# !/usr/bin/python3 
# <---utf-8--->

import numpy as np
from typing import Dict
from hybrid_automaton import Automaton
from shapely.geometry import LineString, Polygon, Point
from shapely.ops import nearest_points 

# ============================================================================
# Ship Navigation Guards
# ============================================================================

def create_unsafe_set_polygon(ox: float, oy: float, Cs: float) -> Polygon:
    """
    Create unsafe set B∞(po, Cs) as a Shapely Polygon.

    Args:
        ox, oy: Obstacle center position
        Cs: Safe distance from obstacle

    Returns:
        Polygon: Square unsafe set centered at obstacle
    """
    return Polygon([
        (ox - Cs, oy - Cs),  # V1 - bottom left
        (ox + Cs, oy - Cs),  # V2 - bottom right
        (ox + Cs, oy + Cs),  # V3 - top right
        (ox - Cs, oy + Cs),  # V4 - top left 
    ])


def create_los_cone(pos_x: float, pos_y: float, xw: float, yw: float, v: float, tp: float) -> Polygon:
    """
    Create LOS cone F(p(t)) = conv(B₂(p(t), vtp), pw) as a Shapely Polygon.

    The cone is formed by the convex hull of:
    - A circle of radius vtp around current position
    - The waypoint

    We approximate the circle with vertices perpendicular to the LOS direction.

    Args:
        pos_x, pos_y: Current ship position
        xw, yw: Waypoint position
        v: Ship velocity
        tp: Prescribed time

    Returns:
        Polygon: LOS cone as convex polygon
    """
    ship_to_waypoint = np.array([xw - pos_x, yw - pos_y])
    dist_to_waypoint = np.linalg.norm(ship_to_waypoint)

    if dist_to_waypoint < 1e-6:
        # At waypoint, return point
        return Point(pos_x, pos_y).buffer(0.01)

    # Unit vector toward waypoint
    unit_to_waypoint = ship_to_waypoint / dist_to_waypoint

    # Perpendicular vector (rotate 90 degrees)
    perp_vector = np.array([-unit_to_waypoint[1], unit_to_waypoint[0]])

    # Radius of uncertainty circle
    radius = v * tp

    # Create cone: left edge, ship position with radius, right edge, waypoint
    left_point = np.array([pos_x, pos_y]) + radius * perp_vector
    right_point = np.array([pos_x, pos_y]) - radius * perp_vector

    # Convex hull forms a triangle/cone
    cone_points = [
        tuple(left_point),
        tuple(right_point),
        (xw, yw)
    ]

    return Polygon(cone_points)


def check_G11(pos_x: float, pos_y: float, ox: float, oy: float, xw: float, yw: float, v: float, tp: float, Cs: float) -> bool:
    """
    G11: Check if waypoint LOS set F(p(t)) intersects unsafe set.

    F(p(t)) = conv(B₂(p(t), vtp), pw)

    Args:
        pos_x, pos_y: Current ship position
        ox, oy: Obstacle center position
        xw, yw: Waypoint position
        v: Ship velocity
        tp: Prescribed time
        Cs: Safe distance from obstacle

    Returns:
        bool: True if LOS cone intersects unsafe set
    """
    # Create geometric objects
    unsafe_set = create_unsafe_set_polygon(ox, oy, Cs)
    los_cone = create_los_cone(pos_x, pos_y, xw, yw, v, tp)

    # Check intersection
    return unsafe_set.intersects(los_cone)


def check_G12(pos_x: float, pos_y: float, ox: float, oy: float, dsafe: float) -> bool:
    """
    G12: Check if distance to obstacle <= dsafe
    
    Args:
        pos_x, pos_y: Current ship position
        ox, oy: Obstacle center position
        dsafe: Safe distance threshold (Cs + vtp)
        
    Returns:
        bool: True if within safe distance
    """
    ds = np.sqrt((pos_x - ox)**2 + (pos_y - oy)**2)
    return ds <= dsafe


def G11_and_G12_guard(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
) -> bool:
    """
    Guard for S1 -> S2 transition: Enter collision avoidance
    
    Activates when obstacle is in path (G11) AND close enough (G12)
        
    Args:
        x: Continuous state [x, y, psi]
        aux_x: Auxiliary states
        u: Control inputs 
        cfg: Configuration containing waypoint, obstacle, v, tp, Cs, dsafe
        clk: Clock
        
    Returns:
        bool: True if should enter collision avoidance
    """
    state = x.get_continous_state()
    
    # Compute distance to obstacle
    ds = np.sqrt((state[0] - cfg['obstacle_x'])**2 + 
                (state[1] - cfg['obstacle_y'])**2)
    G12 = ds <= cfg['dsafe']
    
    G11 = check_G11(
        state[0], state[1],
        cfg['obstacle_x'], cfg['obstacle_y'],
        cfg['waypoint_x'], cfg['waypoint_y'],
        cfg['v'], cfg['tp'], cfg['Cs']
    )
    
    return G11 and G12


def L1_check(pos_x: float, pos_y: float, v1_x: float, v1_y: float, delta: float) -> bool:
    """
    L1: Check if ||p(t) - V1|| > delta (not yet reached V1)
    
    Args:
        pos_x, pos_y: Current ship position
        v1_x, v1_y: Virtual waypoint V1 position
        delta: Arrival tolerance
        
    Returns:
        bool: True if not yet reached V1
    """
    dist_to_v1 = np.sqrt((pos_x - v1_x)**2 + (pos_y - v1_y)**2)
    return dist_to_v1 > delta


def L2_check(pos_x: float, pos_y: float, psi: float, v1_x: float, v1_y: float) -> bool:
    """
    L2: Check if V1 is ahead of ship (within ±π/2 of heading)
    
    Args:
        pos_x, pos_y: Current ship position
        psi: Current heading
        v1_x, v1_y: Virtual waypoint V1 position
        
    Returns:
        bool: True if V1 is ahead
    """
    angle_to_v1 = np.arctan2(v1_y - pos_y, v1_x - pos_x)
    relative_angle = np.arctan2(np.sin(angle_to_v1 - psi), np.cos(angle_to_v1 - psi))
    return -np.pi/2 < relative_angle < np.pi/2


def L1_bar_or_L2_bar_guard(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
) -> bool:
    """
    Guard for S2 -> S3 transition: Enter constant control
    
    Activates when V1 reached (L1) OR V1 is behind (L2)
    
    Docs:
        Corresponds to guard L1 ∨ L2 in ship navigation automaton
        
    Args:
        x: Continuous state [x, y, psi]
        aux_x: Auxiliary states 
        u: Control inputs 
        cfg: Configuration containing delta and ca_controller
        clk: Clock
        
    Returns:
        bool: True if should enter constant control mode 
    """
    state = x.get_continous_state()
    
    # Need virtual waypoint from controller (set by dynamics)
    if 'ca_controller' not in cfg or cfg['ca_controller'] is None:
        return False
    
    if cfg['ca_controller'].virtual_waypoint is None:
        return False
    
    v1_x, v1_y = cfg['ca_controller'].virtual_waypoint
    
    L1 = L1_check(state[0], state[1], v1_x, v1_y, cfg['delta'])
    L2 = L2_check(state[0], state[1], state[2], v1_x, v1_y)
    
    return (not L1) or (not L2)


def not_G11_guard(
    x: Automaton.Runtime.ContinousState,
    aux_x: Dict[str, Automaton.Runtime.AuxiliaryState],
    u: Dict[str, Automaton.Runtime.ControlInput],
    cfg: Dict,
    clk: Automaton.Runtime.Clock
) -> bool:
    """
    Guard for S3 -> S1 transition: Resume waypoint reaching
    
    Activates when LOS to waypoint is clear (G11)
    
    Docs:
        Corresponds to guard G11 in ship navigation automaton
        
    Args:
        x: Continuous state [x, y, psi]
        aux_x: Auxiliary states 
        u: Control inputs 
        cfg: Configuration containing waypoint, obstacle, v, tp, Cs
        clk: Clock
        
    Returns:
        bool: True if LOS is clear and can resume waypoint reaching
    """
    state = x.get_continous_state()
    
    G11 = check_G11(
        state[0], state[1],
        cfg['obstacle_x'], cfg['obstacle_y'],
        cfg['waypoint_x'], cfg['waypoint_y'],
        cfg['v'], cfg['tp'], cfg['Cs']
    )
    
    return not G11 
    