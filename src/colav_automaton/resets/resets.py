from hybrid_automaton import Automaton
from typing import Tuple, List, Dict
import numpy as np
from shapely import LineString, Polygon, Point    

def generate_new_virtual_waypoint(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock):
    """"""
    xx, yy, heading=x.get_continous_state()[0:3]

    vertices = np.array(
        aux_x['unsafe_region'].state
    )
    if vertices.size == 0:
        raise RuntimeError(
            "unsafe set does not contain any vertices, Guard must have activated invalidaly"
        ) 

    vertices_reshaped = vertices.reshape(-1, 2)
    polygon = Polygon(vertices_reshaped)
    if not polygon.is_valid:
        raise RuntimeError('Unsafe set polygon is invalid.')

    visible_vertices = []

    for vx, vy in vertices_reshaped:
        ray = LineString([(xx, yy), (vx, vy)])
        if polygon.exterior.crosses(ray):
            continue
        visible_vertices.append((vx, vy))

    if not visible_vertices:
        raise ValueError(
            "No visible vertices from agent's position to unsafe set.")

    visible_vertices_np = np.array(visible_vertices)
    vx_arr = visible_vertices_np[:, 0]
    vy_arr = visible_vertices_np[:, 1]

    # Compute angle relative to agent heading
    global_angles = np.arctan2(vy_arr - yy, vx_arr - xx)
    relative_angles = global_angles - heading


    # Right side = negative angles, pick minimum angle (most right)
    idx_rightmost = int(np.argmin(relative_angles))
    rightmost_x = vx_arr[idx_rightmost]
    rightmost_y = vy_arr[idx_rightmost]

    # Vector from agent to rightmost vertex
    vec = np.array([rightmost_x - xx, rightmost_y - yy])
    norm = np.linalg.norm(vec)
    if norm == 0:
        raise ValueError(
            "Agent position coincides with the rightmost vertex; cannot compute offset direction.")
    direction = vec / norm

    # Compute right perpendicular vector to 'direction' (for right offset)
    # If forward vector is (dx, dy), right vector is (dy, -dx)
    right_perp = np.array([direction[1], -direction[0]])

    adjusted_x = float(rightmost_x + cfg['longitudinal_offset_distance'] * direction[0] + cfg['lateral_offset_distance'] * right_perp[0])
    adjusted_y = float(rightmost_y + cfg['longitudinal_offset_distance'] * direction[1] + cfg['lateral_offset_distance'] * right_perp[1])
    
    aux_x['waypoints'].state.insert(0, np.array([adjusted_x, adjusted_y]))
    return x, aux_x, u


def pop_waypoint(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> Tuple[Dict]:
    """ """ 
    aux_x['waypoints'].state.pop(0)    
    return x, aux_x, u
