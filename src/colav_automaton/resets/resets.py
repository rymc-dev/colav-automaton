from typing import Tuple, List, Dict
import numpy as np
from shapely import LineString, Polygon, Point    

def generate_new_virtual_waypoint(x: List, aux_x: Dict, ctx: Dict, u: Dict, dt: float):
    """"""
    agent_x = x[0]
    agent_y = x[1]
    agent_heading = x[2]

    vertices = np.array(
        aux_x['unsafe_region']
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
        ray = LineString([(agent_x, agent_y), (vx, vy)])
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
    global_angles = np.arctan2(vy_arr - agent_y, vx_arr - agent_x)
    relative_angles = global_angles - agent_heading


    # Right side = negative angles, pick minimum angle (most right)
    idx_rightmost = int(np.argmin(relative_angles))
    rightmost_x = vx_arr[idx_rightmost]
    rightmost_y = vy_arr[idx_rightmost]

    # Vector from agent to rightmost vertex
    vec = np.array([rightmost_x - agent_x, rightmost_y - agent_y])
    norm = np.linalg.norm(vec)
    if norm == 0:
        raise ValueError(
            "Agent position coincides with the rightmost vertex; cannot compute offset direction.")
    direction = vec / norm

    # Compute right perpendicular vector to 'direction' (for right offset)
    # If forward vector is (dx, dy), right vector is (dy, -dx)
    right_perp = np.array([direction[1], -direction[0]])

    adjusted_x = float(rightmost_x + ctx['longitudinal_offset_distance'] * direction[0] + ctx['lateral_offset_distance'] * right_perp[0])
    adjusted_y = float(rightmost_y + ctx['longitudinal_offset_distance'] * direction[1] + ctx['lateral_offset_distance'] * right_perp[1])

    # Create new virtual waypoint
    new_waypoint = ROSWaypoint(
        position=Point(adjusted_x, adjusted_y, 0.0),
        acceptance_radius=self.__getattribute__('virtual_waypoint_acceptance_radius')
    )

    # Log the waypoint creation
    self.logger.info(f"Generated virtual waypoint at ({adjusted_x:.2f}, {adjusted_y:.2f}) "
                    f"with acceptance radius {self.__getattribute__('virtual_waypoint_acceptance_radius')}")

    waypoints_state.virtual_waypoints.insert(0, new_waypoint)
    reset_output = {'waypoints_state': waypoints_state}
    self._validate_reset_output(reset_output)
    return reset_output

def pop_waypoint(x: List, aux_x: Dict, ctx: Dict, u: Dict, dt: float):
    """ """ 
    aux_x['waypoints'].pop(0)
    return x, aux_x
