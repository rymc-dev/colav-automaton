import numpy as np
from shapely import LineString, Polygon   

from hybrid_automaton.definition import reset  
from hybrid_automaton import RuntimeContext


@reset
def generate_new_virtual_waypoint(ctx: RuntimeContext) -> RuntimeContext:
    """
    utilizes the unsafe set vertices to generate a new virtual waypoint that 
    is an offset from the rightmost visible vertex of the unsafe set, 
    based on the agent's current position and heading. The new waypoint is
    then added to the front of the waypoints list in the auxiliary context 
    state for the automaton.
    
    Inputs: 
        - ctx: hybrid_automaton.RuntimeContext consisting of internal
            continuous state (e.g. position, heading) and auxiliary states 
            (e.g. unsafe region vertices, waypoints list)
            
    Outputs: 
        - ctx: updated RuntimeContext with new waypoint added to auxiliary state
    """
    xx, yy, heading=ctx.continuous_state.latest()[0:3]

    vertices = np.array(
        ctx.auxiliary_states['unsafe_region'].latest()
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

    goal = np.array(ctx.auxiliary_states['waypoints'].latest())
    if goal.size == 0:
        raise ValueError(
            "no current waypoint set; cannot determine which side to offset the virtual waypoint toward")
    agent = np.array([xx, yy])
    goal_vec = goal - agent
    goal_norm = np.linalg.norm(goal_vec)
    if goal_norm == 0:
        raise ValueError(
            "Agent position coincides with the current waypoint; cannot compute offset direction.")
    forward = goal_vec / goal_norm                    # agent→goal unit vector
    right_perp = np.array([forward[1], -forward[0]])  # perpendicular to that

    adjusted_x = float(rightmost_x + right_perp[0] * ctx.configuration.get('lateral_offset_distance', 0))
    adjusted_y = float(rightmost_y + right_perp[1] * ctx.configuration.get('lateral_offset_distance', 0))
    ctx.auxiliary_states['waypoints'].add([adjusted_x, adjusted_y])
    return ctx

@reset
def pop_virtual_waypoint(ctx: RuntimeContext) -> RuntimeContext:
    """ """ 
    if len(ctx.auxiliary_states['waypoints'].aux_buffer) <= 1:
        raise IndexError("Cannot pop last waypoint")
    ctx.auxiliary_states['waypoints'].pop()   
    return ctx
