import math

import numpy as np
from shapely import LineString, Polygon

from hybrid_automaton.definition import reset
from hybrid_automaton import RuntimeContext

# Weights for the "ease of navigation" cost used to pick a side of the
# unsafe set when no COLREGS maneuver_bias is available: how much a radian
# of heading change costs vs. a metre of extra distance to the candidate
# vertex. Tuned so a ~1 radian turn is roughly as costly as a ~50m detour -
# large enough that neither dominates outright.
_ANGLE_COST_WEIGHT = 50.0
_DISTANCE_COST_WEIGHT = 1.0


def _wrap_angle(angle: float) -> float:
    """Wrap an angle (radians) to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _visible_side_vertices(vertices_reshaped, xx, yy, heading, polygon):
    """Find the rightmost (starboard-most) and leftmost (port-most) unsafe-set
    vertices visible from the agent's position, relative to its heading.

    Returns:
        (right_x, right_y, right_angle), (left_x, left_y, left_angle)
        where angle is the vertex's bearing relative to `heading`, wrapped
        to (-pi, pi] (negative = starboard, positive = port).
    """
    visible_vertices = []
    for vx, vy in vertices_reshaped:
        ray = LineString([(xx, yy), (vx, vy)])
        if polygon.exterior.crosses(ray):
            continue
        visible_vertices.append((vx, vy))

    if not visible_vertices:
        raise ValueError("No visible vertices from agent's position to unsafe set.")

    visible_vertices_np = np.array(visible_vertices)
    vx_arr, vy_arr = visible_vertices_np[:, 0], visible_vertices_np[:, 1]

    global_angles = np.arctan2(vy_arr - yy, vx_arr - xx)
    relative_angles = _wrap_angle(global_angles - heading)

    idx_right = int(np.argmin(relative_angles))  # most negative = rightmost/starboard
    idx_left = int(np.argmax(relative_angles))    # most positive = leftmost/port

    right = (vx_arr[idx_right], vy_arr[idx_right], relative_angles[idx_right])
    left = (vx_arr[idx_left], vy_arr[idx_left], relative_angles[idx_left])
    return right, left


def _navigation_cost(angle: float, vx: float, vy: float, xx: float, yy: float) -> float:
    """Lower = easier to navigate to: penalizes both the heading change
    needed to head for the vertex and the distance to it."""
    distance = math.hypot(vx - xx, vy - yy)
    return _ANGLE_COST_WEIGHT * abs(angle) + _DISTANCE_COST_WEIGHT * distance


@reset
def generate_new_virtual_waypoint(ctx: RuntimeContext) -> RuntimeContext:
    """
    Utilizes the unsafe set vertices to generate a new virtual waypoint that
    is an offset from a visible vertex of the unsafe set, chosen from the
    agent's current position and heading. The new waypoint replaces any
    virtual waypoint already queued ahead of the goal - the waypoints
    buffer holds at most one virtual waypoint in front of the original goal
    at a time, rather than stacking a new one on top of it. Repeated reroute
    calls (e3 self-loops on Transit each time los_clear_to_waypoint_guard
    fires) would otherwise pile up a deeper and deeper detour that
    pop_virtual_waypoint then has to unwind one leg at a time, which reads
    as the agent doubling back on itself instead of continuing toward the
    goal.

    Side selection:
        - By default, picks whichever visible vertex (rightmost/starboard
          or leftmost/port) is "easier to navigate to" - the smaller
          combination of heading change and distance, with a starboard
          tie-break (see `_navigation_cost`).
        - If `ctx.auxiliary_states['maneuver_bias']` is present (a
          `{"side": "port"|"starboard", "urgency": 0..1}` dict - see
          `colav_automaton.classification.Maneuver.as_bias()`), it overrides
          the geometric choice - COLREGs compliance takes priority over
          convenience - and scales `lateral_offset_distance` by
          `(1 + urgency)` so higher-urgency encounters get a wider berth.

    Inputs:
        - ctx: hybrid_automaton.RuntimeContext consisting of internal
            continuous state (e.g. position, heading) and auxiliary states
            (unsafe region vertices, waypoints list, and optionally
            maneuver_bias)

    Outputs:
        - ctx: updated RuntimeContext with new waypoint added to auxiliary state
    """
    xx, yy, heading = ctx.continuous_state.latest()[0:3]

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

    right, left = _visible_side_vertices(vertices_reshaped, xx, yy, heading, polygon)

    right_cost = _navigation_cost(right[2], right[0], right[1], xx, yy)
    left_cost = _navigation_cost(left[2], left[0], left[1], xx, yy)
    default_side = "starboard" if right_cost <= left_cost else "port"

    maneuver_bias_aux = ctx.auxiliary_states.get('maneuver_bias') if ctx.auxiliary_states else None
    bias = maneuver_bias_aux.latest() if maneuver_bias_aux is not None else None

    chosen_side = bias.get('side', default_side) if bias else default_side
    urgency = float(bias.get('urgency', 0.0)) if bias else 0.0

    chosen_x, chosen_y, _ = right if chosen_side == "starboard" else left

    waypoints_aux = ctx.auxiliary_states['waypoints']
    goal = np.array(waypoints_aux.latest())
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
    right_perp = np.array([forward[1], -forward[0]])  # perpendicular, starboard side
    perp = right_perp if chosen_side == "starboard" else -right_perp

    lateral_offset = ctx.configuration.get('lateral_offset_distance', 0) * (1.0 + urgency)

    adjusted_x = float(chosen_x + perp[0] * lateral_offset)
    adjusted_y = float(chosen_y + perp[1] * lateral_offset)

    # A virtual waypoint is already queued ahead of the goal - replace it
    # in place instead of stacking another one on top.
    if len(waypoints_aux.aux_buffer) > 1:
        waypoints_aux.pop()
    waypoints_aux.add([adjusted_x, adjusted_y])
    return ctx

@reset
def pop_virtual_waypoint(ctx: RuntimeContext) -> RuntimeContext:
    """ """
    if len(ctx.auxiliary_states['waypoints'].aux_buffer) <= 1:
        raise IndexError("Cannot pop last waypoint")
    ctx.auxiliary_states['waypoints'].pop()
    return ctx
