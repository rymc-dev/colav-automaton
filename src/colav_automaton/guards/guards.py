# !/usr/bin/python3
# <---utf-8--->

"""
guards for colav_automaton hybrid automaton which is built on the hybrid_automaton framework
packages.
"""

import math
from typing import Dict, Tuple
from shapely.geometry import Polygon, LineString, Point
from hybrid_automaton import RuntimeContext
from hybrid_automaton.definition import guard


@guard
def heading_not_within_tolerance_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the agent's heading
    is not within a heading tolerance of the bearing to its current waypoint.

    Docs:
        Flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_not_within_tolerance_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_not_within_tolerance_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.continuous_state (agent [x, y, theta, ...] - theta in
            radians), ctx.auxiliary_states['waypoints'] (current target
            waypoint [wx, wy]), and ctx.configuration['heading_tolerance_on']
            (float, radians)

    Returns:
        bool
            True if heading error is OUTSIDE tolerance.

    Raises:
        ValueError
            if continuous state isn't a 5-element vector, or the current
            waypoint is missing.
    """
    if len(ctx.continuous_state.latest()) != 5:
        raise ValueError('invalid x value for this guard, expected x to be a numpy array of 5 float values')

    x_state = ctx.continuous_state.latest()
    # Extract waypoints from the correct AuxiliaryState
    waypoints_aux = ctx.auxiliary_states['waypoints']
    waypoint = waypoints_aux.latest()

    if waypoint is None:
        raise ValueError('invalid waypoints in aux_x')

    xx, xy, yaw = x_state[0:3]            # current position + heading
    wx, wy = waypoint # waypoint position

    # If exactly at the waypoint → no heading mismatch
    if xx == wx and xy == wy:
        return False

    # desired heading (radians)
    desired = math.atan2(wy - xy, wx - xx)

    # normalized heading error in [-π, π]
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

    # True only if heading error exceeds tolerance
    return abs(error) > ctx.configuration["heading_tolerance_on"]

@guard
def heading_within_tolerance_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the agent's heading
    is within a heading tolerance of the bearing to its current waypoint.
    Uses a separate (smaller) "off" tolerance from
    heading_not_within_tolerance_guard's "on" tolerance, forming a deadband
    to stop chattering between Cruise and Transition_to_LOS.

    Docs:
        Flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_within_tolerance_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_within_tolerance_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.continuous_state (agent [x, y, theta, ...] - theta in
            radians), ctx.auxiliary_states['waypoints'] (current target
            waypoint [wx, wy]), and ctx.configuration['heading_tolerance_off']
            (float, radians)

    Returns:
        bool
            True if heading error is WITHIN tolerance.

    Raises:
        ValueError
            if continuous state isn't a 5-element vector, waypoints are
            missing from auxiliary state, or configuration isn't a dict.
    """
    if len(ctx.continuous_state.latest()) != 5:
        raise ValueError('invalid x value for this guard, expected x to be a numpy array of 5 float values')

    if 'waypoints' not in ctx.auxiliary_states:
        raise ValueError('invalid aux_x for this guard: waypoints not found')

    x_state = ctx.continuous_state.latest()
    # Extract waypoints from the correct AuxiliaryState
    waypoints_aux = ctx.auxiliary_states['waypoints']
    waypoint= waypoints_aux.latest()

    if waypoint is None:
        raise ValueError('invalid waypoints in aux_x')

    if not isinstance(ctx.configuration, Dict):
        raise ValueError('invalid cfg')

    xx, xy, yaw = x_state[0:3]            # current position + heading
    wx, wy = waypoint # waypoint position

    # If exactly at the waypoint → no heading mismatch
    if xx == wx and xy == wy:
        return False

    # desired heading (radians)
    desired = math.atan2(wy - xy, wx - xx)

    # normalized heading error in [-π, π]
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

    # True only if heading error exceeds tolerance
    return abs(error) < ctx.configuration["heading_tolerance_off"]

@guard
def los_clear_to_waypoint_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the straight-line
    path (line of sight) from the agent's current position to its current
    waypoint passes through the unsafe region within a threshold distance.

    Docs:
        Flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/los_clear_to_waypoint_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/los_clear_to_waypoint_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.continuous_state (agent position), ctx.auxiliary_states['waypoints']
            (current target waypoint) and ['unsafe_region'] (unsafe region
            polygon vertices), and ctx.configuration['los_distance_threshold']
            (float, meters, default 50.0)

    Returns:
        bool
            True if the line of sight to the waypoint is NOT clear
            (intersects the unsafe region within the threshold distance),
            representing guard trigger.
    """

    current_waypoint: Tuple[float, float] = ctx.auxiliary_states['waypoints'].latest()

    los: LineString = LineString([ctx.continuous_state.latest()[0:2], current_waypoint])
    unsafe_region: Polygon = Polygon(ctx.auxiliary_states["unsafe_region"].latest())

    if los.intersects(unsafe_region):
        intersection = los.intersection(unsafe_region)

        if intersection.is_empty:
            return False

        if isinstance(intersection, LineString):
            intersection = intersection.interpolate(0.5, normalized=True)

        intersection_distance = math.dist(
            ctx.continuous_state.latest()[0:2], (intersection.x, intersection.y)
        )
        if intersection_distance <= ctx.configuration.get('los_distance_threshold', 50.0):
            return True

    return False

@guard
def unsafe_conditions_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the agent's safety
    circle (a disk of radius `agent_safety_radius` centered on the agent's
    current position) intersects the unsafe region polygon.

    Docs:
        flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/unsafe_conditions_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/unsafe_conditions_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.continuous_state (agent position), ctx.auxiliary_states['unsafe_region']
            (unsafe region polygon vertices), and ctx.configuration['agent_safety_radius']

    Returns:
        bool
            True if the agent's safety circle intersects the unsafe region,
            representing guard trigger (routes to the Fallback state).
    """
    agent_safety_radius = ctx.configuration.get('agent_safety_radius', 100.0)
    agent_circle = Point(ctx.continuous_state.latest()[0:2]).buffer(agent_safety_radius)

    unsafe_region: Polygon = Polygon(ctx.auxiliary_states['unsafe_region'].latest())

    return agent_circle.intersects(unsafe_region)

@guard
def safe_conditions_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the agent's safety
    circle no longer intersects the unsafe region - the logical inverse of
    unsafe_conditions_guard. Used to recover out of the Fallback state once
    conditions are safe again.

    Docs:
        flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/safe_conditions_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/safe_conditions_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            same inputs as unsafe_conditions_guard

    Returns:
        bool
            True once the agent's safety circle no longer intersects the
            unsafe region, representing guard trigger (routes back to Cruise).
    """
    agent_safety_radius = ctx.configuration.get('agent_safety_radius', 100.0)
    agent_circle = Point(ctx.continuous_state.latest()[0:2]).buffer(agent_safety_radius)

    unsafe_region: Polygon = Polygon(ctx.auxiliary_states['unsafe_region'].latest())

    return not agent_circle.intersects(unsafe_region)

@guard
def virtual_waypoints_guard(ctx: RuntimeContext) -> bool:
    """
    Checks whether there are virtual (intermediate/avoidance) waypoints
    still queued ahead of the original goal waypoint.

    Docs:
        flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/virtual_waypoints_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/virtual_waypoints_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.auxiliary_states['waypoints'].aux_buffer (the full
            waypoint stack - each generate_new_virtual_waypoint() call
            pushes one entry, each pop_virtual_waypoint() call removes one)

    Raises:
        ValueError
            if ctx.auxiliary_states is missing or has no 'waypoints' entry

    Returns:
        bool
            True if there is more than one waypoint on the stack (i.e. at
            least one virtual waypoint queued in front of the goal),
            representing guard trigger.
    """

    if ctx.auxiliary_states is None or 'waypoints' not in ctx.auxiliary_states:
        raise ValueError('invalid aux_x')

    return len(ctx.auxiliary_states['waypoints'].aux_buffer) > 1

@guard
def waypoint_reached_guard(ctx: RuntimeContext) -> bool:
    """
    Guard function for hybrid automaton which checks if the agent has
    reached its current waypoint, within an acceptance radius.

    Docs:
        flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/waypoint_reached_guard.mmd
        DataTable:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/waypoint_reached_guard_datatable.txt

    Args:
        ctx: RuntimeContext
            uses ctx.continuous_state (agent position), ctx.auxiliary_states['waypoints']
            (current target waypoint), and ctx.configuration['acceptance_radius']
            (float, meters, default 20.0)

    Returns:
        bool
            True if waypoint is reached within acceptance radius,
            representing guard trigger.
    """
    pos = ctx.continuous_state.latest()[0:2]
    waypoint = ctx.auxiliary_states['waypoints'].latest()

    waypoints_reached_acceptance_radius = ctx.configuration.get("acceptance_radius", 20.0)
    eval = waypoints_reached_acceptance_radius >= math.dist(
        pos, waypoint
    )

    return eval
