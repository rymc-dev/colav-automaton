import logging

from hybrid_automaton import Automaton
from hybrid_automaton.definition import State
from hybrid_automaton.definition import Transition

from .guards import (
    los_clear_to_waypoint_guard,
    unsafe_conditions_guard,
    safe_conditions_guard,
    virtual_waypoints_guard,
    waypoint_reached_guard,
)
from .resets import generate_new_virtual_waypoint, pop_virtual_waypoint
from .invariants import failing_invariant, fallback_recoverable_invariant
from .dynamics import constant_heading_dynamics, flow_los_heading, hold_position_dynamics

_logger = logging.getLogger(__name__)


def ColavAutomaton(
    k_theta: float = 1.0,
    k_v: float = 1.0,
    constant_velocity: float = 2.0,
    safety_radius: float = 30.0,
    acceptance_radius: float = 5,
    los_distance_threshold: float = 60.0,
    longitudinal_offset_distance: float = 50.0,
    lateral_offset_distance: float = 50.0,
    fallback_timeout: float = 30.0
) -> Automaton:
    """state definitions

    3 states, always LOS-guided while under way:

        Transit -> Fallback -> Transit -> Waypoint_Reached -> Transit -> ...

    Cruise and Transition_to_LOS from earlier revisions have been merged
    into a single `Transit` state - LOS heading control (`flow_los_heading`)
    safely subsumes open-loop heading-hold, so splitting the two bought no
    behavior at the cost of two extra guards/transitions
    (heading_not_within_tolerance_guard / heading_within_tolerance_guard)
    purely to gate when LOS correction was "allowed" to run.

    e3 (reroute, generate_new_virtual_waypoint) outranks e2 (Fallback) out
    of Transit - see the priority note on e2/e3 below. This matters because
    los_distance_threshold (where a reroute becomes available) is normally
    farther out than safety_radius (where Fallback triggers), but the two
    can still go true in the same evaluation step at speed/coarse dt - the
    reroute must win that race, not lose it, or the agent ends up braced in
    Fallback for an obstacle it never actually tried to route around.
    """

    q1 = State(
        name="Transit",
        initial=True,
        flow=flow_los_heading,
        on_enter=lambda: _logger.debug("entered Transit"),
    )
    q2 = State(
        name="Fallback",
        # Brake to a stop and hold position (no active re-planning) while
        # too close to the unsafe region - see e2/e4 below.
        # fallback_recoverable_invariant (not failing_invariant - that
        # would force an INVARIANT_VIOLATION on the very next step, and
        # Fallback needs to be able to idle across multiple steps while it
        # waits for safe_conditions_guard/e4 to clear) lets it idle up to
        # fallback_timeout seconds before giving up and failing the leg,
        # rather than idling here indefinitely if the unsafe region never
        # clears.
        flow=hold_position_dynamics,
        invariants=[fallback_recoverable_invariant],
        on_enter=lambda: _logger.debug("entered Fallback"),
    )
    q3 = State(
        name="Waypoint_Reached",
        invariants=[failing_invariant],
        flow=constant_heading_dynamics,
        on_enter=lambda: _logger.debug("entered Waypoint_Reached"),
        final=True
    )

    """transitions"""

    # NOTE: transitions from TRANSIT (q1)
    e1 = Transition(
        name="e1",
        to_state=q3,
        guards=[waypoint_reached_guard],
        priority=0
    )
    e2 = Transition(
        name="e2",
        to_state=q2,
        guards=[unsafe_conditions_guard],
        # Lower precedence than e3 (priority is min-wins - see
        # _Runtime._evaluation_step) - if both guards are true in the same
        # step (plausible at speed/coarse dt, since los_distance_threshold
        # is normally a larger radius than safety_radius), the reroute
        # must get first refusal. Fallback should only ever be reached once
        # a reroute genuinely isn't available, not race it and win by
        # default.
        priority=2
    )
    e3 = Transition(
        name="e3",
        to_state=q1,
        guards=[los_clear_to_waypoint_guard],
        reset=generate_new_virtual_waypoint,
        priority=1
    )
    q1.add_transitions([e1, e2, e3])

    # NOTE: transition from FALLBACK (q2) - recover to Transit once the
    # agent's safety radius no longer intersects the unsafe region.
    e4 = Transition(
        name="e4",
        to_state=q1,
        guards=[safe_conditions_guard],
        priority=0
    )
    q2.add_transition(e4)

    # NOTE: from goal reached
    e5 = Transition(
        name="e5",
        to_state=q1,
        guards=[virtual_waypoints_guard],
        reset=pop_virtual_waypoint,
        priority=0
    )
    q3.add_transition(e5)

    ha = Automaton(
        name="COLAV Automaton",
        version="1.0.5",
        states=[
            q1,
            q2,
            q3
        ],
        configuration={
            'k_theta': k_theta,
            'k_v': k_v,
            'agent_safety_radius': safety_radius,
            'constant_velocity': constant_velocity,
            'acceptance_radius': acceptance_radius,
            'los_distance_threshold': los_distance_threshold,
            'longitudinal_offset_distance': longitudinal_offset_distance,
            'lateral_offset_distance': lateral_offset_distance,
            'fallback_timeout': fallback_timeout
        }
    )

    return ha
