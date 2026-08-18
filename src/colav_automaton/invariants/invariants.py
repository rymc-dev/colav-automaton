from hybrid_automaton.definition import invariant
from hybrid_automaton import RuntimeContext


@invariant
def failing_invariant(ctx: RuntimeContext) -> bool:
    """Always-false invariant. Used on states that must never idle - the
    automaton has to take an active transition guard every step it's in
    one of these states, or it's considered stuck. Used by Waypoint_Reached
    in automaton.py."""
    return False


@invariant
def fallback_recoverable_invariant(ctx: RuntimeContext) -> bool:
    """Holds (True) as long as Fallback hasn't been idling longer than
    ctx.configuration['fallback_timeout'] seconds without recovering.

    Fallback's own flow (hold_position_dynamics) deliberately doesn't
    re-plan - it just brakes and waits for safe_conditions_guard (e4) to
    clear. Without this invariant, if the unsafe region never clears (e.g.
    a slow-moving or stationary obstacle sitting on the agent's only route)
    the automaton would idle in Fallback indefinitely rather than ever
    reporting that the leg failed.

    Once the timeout is exceeded this returns False; since Fallback is not
    final (unlike Waypoint_Reached), the runtime treats that as an
    ordinary INVARIANT_VIOLATION - RunResult.status becomes FAILURE,
    which hybraut_nav's tactical_node.py already surfaces as an aborted
    ExecuteMission leg (see its termination_code handling), no
    tactical_node-side change needed.
    """
    timeout = ctx.configuration.get('fallback_timeout', 30.0)
    return ctx.clock.get_time_elapsed_since_transition() < timeout