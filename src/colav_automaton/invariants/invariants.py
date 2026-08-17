from hybrid_automaton.definition import invariant
from hybrid_automaton import RuntimeContext


@invariant
def failing_invariant(ctx: RuntimeContext) -> bool:
    """Always-false invariant. Used on states that must never idle - the
    automaton has to take an active transition guard every step it's in
    one of these states, or it's considered stuck. Used by Waypoint_Reached
    in automaton.py (Fallback deliberately has no invariant - see the
    comment on Fallback's State() construction)."""
    return False