from hybrid_automaton.definition import invariant
from hybrid_automaton import RuntimeContext


@invariant
def failing_invariant(ctx: RuntimeContext) -> bool:
    """Always-false invariant. Used on states that must never idle - the
    automaton has to take an active transition guard every step it's in
    one of these states, or it's considered stuck. Used by Fallback and
    Waypoint_Reached in automaton.py."""
    return False