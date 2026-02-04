from hybrid_automaton.definition import invariant
from hybrid_automaton import RuntimeContext 

@invariant
def is_goal_waypoint_invariant(ctx: RuntimeContext) -> bool:
    if ctx.configuration["waypoints"] == 1: 
        return True
    return False
    