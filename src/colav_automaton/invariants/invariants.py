from hybrid_automaton.definition import invariant
from hybrid_automaton import RuntimeContext 

@invariant
def is_goal_waypoint_invariant(ctx: RuntimeContext) -> bool:
    if len(ctx.auxiliary_states['waypoints'].latest()) <= 0:
        raise IndexError('no waypoints in auxiliary state!')
    
    if len(ctx.auxiliary_states["waypoints"].latest()) > 1: 
        return True
    return False
   
   
@invariant
def failing_invariant(ctx: RuntimeContext) -> bool:
    return False 