from typing import List, Dict

def is_goal_waypoint_invariant(x: List, aux_x: Dict, ctx: Dict, u: Dict, dt: float) -> bool:
    if ctx["waypoints"] == 1: 
        return True
    return False
    