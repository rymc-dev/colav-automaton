from hybrid_automaton import Automaton, State, Transition

from .guards import *
from .resets import *
from .invariants import *
from .dynamics import *
from .integration import *

def ColavAutomaton(heading_tolerance: float = 0.2, k_theta: float = 1.0, k_v: float = 1.0, constant_velocity: float = 2.0, acceptance_radius: float = 0.2) -> Automaton:
    """state definitions""" 
    

    q1 = State(
        name="Cruise",
        initial=True,
        flow=constant_heading_dynamics,
        on_enter=lambda: print('cruise')
    )
    q2 = State(
        name="Transition_to_LOS",
        flow = flow_los_heading,
        on_enter=lambda: print('T2LOS')
    )
    q3 = State(
        name="Fallback",
        flow=constant_heading_dynamics,
        on_enter=lambda: print('fallback')
    )
    q4 = State( 
        name="Waypoint_Reached",
        invariants=[is_goal_waypoint_invariant],
        flow=constant_heading_dynamics,
        on_enter=lambda: print('waypoint_reached')
    )

    """transitions""" 

    # NOTE: transitions from CRUISE (q1)
    e1 = Transition(
        name="e1",
        to_state=q2,
        guards=[heading_not_within_tolerance_guard]
    )
    e2 = Transition(
        name="e2",
        to_state=q2,
        guards=[los_clear_to_waypoint_guard],
        reset=generate_new_virtual_waypoint
    )
    e3 = Transition(
        name="e3",
        to_state=q3,
        guards=[unsafe_conditions_guard]
    )
    e4 = Transition(
        name="e4",
        to_state=q4,
        guards=[waypoint_reached_guard]
    )
    q1.add_transitions([e1, e2, e3, e4])

    # NOTE: transitions from Turn to LOS (q2)
    e5 = Transition(
        name = "e5",
        to_state=q1,
        guards=[heading_within_tolerance_guard]
    )
    e6 = Transition( # NOTE: TO Fallback if unsafe
        name="e6",
        to_state=q1,
        guards=[unsafe_conditions_guard]
    )
    # TODO: Maybe should have transition back to cruise
    q2.add_transitions([e5, e6])

    # NOTE: transitions from FALLBACK (q3)
    # e7 = None
    # q3.add_transition(e7)

    # NOTE: from goal reached
    e7 = Transition(
        name="e7",
        to_state=q1,
        guards=[virtual_waypoints_guard],
        reset=pop_waypoint
    )
    q4.add_transition(e7)


    ha = Automaton(
        name="COLAV Automaton",
        states=[
            q1,
            q2,
            q3,
            q4
        ],
        configuration={
            'heading_tolerance': heading_tolerance,
            'k_theta': k_theta,
            'k_v': k_v,
            'constant_velocity': constant_velocity,
            'acceptance_radius': acceptance_radius
        }
    )

    return ha

if __name__ == '__main__': 
    automaton = ColavAutomaton(real_time_mode=False)