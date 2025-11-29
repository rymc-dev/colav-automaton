from hybrid_automaton import Automaton, State, Transition
import os
import sys

sys.path.append([os.path.join(os.path.dirname(__file__), dirname) for dirname in ['guards', 'resets', 'dynamics', 'integration', 'invariants']])

from guards import *
from resets import *
from invariants import *
from dynamics import *
from integration import *

def ColavAutomaton(real_time_mode: bool = False) -> Automaton:
    """state definitions""" 
    q1 = State(
        name="Cruise",
        initial=True
    )
    q2 = State(
        name="Transition to LOS"
    )
    q3 = State(
        name="Fallback"
    )
    q4 = State( 
        name="Waypoint Reached",
        invariants=[is_goal_waypoint_invariant]
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
        # resets=... # TODO: Add the virutal waypoint generation
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
        guards=[virtual_waypoints_guard]
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
        real_time_mode=real_time_mode,
    )
    ha.activate(
        [0.0, 0.0, 0.0, 0.0, 0.0],
        {"waypoints": [(10.0, 10.0), (20.0, 20.0)]}
    )
    ha.step()

if __name__ == '__main__': 
    automaton = ColavAutomaton(real_time_mode=False)