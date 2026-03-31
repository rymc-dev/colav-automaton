import os
import sys

sys.path.append(os.path.dirname(__file__))
# base_dir = os.path.dirname(os.path.abspath(__file__))

# for subdir in ['guards', 'dynamics', 'invariants', 'integration', 'resets']:
#     sys.path.append(os.path.join(base_dir, subdir))

from hybrid_automaton import Automaton
from hybrid_automaton.definition import State
from hybrid_automaton.definition import Transition

from hybrid_automaton import ContinuousState
from hybrid_automaton import IntegrationFunction

from guards import *
from resets import *
from invariants import *
from dynamics import *
from integration import *

import numpy as np

def ColavAutomaton(
    heading_tolerance: float = 0.2,
    k_theta: float = 1.0, 
    k_v: float = 1.0, 
    constant_velocity: float = 2.0, 
    acceptance_radius: float = 10, 
    los_distance_threshold: float = 30.0, 
    longitudinal_offset_distance: float = 10.0, 
    lateral_offset_distance: float = 10.0
) -> Automaton:
    """state definitions""" 
    
    q1 = State(
        name="Cruise",
        initial=True,
        flow=constant_heading_dynamics,
        on_enter=lambda: print('cruise'),
    )
    q2 = State(
        name="Transition_to_LOS",
        flow = flow_los_heading,
        on_enter=lambda: print('T2LOS'),
    )
    q3 = State(
        name="Fallback",
        flow=constant_heading_dynamics,
        on_enter=lambda: print('fallback'),
    )
    q4 = State( 
        name="Waypoint_Reached",
        invariants=[is_goal_waypoint_invariant],
        flow=constant_heading_dynamics,
        on_enter=lambda: print('waypoint_reached'),
        final=True
    )

    """transitions""" 

    # NOTE: transitions from CRUISE (q1)
    e1 = Transition(
        name="e1",
        to_state=q2,
        guards=[heading_not_within_tolerance_guard],
        priority=1
    )
    e2 = Transition(
        name="e2",
        to_state=q2,
        guards=[los_clear_to_waypoint_guard],
        reset=generate_new_virtual_waypoint,
        priority=0
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
        reset=pop_virtual_waypoint
    )
    q4.add_transition(e7)


    ha = Automaton(
        name="COLAV Automaton",
        version="0.0.2",
        states=[
            q1,
            q2,
            q3,
            q4
        ],
        configuration={
            'heading_tolerance_on': heading_tolerance,
            'heading_tolerance_off': heading_tolerance * 0.5, # adds a deadband to stop chattering
            'k_theta': k_theta,
            'k_v': k_v,
            'constant_velocity': constant_velocity,
            'acceptance_radius': acceptance_radius,
            'los_distance_threshold': los_distance_threshold,
            'longitudinal_offset_distance': longitudinal_offset_distance,
            'lateral_offset_distance': lateral_offset_distance
        }
    )

    return ha

def main():
    from hybrid_automaton import RunResult 
    import asyncio 
    ha: Automaton = ColavAutomaton()
    results = None
    
    print (ha)
    
    async def run():
        results: RunResult = await ha.activate(
            initial_continuous_state=ContinuousState(
                name="agent", 
                x0=np.array([0.0, 0.0, 0.0, 0.0, 0.0]), 
                x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
            ),
            initial_auxiliary_states={
                "waypoints": [np.array([40.0, 40.0]), np.array([100.0, 40.0])],
                "unsafe_region": []
            },
            timeout_sec=100.0,
            delta_time=0.1,
            enable_real_time_mode=False,
            continuous_state_sampler_enabled=True,
            continuous_state_sampler_rate=10,
            enable_self_integration=True,
            # auxiliary_states_sampler_enabled=True,
            # auxiliary_states_sampler_rate=10.0,
            should_write_logs=True,
            output_dir="./colav-automaton-logs"
        )
        print (results)
    asyncio.run(run())

if __name__ == '__main__': 
    main()