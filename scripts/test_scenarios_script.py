import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", 'src'))

import asyncio

from colav_automaton import ColavAutomaton
from hybrid_automaton import RunResult
from hybrid_automaton import Automaton
from hybrid_automaton import ContinuousState
from hybrid_automaton import AuxiliaryState
from hybrid_automaton import RunResult

scenarios = {
    "t1": {
        "hyperparameters": {
            "heading_tolerance": 0.2,
            "k_theta": 1.0,
            "k_v": 1.0,
            "constant_velocity": 2.0,
            "safety_radius": 30.0,
            "acceptance_radius": 5, 
            "los_distance_threshold": 60.0, 
            "longitudinal_offset_distance": 50.0, 
            "lateral_offset_distance": 50.0
        },
        "environment": {
            "initial_state": [0.0, 0.0, 0.0, 0.0, 0.0],
            "unsafe_region": [
                [40.0, 20.0],
                [80.0, 20.0],
                [80.0, 60.0],
                [40.0, 60.0]
            ],
            "waypoint": [180.0, 140.0]
        }
    }
}

automaton: Automaton = ColavAutomaton(**scenarios["t1"]["hyperparameters"]) 

async def run_automaton():
    results: RunResult = await automaton.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state", 
            x0=scenarios['t1']['environment']['initial_state'], 
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
        ),
        initial_auxiliary_states=[
            AuxiliaryState("waypoints", aux0=scenarios["t1"]['environment']['waypoint']),
            AuxiliaryState("unsafe_region", aux0=scenarios['t1']['environment']['unsafe_region'])
        ],
        delta_time=0.1,
        enable_real_time_mode=False,
        auxiliary_states_sampler_enabled=True,
        auxiliary_states_sampler_rate=10,
        should_write_logs=True,
        output_dir="./colav-automaton_logs" 
    )
    print (results)

asyncio.run(run_automaton())