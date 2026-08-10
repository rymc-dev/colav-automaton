"""
Demo driver for ColavAutomaton - runs a single scenario end-to-end and
writes logs + a run summary. This is example/demo code, not part of the
installable colav_automaton package (see src/colav_automaton/automaton.py
for the ColavAutomaton() factory itself).

Usage:
    python scripts/run_demo_scenario.py
"""
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

import asyncio
import logging

import numpy as np

from colav_automaton import ColavAutomaton
from hybrid_automaton import Automaton, RunResult, ContinuousState, AuxiliaryState

logging.basicConfig(level=logging.INFO)

# A few example scenarios (initial_state, unsafe_region vertices, goal waypoint).
# Swap SCENARIO below to try a different one.
SCENARIOS = {
    "head_on_box": dict(
        initial_state=[0.0, 0.0, 0.0, 0.0, 0.0],
        unsafe_region=[
            [40.0, 20.0],
            [80.0, 20.0],
            [80.0, 60.0],
            [40.0, 60.0],
        ],
        waypoint=[180, 140],
    ),
    "offset_box_1": dict(
        initial_state=[0.0, 0.0, 0.0, 0.0, 0.0],
        unsafe_region=[
            [30.0, 0.0],
            [70.0, 0.0],
            [70.0, 40.0],
            [30.0, 40.0],
        ],
        waypoint=[120, 80],
    ),
    "offset_box_2": dict(
        initial_state=[0.0, 0.0, 0.0, 0.0, 0.0],
        unsafe_region=[
            [60.0, 60.0],
            [90.0, 60.0],
            [90.0, 90.0],
            [60.0, 90.0],
        ],
        waypoint=[150, 150],
    ),
}


async def run_scenario(initial_state, unsafe_region, waypoint, output_dir):
    ha: Automaton = ColavAutomaton()
    print(ha)

    results: RunResult = await ha.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state",
            x0=np.array(initial_state),
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
        ),
        initial_auxiliary_states=[
            AuxiliaryState(name="waypoints", aux0=waypoint, aux_buffer_len=10),
            AuxiliaryState(name="unsafe_region", aux0=unsafe_region, aux_buffer_len=10, expected_update_hz=10)
        ],
        delta_time=0.1,
        enable_real_time_mode=False,
        continuous_state_sampler_enabled=True,
        continuous_state_sampler_rate=100,
        enable_self_integration=True,
        auxiliary_states_sampler_enabled=True,
        auxiliary_states_sampler_rate=10,
        should_write_logs=True,
        output_dir=output_dir,
    )
    print(results)
    return results


if __name__ == "__main__":
    scenario_name = sys.argv[1] if len(sys.argv) > 1 else "head_on_box"
    scenario = SCENARIOS[scenario_name]
    output_dir = os.path.join(os.path.dirname(__file__), "..", "colav-automaton-logs")
    asyncio.run(run_scenario(output_dir=output_dir, **scenario))
