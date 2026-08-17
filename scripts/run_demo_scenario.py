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
import math

import numpy as np
from riskenv import Agent, Obstacle

from colav_automaton import ColavAutomaton
from colav_automaton.classification import classify_unsafe_set_obstacles
from hybrid_automaton import Automaton, RunResult, ContinuousState, AuxiliaryState

logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger(__name__)

# Matches ColavAutomaton()'s own `safety_radius` default - used here only to
# build the Agent passed to classification, since the constructed Automaton
# doesn't expose its resolved configuration back out.
DEFAULT_SAFETY_RADIUS = 30.0

# Distance safety factor / TCPA horizon for scoping which ships actually
# count as "of interest" for the maneuver decision - see
# colav_automaton.classification.classify_unsafe_set_obstacles. These are
# independent of the (hardcoded, per-scenario) unsafe_region boxes above,
# which stand in for a real riskenv-generated envelope in this demo.
DSF = 40.0
TIME_OF_INTEREST = 20.0

# A few example scenarios (initial_state, unsafe_region vertices, goal
# waypoint, and an optional "ships" list). When "ships" is present, each
# entry (tag/position/heading/speed) is classified via
# colav_automaton.classification and aggregated into a maneuver_bias fed
# alongside unsafe_region - see scripts/generate_unsafe_set.py for the
# fuller example that also derives unsafe_region itself from ships via
# riskenv. Swap SCENARIO below to try a different one.
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
        ships=[
            {"tag": "cargo", "position": (60.0, 40.0), "heading": math.pi, "speed": 3.0},
        ],
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
        ships=[
            {"tag": "fishing boat", "position": (50.0, -10.0), "heading": math.pi / 2, "speed": 1.5},
        ],
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


def _maneuver_bias_for_ships(initial_state, safety_radius, ships):
    """Classify the mock AIS ships that are actually "of interest" (within
    DSF/TIME_OF_INTEREST of the agent - see classify_unsafe_set_obstacles)
    against the agent's initial state, and aggregate into a single
    {"side", "urgency"} maneuver_bias. Returns None if no ships were
    provided for this scenario."""
    if not ships:
        return None

    agent = Agent(
        position=tuple(initial_state[0:2]),
        heading=initial_state[2],
        speed=initial_state[3],
        yaw_rate=initial_state[4],
        safety_radius=safety_radius,
    )
    obstacles = [
        Obstacle(
            position=ship["position"], heading=ship["heading"], speed=ship["speed"],
            yaw_rate=0.0, safety_radius=ship.get("safety_radius", 15.0), tag=ship["tag"],
        )
        for ship in ships
    ]
    maneuver = classify_unsafe_set_obstacles(agent=agent, obstacles=obstacles, dsf=DSF, time_of_interest=TIME_OF_INTEREST)
    _logger.info(
        "aggregated maneuver: %s side=%-9s give_way=%-5s urgency=%.2f (%s)",
        maneuver.encounter.value, maneuver.side, maneuver.give_way, maneuver.urgency, maneuver.reason,
    )
    return maneuver.as_bias()


async def run_scenario(initial_state, unsafe_region, waypoint, output_dir, ships=None):
    ha: Automaton = ColavAutomaton()
    print(ha)

    initial_auxiliary_states = [
        AuxiliaryState(name="waypoints", aux0=waypoint, aux_buffer_len=10),
        AuxiliaryState(name="unsafe_region", aux0=unsafe_region, aux_buffer_len=10, expected_update_hz=10),
    ]
    maneuver_bias = _maneuver_bias_for_ships(initial_state, DEFAULT_SAFETY_RADIUS, ships)
    if maneuver_bias is not None:
        _logger.info("aggregated maneuver_bias: %s", maneuver_bias)
        initial_auxiliary_states.append(
            AuxiliaryState(name="maneuver_bias", aux0=maneuver_bias, aux_buffer_len=10, expected_update_hz=10)
        )

    results: RunResult = await ha.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state",
            x0=np.array(initial_state),
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"]
        ),
        initial_auxiliary_states=initial_auxiliary_states,
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
