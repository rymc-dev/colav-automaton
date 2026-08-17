"""
End-to-end example: mock AIS ship data -> riskenv risk envelope
(`unsafe_region`) + colav_automaton COLREGs classification (`maneuver_bias`)
-> fed into ColavAutomaton().

This finishes what was previously a non-functional stub importing a
nonexistent `colav_unsafe_set` package - `riskenv` (the actual, published
sibling package) is now a real dependency of colav-automaton (see
pyproject.toml) and is used directly here.

Usage:
    python scripts/generate_unsafe_set.py
"""
import math
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

import asyncio
import logging

import numpy as np
from riskenv import Agent, Obstacle, create_unsafe_set

from colav_automaton import ColavAutomaton
from colav_automaton.classification import classify_unsafe_set_obstacles
from hybrid_automaton import Automaton, RunResult, ContinuousState, AuxiliaryState

logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger(__name__)

# Own-ship initial state: [x, y, heading, speed, yaw_rate].
OWN_SHIP_STATE = [0.0, 0.0, 0.0, 2.0, 0.0]
OWN_SHIP_SAFETY_RADIUS = 15.0
GOAL_WAYPOINT = [200.0, 0.0]

DSF = 40.0                # distance safety factor for riskenv's unsafe-set generation
TIME_OF_INTEREST = 20.0   # seconds, TCPA horizon for riskenv's I3 filter

# Mock AIS contacts - the kind of feed a real deployment would get from an
# AIS receiver/decoder: type ("tag"), position, heading (radians), speed
# (m/s). `tag` is normalized to a COLREGs Rule 18 vessel-type category by
# colav_automaton.classification.normalize_vessel_type.
MOCK_AIS_SHIPS = [
    {"tag": "cargo",         "position": (100.0, 0.5),  "heading": math.pi,        "speed": 3.0},
    {"tag": "fishing boat",  "position": (70.0, -35.0), "heading": math.pi / 2,    "speed": 1.5},
    {"tag": "super tanker",  "position": (160.0, 60.0), "heading": -math.pi / 2,   "speed": 2.5},
]


def _build_obstacles(ships: list) -> list:
    return [
        Obstacle(
            position=ship["position"],
            heading=ship["heading"],
            speed=ship["speed"],
            yaw_rate=0.0,
            safety_radius=ship.get("safety_radius", 15.0),
            tag=ship["tag"],
        )
        for ship in ships
    ]


def build_unsafe_region_and_maneuver_bias(agent: Agent, ships: list):
    """The core integration: mock AIS ships -> (unsafe_region vertices,
    maneuver_bias dict).

    Both are scoped to the *same* obstacle set: `riskenv.create_unsafe_set`
    only folds obstacles that pass its I1/I2/I3 "of interest" filters (dsf
    proximity, obstacle clustering, CPA/TCPA horizon) into the unsafe-region
    hull, so `classify_unsafe_set_obstacles` re-applies that same filtering
    before classifying/aggregating - a ship riskenv would treat as
    irrelevant (too far, no CPA risk within `time_of_interest`) can't
    dominate the maneuver decision just because its encounter geometry
    looks dangerous in isolation. Multi-obstacle encounters are handled by
    aggregate_maneuvers inside it: highest urgency wins, starboard breaks
    ties.

    In a live deployment this same pair of calls would run once per AIS
    update inside an `auxiliary_fn` callback passed to `ha.activate(...)`
    (see `hybrid_automaton._runtime.AuxiliaryStateProvider`) so both
    auxiliary states stay current without restarting the automaton; here
    they're computed once up front for a static demo scenario.
    """
    obstacles = _build_obstacles(ships)

    unsafe_region = create_unsafe_set(agent=agent, obstacles=obstacles, dsf=DSF, time_of_interest=TIME_OF_INTEREST)
    maneuver = classify_unsafe_set_obstacles(agent=agent, obstacles=obstacles, dsf=DSF, time_of_interest=TIME_OF_INTEREST)

    _logger.info(
        "aggregated maneuver: %s side=%-9s give_way=%-5s urgency=%.2f (%s)",
        maneuver.encounter.value, maneuver.side, maneuver.give_way, maneuver.urgency, maneuver.reason,
    )
    return unsafe_region, maneuver.as_bias()


async def run(output_dir: str):
    agent = Agent(
        position=tuple(OWN_SHIP_STATE[0:2]),
        heading=OWN_SHIP_STATE[2],
        speed=OWN_SHIP_STATE[3],
        yaw_rate=OWN_SHIP_STATE[4],
        safety_radius=OWN_SHIP_SAFETY_RADIUS,
    )

    unsafe_region, maneuver_bias = build_unsafe_region_and_maneuver_bias(agent, MOCK_AIS_SHIPS)
    if not unsafe_region:
        raise RuntimeError(
            "No obstacles fell within the risk envelope (dsf too small, or "
            "MOCK_AIS_SHIPS too far away) - nothing to route around."
        )
    _logger.info("unsafe_region vertices: %s", unsafe_region)
    _logger.info("aggregated maneuver_bias: %s", maneuver_bias)

    ha: Automaton = ColavAutomaton(safety_radius=OWN_SHIP_SAFETY_RADIUS)
    print(ha)

    results: RunResult = await ha.activate(
        initial_continuous_state=ContinuousState(
            name="agent_state",
            x0=np.array(OWN_SHIP_STATE),
            x_labels=["x", "y", "theta", "velocity", "yaw_rate"],
        ),
        initial_auxiliary_states=[
            AuxiliaryState(name="waypoints", aux0=GOAL_WAYPOINT, aux_buffer_len=10),
            AuxiliaryState(name="unsafe_region", aux0=unsafe_region, aux_buffer_len=10, expected_update_hz=10),
            AuxiliaryState(name="maneuver_bias", aux0=maneuver_bias, aux_buffer_len=10, expected_update_hz=10),
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
    output_dir = os.path.join(os.path.dirname(__file__), "..", "colav-automaton-logs")
    asyncio.run(run(output_dir))
