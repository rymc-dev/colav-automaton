import math
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from riskenv import Agent, Obstacle, ObstacleWithMetrics

from colav_automaton.classification import (
    Encounter,
    Maneuver,
    normalize_vessel_type,
    classify_encounter,
    determine_maneuver,
    aggregate_maneuvers,
    classify_unsafe_set_obstacles,
)


def _agent(position=(0.0, 0.0), heading=0.0, speed=2.0):
    return Agent(position=position, heading=heading, speed=speed, yaw_rate=0.0, safety_radius=10.0)


def _obstacle(position, heading, speed=2.0, tag="", safety_radius=10.0):
    return Obstacle(position=position, heading=heading, speed=speed, yaw_rate=0.0, safety_radius=safety_radius, tag=tag)


def _metrics(obstacle, dcpa=float("nan"), tcpa=float("nan")):
    return ObstacleWithMetrics(obstacle=obstacle, tcpa=tcpa, dcpa=dcpa)


# ---------------------------------------------------------------------------
# classify_encounter
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "obstacle_position, obstacle_heading, expected",
    [
        # Reciprocal course, dead ahead both ways -> head-on.
        ((100.0, 0.0), math.pi, Encounter.HEAD_ON),
        # Obstacle crosses from our starboard bow -> we give way.
        ((50.0, -50.0), math.pi / 2, Encounter.CROSSING_GIVE_WAY),
        # Obstacle crosses from our port bow -> we stand on.
        ((50.0, 50.0), -math.pi / 2, Encounter.CROSSING_STAND_ON),
        # Same heading, obstacle dead ahead of us and we're closing -> we overtake it.
        ((50.0, 0.0), 0.0, Encounter.OVERTAKING),
        # Same heading, obstacle dead astern of us -> it overtakes us.
        ((-50.0, 0.0), 0.0, Encounter.OVERTAKEN),
        # Obstacle abeam-ish and diverging heading -> no COLREGS-significant relationship.
        ((-50.0, 0.0), math.pi / 2, Encounter.NONE),
    ],
    ids=[
        "head-on: reciprocal course, dead ahead",
        "crossing give-way: obstacle on our starboard bow",
        "crossing stand-on: obstacle on our port bow",
        "overtaking: we come up on obstacle dead astern of it",
        "overtaken: obstacle comes up on us dead astern",
        "none: abaft the beam but diverging headings",
    ],
)
def test_classify_encounter(obstacle_position, obstacle_heading, expected):
    agent = _agent()
    obstacle = _obstacle(obstacle_position, obstacle_heading)
    assert classify_encounter(agent, obstacle) == expected


def test_classify_encounter_same_position_is_none():
    agent = _agent(position=(5.0, 5.0))
    obstacle = _obstacle((5.0, 5.0), 0.0)
    assert classify_encounter(agent, obstacle) == Encounter.NONE


# ---------------------------------------------------------------------------
# normalize_vessel_type
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "raw_tag, expected_category",
    [
        ("fishing boat", "fishing"),
        ("trawler", "fishing"),
        ("super tanker", "constrained_by_draft"),
        ("sailing", "sailing"),
        ("sailboat", "sailing"),
        ("hydrofoil", "power_driven"),
        ("small motor boat", "power_driven"),
        ("", "power_driven"),
        ("Not Under Command", "not_under_command"),
        ("vessel restricted in ability to maneuver", "restricted_maneuverability"),
    ],
)
def test_normalize_vessel_type(raw_tag, expected_category):
    assert normalize_vessel_type(raw_tag) == expected_category


# ---------------------------------------------------------------------------
# determine_maneuver
# ---------------------------------------------------------------------------

def test_determine_maneuver_head_on_gives_way_to_starboard():
    agent = _agent()
    obstacle = _obstacle((100.0, 0.0), math.pi, tag="cargo")
    maneuver = determine_maneuver(agent, _metrics(obstacle))
    assert maneuver.encounter == Encounter.HEAD_ON
    assert maneuver.side == "starboard"
    assert maneuver.give_way is True
    assert maneuver.urgency > 0.5


def test_determine_maneuver_stand_on_by_default():
    agent = _agent()
    obstacle = _obstacle((50.0, 50.0), -math.pi / 2, tag="cargo")
    maneuver = determine_maneuver(agent, _metrics(obstacle))
    assert maneuver.encounter == Encounter.CROSSING_STAND_ON
    assert maneuver.give_way is False
    assert maneuver.urgency < 0.5


def test_determine_maneuver_rule18_override_forces_give_way():
    """A power-driven agent must keep clear of a fishing vessel even in an
    encounter geometry that would otherwise leave the agent as stand-on."""
    agent = _agent()
    obstacle = _obstacle((50.0, 50.0), -math.pi / 2, tag="fishing boat")
    maneuver = determine_maneuver(agent, _metrics(obstacle))
    assert maneuver.encounter == Encounter.CROSSING_STAND_ON
    assert maneuver.give_way is True
    assert "Rule 18" in maneuver.reason


def test_determine_maneuver_proximity_raises_urgency():
    agent = _agent()
    obstacle = _obstacle((100.0, 0.0), math.pi, tag="cargo")

    far = determine_maneuver(agent, _metrics(obstacle, dcpa=500.0, tcpa=30.0))
    close = determine_maneuver(agent, _metrics(obstacle, dcpa=0.0, tcpa=5.0))

    assert close.urgency > far.urgency
    assert close.urgency <= 1.0


def test_determine_maneuver_ignores_nan_metrics():
    """No CPA data available (NaN) shouldn't blow up or alter the base
    encounter/vessel-type urgency."""
    agent = _agent()
    obstacle = _obstacle((100.0, 0.0), math.pi, tag="cargo")
    maneuver = determine_maneuver(agent, _metrics(obstacle))
    assert maneuver.urgency == pytest.approx(0.9)


def test_maneuver_as_bias():
    maneuver = Maneuver(side="port", give_way=True, urgency=0.42, reason="test")
    assert maneuver.as_bias() == {"side": "port", "urgency": 0.42}


# ---------------------------------------------------------------------------
# aggregate_maneuvers
# ---------------------------------------------------------------------------

def test_aggregate_maneuvers_empty_defaults_to_none():
    result = aggregate_maneuvers([])
    assert result.encounter == Encounter.NONE
    assert result.give_way is False
    assert result.urgency == 0.0


def test_aggregate_maneuvers_highest_urgency_wins():
    low = Maneuver(side="port", give_way=False, urgency=0.2, reason="low")
    high = Maneuver(side="port", give_way=True, urgency=0.9, reason="high")
    assert aggregate_maneuvers([low, high]) is high


def test_aggregate_maneuvers_starboard_wins_ties():
    port = Maneuver(side="port", give_way=True, urgency=0.5, reason="port")
    starboard = Maneuver(side="starboard", give_way=True, urgency=0.5, reason="starboard")
    assert aggregate_maneuvers([port, starboard]) is starboard


# ---------------------------------------------------------------------------
# classify_unsafe_set_obstacles
# ---------------------------------------------------------------------------

def test_classify_unsafe_set_obstacles_filters_out_of_range_ships():
    """A distant head-on ship has a high *base* urgency (Rule 14 starts at
    0.9) regardless of distance - it must not dominate the aggregate just
    because riskenv's own I1/I2/I3 "of interest" filtering would have
    excluded it from the unsafe_region hull in the first place."""
    agent = _agent()
    close = _obstacle((50.0, 50.0), -math.pi / 2, tag="cargo")  # crossing stand-on, in range
    far = _obstacle((5000.0, 0.0), math.pi, tag="cargo")        # head-on, far outside dsf/time_of_interest
    result = classify_unsafe_set_obstacles(agent, [close, far], dsf=100.0, time_of_interest=20.0)
    assert result.encounter == Encounter.CROSSING_STAND_ON


def test_classify_unsafe_set_obstacles_empty_obstacles_defaults_to_none():
    agent = _agent()
    result = classify_unsafe_set_obstacles(agent, [], dsf=50.0, time_of_interest=20.0)
    assert result.encounter == Encounter.NONE


def test_classify_unsafe_set_obstacles_none_of_interest_defaults_to_none():
    """All obstacles present, but none within dsf/time_of_interest -> no
    obstacle is "of interest", so the result is the same default as no
    obstacles at all."""
    agent = _agent()
    far = _obstacle((5000.0, 0.0), math.pi, tag="cargo")
    result = classify_unsafe_set_obstacles(agent, [far], dsf=100.0, time_of_interest=20.0)
    assert result.encounter == Encounter.NONE


if __name__ == '__main__':
    pytest.main([__file__])
