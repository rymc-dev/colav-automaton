"""
COLREGs-informed encounter classification and maneuver selection.

Classifies the geometric relationship between the agent and a nearby ship
into a COLREGs encounter category (Rule 13 overtaking, Rule 14 head-on, Rule
15 crossing), weights that by vessel-type right-of-way (Rule 18), and turns
the result into a `Maneuver` - which side to route around and how urgently
- that `resets.generate_new_virtual_waypoint` can use to bias its otherwise
purely-geometric "easiest side" choice.

This is a practical approximation, not a certified rule-engine: it covers
Rules 13/14/15/17/18 geometrically and by vessel type, but not restricted
visibility (Rule 19), sound/light signals, or simultaneous multi-vessel
priority beyond `aggregate_maneuvers`'s highest-urgency-wins rule.

Built directly on `riskenv`'s `Agent`/`Obstacle`/`ObstacleWithMetrics` types
(the same ones used upstream to build the `unsafe_region` risk envelope) so
there's a single shared definition of "ship" across the risk-envelope and
classification layers.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Literal

from riskenv import (
    Agent,
    Obstacle,
    ObstacleWithMetrics,
    calc_I1,
    calc_I2,
    calc_I3,
    calculate_obstacle_metrics_for_agent,
    unionise_indices_of_interest,
)


class Encounter(Enum):
    """COLREGs encounter category, from the agent's point of view."""
    HEAD_ON = "head_on"
    CROSSING_GIVE_WAY = "crossing_give_way"   # obstacle on our starboard bow - we give way (Rule 15)
    CROSSING_STAND_ON = "crossing_stand_on"   # obstacle on our port bow - we stand on (Rule 15/17)
    OVERTAKING = "overtaking"                 # we are overtaking the obstacle - we give way (Rule 13)
    OVERTAKEN = "overtaken"                   # the obstacle is overtaking us - we stand on (Rule 13)
    NONE = "none"                             # no COLREGS-significant relationship


# Rule 18 responsibilities between vessels: a power-driven vessel keeps out
# of the way of every category below it. Higher rank = more right-of-way.
VESSEL_TYPE_PRIORITY = {
    "power_driven": 0,
    "sailing": 1,
    "fishing": 2,
    "constrained_by_draft": 3,
    "restricted_maneuverability": 4,
    "not_under_command": 5,
}

# Free-text AIS tag substrings -> Rule 18 category. Checked in order, first
# match wins. Anything unrecognized defaults to "power_driven".
_VESSEL_TYPE_KEYWORDS = [
    ("not under command", "not_under_command"),
    ("nuc", "not_under_command"),
    ("restricted", "restricted_maneuverability"),
    ("dredg", "restricted_maneuverability"),
    ("constrained", "constrained_by_draft"),
    ("tanker", "constrained_by_draft"),
    ("fishing", "fishing"),
    ("trawler", "fishing"),
    ("sail", "sailing"),
    ("hydrofoil", "power_driven"),
    ("motor", "power_driven"),
    ("tug", "power_driven"),
    ("cargo", "power_driven"),
]


def normalize_vessel_type(raw_tag: str) -> str:
    """Map a free-text AIS vessel-type tag (e.g. "fishing boat", "super
    tanker", "small motor boat") to a Rule 18 right-of-way category. Empty
    or unrecognized tags default to "power_driven", the lowest-priority
    (most common) category.
    """
    tag = (raw_tag or "").strip().lower()
    for keyword, category in _VESSEL_TYPE_KEYWORDS:
        if keyword in tag:
            return category
    return "power_driven"


# COLREGs sector thresholds (radians).
_HEAD_ON_BEARING_LIMIT = math.radians(15.0)   # obstacle within this of dead ahead, both ways
_HEAD_ON_HEADING_SLACK = math.radians(30.0)   # heading within this of exactly reciprocal
_ABAFT_BEAM_LIMIT = math.radians(112.5)       # Rule 13 overtaking sector boundary
_SAME_DIRECTION_LIMIT = math.radians(67.5)    # heading similarity used to detect "coming up"


def _normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _relative_bearing(from_pos, from_heading: float, to_pos) -> float:
    """Bearing of `to_pos` as seen from `from_pos`, relative to
    `from_heading`. Negative = to starboard, positive = to port - the same
    sign convention `resets.generate_new_virtual_waypoint` uses for
    relative vertex angles.
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return _normalize_angle(math.atan2(dy, dx) - from_heading)


def classify_encounter(agent: Agent, obstacle: Obstacle) -> Encounter:
    """Classify the COLREGs encounter category between `agent` and a single
    `obstacle`, from geometry (position + heading) alone.
    """
    if math.dist(agent.position[:2], obstacle.position[:2]) < 1e-6:
        return Encounter.NONE

    bearing_to_obstacle = _relative_bearing(agent.position, agent.heading, obstacle.position)
    bearing_to_agent = _relative_bearing(obstacle.position, obstacle.heading, agent.position)
    heading_diff = _normalize_angle(obstacle.heading - agent.heading)

    reciprocal = abs(abs(heading_diff) - math.pi) < _HEAD_ON_HEADING_SLACK
    same_direction = abs(heading_diff) < _SAME_DIRECTION_LIMIT

    if (
        abs(bearing_to_obstacle) <= _HEAD_ON_BEARING_LIMIT
        and abs(bearing_to_agent) <= _HEAD_ON_BEARING_LIMIT
        and reciprocal
    ):
        return Encounter.HEAD_ON

    agent_is_abaft_obstacle_beam = abs(bearing_to_agent) > _ABAFT_BEAM_LIMIT
    obstacle_is_abaft_agent_beam = abs(bearing_to_obstacle) > _ABAFT_BEAM_LIMIT

    if same_direction and agent_is_abaft_obstacle_beam:
        return Encounter.OVERTAKING
    if same_direction and obstacle_is_abaft_agent_beam:
        return Encounter.OVERTAKEN

    if abs(bearing_to_obstacle) < _ABAFT_BEAM_LIMIT:
        return Encounter.CROSSING_GIVE_WAY if bearing_to_obstacle < 0 else Encounter.CROSSING_STAND_ON

    return Encounter.NONE


@dataclass
class Maneuver:
    """A recommended avoidance maneuver, derived from a COLREGs encounter
    classification (and/or Rule 18 vessel-type priority).

    Attributes:
        side:      Which side to route the virtual waypoint around -
                   "port" or "starboard".
        give_way:  Whether the agent is obligated to actively give way
                   (True) or should hold course as the stand-on vessel
                   (False) unless forced to act for safety.
        urgency:   0-1. How strongly this should be trusted/enforced -
                   scales the avoidance offset distance in
                   generate_new_virtual_waypoint and how it's weighted
                   in aggregate_maneuvers.
        reason:    Human-readable rule citation, for logging/debugging.
        encounter: The Encounter this maneuver was derived from.
    """
    side: Literal["port", "starboard"]
    give_way: bool
    urgency: float
    reason: str
    encounter: Encounter = Encounter.NONE

    def as_bias(self) -> dict:
        """Serialize to the {"side", "urgency"} dict
        `resets.generate_new_virtual_waypoint` reads from
        `ctx.auxiliary_states['maneuver_bias']`."""
        return {"side": self.side, "urgency": self.urgency}


# Default (side, give_way, urgency, reason) per encounter, before Rule 18 /
# CPA-proximity adjustments. Side defaults to starboard throughout, per the
# international "when in doubt, alter to starboard, and never to port"
# convention (Rule 17) - even stand-on/overtaken/no-encounter cases keep a
# starboard default, but at low urgency so a real geometric override or a
# higher-urgency obstacle takes precedence in aggregate_maneuvers.
_ENCOUNTER_DEFAULTS = {
    Encounter.HEAD_ON: ("starboard", True, 0.9, "Rule 14 head-on: alter course to starboard"),
    Encounter.CROSSING_GIVE_WAY: ("starboard", True, 0.8, "Rule 15 crossing, obstacle to starboard: give way"),
    Encounter.CROSSING_STAND_ON: ("starboard", False, 0.15, "Rule 17 crossing, obstacle to port: stand on (not-to-port if forced to act)"),
    Encounter.OVERTAKING: ("starboard", True, 0.5, "Rule 13 overtaking: give way, default pass to starboard"),
    Encounter.OVERTAKEN: ("starboard", False, 0.1, "Rule 13 being overtaken: stand on (maintain course/speed)"),
    Encounter.NONE: ("starboard", False, 0.0, "No COLREGS-significant encounter geometry"),
}


def determine_maneuver(agent: Agent, obstacle_metrics: ObstacleWithMetrics) -> Maneuver:
    """Determine the avoidance maneuver for a single classified obstacle,
    combining its COLREGs encounter category, Rule 18 vessel-type
    right-of-way (from `obstacle.tag`), and CPA proximity (from `riskenv`'s
    `dcpa`/`tcpa`) into one urgency-weighted `Maneuver`.
    """
    obstacle = obstacle_metrics.obstacle
    encounter = classify_encounter(agent, obstacle)
    side, give_way, urgency, reason = _ENCOUNTER_DEFAULTS[encounter]

    vessel_type = normalize_vessel_type(obstacle.tag)
    if VESSEL_TYPE_PRIORITY.get(vessel_type, 0) > VESSEL_TYPE_PRIORITY["power_driven"]:
        if not give_way:
            give_way = True
            urgency = max(urgency, 0.5)
        else:
            urgency = min(1.0, urgency + 0.2)
        reason += f"; Rule 18: obstacle type '{vessel_type}' outranks power-driven, we must keep clear"

    dcpa, tcpa = obstacle_metrics.dcpa, obstacle_metrics.tcpa
    if not math.isnan(dcpa) and not math.isnan(tcpa) and tcpa >= 0:
        combined_radius = max(agent.safety_radius + obstacle.safety_radius, 1e-6)
        proximity_factor = max(0.0, min(1.0, 1.0 - dcpa / combined_radius))
        urgency = min(1.0, urgency + 0.3 * proximity_factor)

    return Maneuver(side=side, give_way=give_way, urgency=round(urgency, 3), reason=reason, encounter=encounter)


def classify_unsafe_set_obstacles(
    agent: Agent,
    obstacles: List[Obstacle],
    dsf: float,
    time_of_interest: float = 15.0,
) -> Maneuver:
    """Classify only the obstacles that `riskenv.create_unsafe_set` would
    itself fold into the current unsafe region - the union of I1
    (currently within `dsf`), I2 (clustered with another obstacle within
    `dsf`), and I3 (DCPA within `dsf` and TCPA within `time_of_interest`) -
    and aggregate them into a single `Maneuver`.

    This is the multi-obstacle entry point: with several ships around, it's
    not enough to classify each one and aggregate blindly - a distant
    head-on ship posing no real risk still has a high *base* encounter
    urgency (Rule 14 always starts at 0.9) that isn't otherwise tempered by
    distance, so it could wrongly outrank a closer, genuinely dangerous
    crossing encounter in `aggregate_maneuvers`. Filtering to the same
    "of interest" set `riskenv` used to build `unsafe_region` keeps the
    maneuver_bias scoped to exactly the obstacles that unsafe region
    actually represents - use this instead of calling `determine_maneuver`
    directly over a raw, unfiltered ship list. `dsf`/`time_of_interest`
    should match whatever was passed to `riskenv.create_unsafe_set` for the
    same tick.
    """
    metrics = calculate_obstacle_metrics_for_agent(agent=agent, obstacles=obstacles)

    I1 = calc_I1(agent=agent, dynamic_obstacles_with_metrics=metrics, dsf=dsf)
    I2 = calc_I2(I1=I1, dynamic_obstacles_with_metrics=metrics, dsf=dsf)
    I3 = calc_I3(dynamic_obstacles_with_metrics=metrics, dsf=dsf, time_of_interest=time_of_interest)
    uIoI = unionise_indices_of_interest(I1, I2, I3)

    return aggregate_maneuvers([determine_maneuver(agent, obs) for obs in uIoI])


def aggregate_maneuvers(maneuvers: List[Maneuver]) -> Maneuver:
    """Combine multiple obstacles' maneuver decisions (e.g. everyone
    currently contributing to the unsafe region) into a single bias:
    highest urgency wins, starboard wins ties - matching Rule 17's
    "not to port" default.
    """
    if not maneuvers:
        return Maneuver(
            side="starboard", give_way=False, urgency=0.0,
            reason="No obstacles to classify", encounter=Encounter.NONE,
        )

    def _rank(m: Maneuver):
        return (-m.urgency, 0 if m.side == "starboard" else 1)

    return min(maneuvers, key=_rank)
