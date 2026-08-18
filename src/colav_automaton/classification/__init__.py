from .classification import (
    Encounter,
    Maneuver,
    VESSEL_TYPE_PRIORITY,
    normalize_vessel_type,
    classify_encounter,
    determine_maneuver,
    aggregate_maneuvers,
    classify_unsafe_set_obstacles,
)

__author__ = "Ryan McKee <ryanmckee47@icloud.com>"
__version__ = "1.0.6"
__description__ = "COLREGs-informed encounter classification and maneuver selection for colav-automaton"

__all__ = [
    "Encounter",
    "Maneuver",
    "VESSEL_TYPE_PRIORITY",
    "normalize_vessel_type",
    "classify_encounter",
    "determine_maneuver",
    "aggregate_maneuvers",
    "classify_unsafe_set_obstacles",
]
