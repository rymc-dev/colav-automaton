__author__ = "Ryan McKee <r.mckee@liverpool.ac.uk>"
__version__ = "0.0.1"
__description__ = "dynamics functions for the states of hybrid-automaton colav-automaton" \
                  ""

from .dynamics import (
    constant_heading_dynamics,
    flow_los_heading,
    PrescribedTimeController,
    CollisionAvoidanceController,
    S1_waypoint_reaching_dynamics,
    S2_collision_avoidance_dynamics,
    S3_constant_control_dynamics
)

__all__ = [
    'constant_heading_dynamics',
    'flow_los_heading',
    'PrescribedTimeController',
    'CollisionAvoidanceController',
    'S1_waypoint_reaching_dynamics',
    'S2_collision_avoidance_dynamics',
    'S3_constant_control_dynamics'
]