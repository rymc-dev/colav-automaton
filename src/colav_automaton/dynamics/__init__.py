__author__ = "Ryan McKee <ryanmckee47@icloud.com>"
__version__ = "1.0.0"
__description__ = "dynamics functions for the states of hybrid-automaton colav-automaton"

from .dynamics import constant_heading_dynamics, flow_los_heading

__all__ = [
    'constant_heading_dynamics',
    'flow_los_heading'
]