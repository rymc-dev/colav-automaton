""" 

"""
from .resets import (
    generate_new_virtual_waypoint,
    pop_virtual_waypoint 
)

__author__ = "Ryan McKee <ryanmckee47@icloud.com>"
__version__ = "1.0.5"
__description__ = "resets for hybrid-automaton transitions that will takes all inputs and " \
                "output update continous states (agent continous state scalers, auxielary continous" \
                "states dictionary) these resets are specifically for colav-automaton"

__all__ = [
    "generate_new_virtual_waypoint",
    "pop_virtual_waypoint"
]