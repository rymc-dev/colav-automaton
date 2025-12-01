from finite_time_control import HeadingFTC
from typing import Tuple
from typing import Dict

from hybrid_automaton import Automaton
import numpy as np

def integrate_simple_vessel_kinematics(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> Tuple[float, float, float]:
    """
    used for simulating the integration of continuous dynamics
    into a finite time controller

    Args: 
        x: float | vector
            represents agetn continous state we are 
            executing commands on for the hybrid automaton,
            for colav its a vector: [x,y,θ,v,θ_rate]
        aux_x: Dict
            a dictionary containing auxielary continous states
            keys and their values, these are continous states
            which retrieves from external sources like sensors
            and so on, for colav we have {'waypoints': [(10.0, 10.0), (100.0, 100.0)]}
        ctx: Dict
            contextual information regarding the state of the automaton
            this will contain current discrete state (Mode | Q), 
            time since last transition time active and configuration 
            information regarding the current implementation of the automaton
        u: Dict
            additional command influeces, this can be stuff like 
            how much sway their is for a boat we need to coutneract.
        dt: float
            delta of time between last integration, which is used to calculate
            the rate delta of the integration we are going to be calculating
            for. 

    Outputs: 
        list (a vector of scalars in np.array format representing the x,y,theta state of automaton) 
    """
    ctrl = HeadingFTC()
    ctrl.update_x_state(x)
    desired_waypoint = aux_x['waypoints'][0] # should get the first waypoint in the list
    ctrl.update_desired_state(desired_waypoint)
    desired_yaw_rate = ctrl.update(dt=dt, ctrl_saturation=True)


