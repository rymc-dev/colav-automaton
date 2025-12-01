# !/usr/bin/python3 
# <---utf-8--->

""" 
guards for colav_automaton hybrid automaton which is built on the hybrid_automaton framework 
packages. 
"""

import math
from typing import Dict, List
from shapely.geometry import Polygon, LineString, Point
from hybrid_automaton.automaton import Automaton
import numpy as np


def heading_not_within_tolerance_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict,  clk: Automaton.Runtime.Clock) -> bool:
    """ 
    guard function for hybrid automaton which checks if a heading provided is 
    not within a heading tolerance

    Docs: 
        Flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_not_within_tolerance_guard.mmd
        DataTable: 


    Args: 
        x: List
            represents continous state of agent in this case
            agent continous state is [x: float, y: float, theta: float]
            theta should be in radians
        aux_x: Dict
            represents auxielary continous states within state space 
            in this case we utilize aux_x["waypoints"] = [[wx, wy], [wx, wy]]
            to get the current_waypoint we are navigating towards
        cfg: Dict
            represents context values of the hybrid automaton, this can include
            automaton configuration and also details regarding time since last transition
            and so on. In this case we use cfg['cfg']['heading_tolerance'] which is a float
            value representing heading_tolerance in radians. 
        u: Dict
            this are control inputs we expected, this this case is is None by default and not used
        # dt: float
        #     delta time between guard checks, this is not used within this guard however, just 
        #     a required arg for integration with automaton guards # TODO: remove this, it's not needed anymore!

    Returns: 
        Returns True if heading error is OUTSIDE tolerance.

    Raises: 
        ... #TODO: Change cfg to cfg
    """
    if not isinstance(x, np.ndarray) or x.dtype != float or x.shape != (5,):
        raise ValueError('invalid x value for this guard, expected x to be a numpy array of 5 float values')

    if not isinstance(aux_x, list) or not any(getattr(aux, "name", None) == "waypoints" for aux in aux_x):
        raise ValueError('invalid aux_x for this guard: waypoints not found')

    # Extract waypoints from the correct AuxiliaryState
    waypoints_aux = next(aux for aux in aux_x if getattr(aux, "name", None) == "waypoints")
    waypoints = waypoints_aux.state

    if waypoints is None or len(waypoints) <= 0:
        raise ValueError('invalid waypoints in aux_x')
    
    if not isinstance(cfg, Dict):
        raise ValueError('invalid cfg')
    
    # try: 
    #     cfg['heading_tolerance']
    # except Exception as e:
    #     raise Exception('cfg heading tolerance is not in context, but required for this guard')
    
    xx, xy, yaw = x[0:3]            # current position + heading
    wx, wy = waypoints[-1] # waypoint position

    # If exactly at the waypoint → no heading mismatch
    if xx == wx and xy == wy:
        return False

    # desired heading (radians)
    desired = math.atan2(wy - xy, wx - xx)

    # normalized heading error in [-π, π]
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

    # True only if heading error exceeds tolerance
    return abs(error) > cfg["heading_tolerance"]

def heading_within_tolerance_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> bool:
    """ 
    Guard function for hybrid automaton which checks if a heading provided is 
    within a heading tolerance.

    Docs: 
        Flowchart: 
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/heading_within_tolerance_guard.mmd
        DataTable: 
            ...
    
    Args: 
        x: List
            Represents continuous state of the agent.
            Agent continuous state is [x: float, y: float, theta: float]
            theta should be in radians.
        aux_x: Dict
            Represents auxiliary continuous states.
            Uses aux_x["waypoints"] = [[wx, wy], [wx, wy]] for current navigation targets.
        cfg: Dict
            Context values of the hybrid automaton.
            Uses cfg['cfg']['heading_tolerance'] (float, in radians).
        u: Dict
            Control inputs (unused for this guard).
        dt: float
            Delta time between guard checks (unused).

    Returns: 
        Returns True if heading error is WITHIN tolerance.

    Raises:
        ...
    """

    if not isinstance(x, np.ndarray) or x.dtype != float or x.shape != (5,):
        raise ValueError('invalid x value for this guard, expected x to be a numpy array of 5 float values')

    if not isinstance(aux_x, list) or not any(getattr(aux, "name", None) == "waypoints" for aux in aux_x):
        raise ValueError('invalid aux_x for this guard: waypoints not found')

    # Extract waypoints from the correct AuxiliaryState
    waypoints_aux = next(aux for aux in aux_x if getattr(aux, "name", None) == "waypoints")
    waypoints = waypoints_aux.state

    if waypoints is None or len(waypoints) <= 0:
        raise ValueError('invalid waypoints in aux_x')
    
    if not isinstance(cfg, Dict):
        raise ValueError('invalid cfg')
    
    # try: 
    #     cfg['heading_tolerance']
    # except Exception as e:
    #     raise Exception('cfg heading tolerance is not in context, but required for this guard')
    
    xx, xy, yaw = x[0:3]            # current position + heading
    wx, wy = waypoints[-1] # waypoint position

    # If exactly at the waypoint → no heading mismatch
    if xx == wx and xy == wy:
        return False

    # desired heading (radians)
    desired = math.atan2(wy - xy, wx - xx)

    # normalized heading error in [-π, π]
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

    # True only if heading error exceeds tolerance
    return abs(error) < cfg["heading_tolerance"]

def los_clear_to_waypoint_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> bool: 
    """  
    
    Docs: 
        Flowchart: 
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/los_clear_to_waypoint_guard.mmd
        DataTable: 
            ...

    Args: 
        ... 

    Raises: 
        ValueError
        if any of the inputs are invalid

    Returns: 
        bool
            True if line of sight to waypoint is NOT clear representing guard trigger
    """

    current_waypoint: List[float, float] = aux_x["waypoints"][-1]

    los: LineString = LineString([x[0:1], current_waypoint])
    unsafe_region: Polygon = Polygon(aux_x["unsafe_region"])

    if los.intersects(unsafe_region): 
        intersection = los.intersection(unsafe_region)

        if intersection.is_empty:
            return False
        
        if isinstance(intersection, LineString):
            intersection = intersection.interpolate(0.5, normalized=True)

        intersection_distance = math.dist(
            x[0:1], (intersection.x, intersection.y)
        )
        if intersection_distance <= cfg['los_distance_threshold']: 
            return True

    return False

def unsafe_conditions_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> bool: 
    """  
    
    Docs: 
        flowchart:
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/unsafe_conditions_guard.mmd
        DataTable: 
            ...
    
    Args: 
        ...
    
    Raises: 
        ValueError
            if any of the inputs are invalid

    Returns: 
        bool 
            True if unsafe conditions are detected representing guard trigger
    """
    if not isinstance(x, List) or len(x) < 2:
        raise ValueError('invalid x')
    if [float != type(x_val) for x_val in x]:
        raise ValueError('invalid x value types')
    if not isinstance(cfg, Dict):
        raise ValueError('invalid cfg')
    if 'agent_safety_radius' not in cfg:
        raise ValueError('invalid cfg agent_safety_radius')
    if 'unsafe_region' not in cfg:
        raise ValueError('invalid cfg unsafe_region')

    agent_safety_radius = cfg['agent_safety_radius']
    agent_circle = Point(x[0:1]).buffer(agent_safety_radius)

    unsafe_region: Polygon = Polygon(cfg['unsafe_region'])

    return agent_circle.intersects(unsafe_region)

def virtual_waypoints_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> bool:
    """
    validate is there are virtual waypoints in the waypoints list 

    Docs: 
        flowchart: 
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/virtual_waypoints_guard.mmd
        DataTable: 
            ...

    Args: 
        ...

    Raises: 
        ValueError
            if aux_x is invalid
    Returns: 
        bool 
            True if there are virtual waypoints present, representing guard trigger
    """

    if aux_x is None or 'waypoints' not in aux_x:
        raise ValueError('invalid aux_x') 
    
    return len(aux_x['waypoints']) > 1

def waypoint_reached_guard(x: np.array, aux_x: Dict[str, Automaton.Runtime.AuxiliaryState], u: Dict[str, Automaton.Runtime.ControlInput], cfg: Dict, clk: Automaton.Runtime.Clock) -> bool:
    """
    automaton for 

    Docs: 
        flowchart: 
            https://github.com/rymc-dev/colav-automaton/blob/main/docs/guards/waypoint_reached_guard.mmd
        DataTable: 
            ...

    Args: 
        ...
    Raises: 
        ValueError
            if any of the inputs are invalid
    Returns: 
        bool
            True if waypoint is reached within acceptance radius representing guard trigger
    """
    if not isinstance(x, List) or len(x) != 5:
        raise ValueError('invalid x')
    if [float != type(x_val) for x_val in x]:
        raise ValueError('invalid x value types')
    if aux_x is None or 'waypoints' not in aux_x:
        raise ValueError('invalid aux_x')
    if len(aux_x['waypoints']) <= 0:
        raise ValueError('invalid waypoints')
    if not isinstance(cfg, Dict): 
        raise ValueError('invalid ctx')
    if 'cfg' not in ctx or 'acceptance_radius' not in ctx['cfg']:
        raise ValueError('invalid ctx cfg acceptance radius')

    waypoints_reached_acceptance_radius = ctx["cfg"]["acceptance_radius"] 

    return waypoints_reached_acceptance_radius <= math.dist(
        x[0:1], aux_x['waypoints'][-1]
    )