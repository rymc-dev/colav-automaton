from colav_automaton.guards import (
    heading_not_within_tolerance_guard,
    heading_within_tolerance_guard,
    los_clear_to_waypoint_guard,
    unsafe_conditions_guard,
    virtual_waypoints_guard,
    waypoint_reached_guard
)
from hybrid_automaton import RuntimeContext as _RuntimeContext
import pytest
from unittest.mock import MagicMock
import numpy as np

_ContinuousState = _RuntimeContext.ContinuousState
_AuxiliaryState = _RuntimeContext.AuxiliaryState


@pytest.mark.parametrize(
    "agent_pose, waypoint, expected_eval",
    [
        (np.array([]), [], False),
        (np.array([]), [], True),
        (np.array([]), [], False)
    ],
    ids=[
        "Test 1: Agent within tolerance",
        "Test 2: Waypoint outside tolerance",
        "Test 3: Agent & Waypoint are at the same position"
    ]
)
def test_heading_not_within_tolerance_guard(): 
    ...

@pytest.mark.parametrize(
    "agent_pose, waypoints, expected_eval",
    [
        (np.array([]), [], False),
        (np.array([]), [], True),
        (np.array([]), [], False)
    ],
    ids=[
        "Test 1: Agent outside heading tolerance to waypoint",
        "Test 2: Waypoint within heading tolerance of the agent",
        "Test 3: Agent & the waypoint are in the same position"
    ]
)
def test_heading_within_tolerance_guard(agent_state, waypoints, expected_eval): 
    ... 
    
@pytest.mark.parametrize(
    "agent_pose, waypoint, unsafe_region, configuration, expected_eval",
    [
       (np.array([]), ), 
    ],
    ids=[
        "Test 1: No vertices in the unsafe region, therefore los clear to waypoint",
        "Test 2: Vertices in the unsafe safe not intercepting the current los to the waypoint",
        "Test 3: Vertices in the unsafe region interceptin the current waypoint",
        ""
    ]
)
def test_los_clear_to_waypoint_guard(agent_pose, waypoint, unsafe_region, configuration, expected_eval): 
    mock_continuous_state = MagicMock(spec=_ContinuousState)
    mock_continuous_state.latest.return_value = agent_pose
    mock_waypoint_auxiliary_state = MagicMock(spec=_AuxiliaryState)
    mock_unsafe_region_auxiliary_state = MagicMock(spec=_AuxiliaryState) 

    mock_runtime_context = MagicMock(spec=_RuntimeContext)
    mock_runtime_context.auxiliary_state = {
        "unsafe_region": mock_unsafe_region_auxiliary_state,
        "waypoints": mock_waypoint_auxiliary_state
    } 

    

    
@pytest.mark.parametrize(
    "agent_state, waypoints, configuration, expected_eval",
    [
       (), 
    ],
    ids=[
        "",
    ]
)
def test_unsafe_conditions_guard():
  ... 
    
@pytest.mark.parametrize(
    "agent_position, waypoints, acceptance_radius",
    [
        ([0.0, 0.0], [[20.0, 20.0]], 10.0)
    ],
    ids=[
        "Test 1: waypoint outside of acceptance radius"
    ]
)
def waypoint_reached_guard(agent_position, waypoints, acceptance_radius):
    ctx: RuntimeContext = MagicMock(spec=RuntimeContext)
    ContinuousState = RuntimeContext.ContinuousState
    AuxiliaryState = RuntimeContext.AuxiliaryState
    mock_configuration = {
        "acceptance_radius": acceptance_radius
    }
    ctx.configuration = mock_configuration
    mock_continuous_state: ContinuousState = MagicMock(spec=ContinuousState)
    mock_continuous_state.latest.return_value = []
    ctx.continuous_state = mock_continuous_state
    
    mock_auxiliary_state = {
        "waypoints": MagicMock(spec=AuxiliaryState)
    }
    ctx.auxiliary_state = mock_auxiliary_state 
    ctx.axuliiary_state['waypoints'].latest.return_value = waypoints
    
    guard_eval: bool = waypoint_reached_guard(ctx) 
   
    
if __name__ == '__main__': 
    pytest.main([__file__])