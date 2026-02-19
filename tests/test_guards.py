from colav_automaton.guards import (
    heading_not_within_tolerance_guard,
    heading_within_tolerance_guard,
    los_clear_to_waypoint_guard,
    unsafe_conditions_guard,
    virtual_waypoints_guard,
    waypoint_reached_guard
)
from hybrid_automaton import RuntimeContext
import pytest
from unittest.mock import MagicMock


@pytest.mark.parametrize(
    "",
    [
        
    ],
    ids=[
        
    ]
)
def test_heading_not_within_tolerance_guard(): 
    ...

@pytest.mark.parametrize(
    "",
    [
        
    ],
    ids=[
        
    ]
)
def test_heading_within_tolerance_guard(): 
    ... 

@pytest.mark.parametrize(
    "",
    [
        
    ],
    ids=[
        
    ]
)
def test_los_clear_to_waypoint_guard(): 
    ...
    
@pytest.mark.parametrize(
    "",
    [
        
    ],
    ids=[
        
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