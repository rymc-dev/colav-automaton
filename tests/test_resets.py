import os 
import sys 

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from hybrid_automaton import RuntimeContext
from unittest.mock import MagicMock
from colav_automaton.resets import generate_new_virtual_waypoint
from colav_automaton.resets import pop_virtual_waypoint
import numpy as np
import numpy.testing as npt


@pytest.mark.parametrize(
    "agent_pos, risk_region_vertices, config, expected_vw_point",
    [
        ([0.0,0.0, 0.0], [[30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]], {}, [30.0, 20.0]), 
        ([0.0,0.0, 0.0], [[30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]], {'longitudinal_offset_distance': 5.0, 'lateral_offset_distance': 5.0}, [36.93375, 18.61325]), 
        
    ],
    ids=[ 
         "Test Case 1: Agent at origin, risk region vertices form a polygon, expected virtual waypoint is the rightmost visible vertex, no config.",
         "Test Case 2: Agent at origin, risk region vertices form a polygon, expected virtual waypoint is the rightmost visible vertex, with config offsets."
    ]
)
def test_generate_new_virtual_waypoint(agent_pos, risk_region_vertices, config, expected_vw_point): 
    """
    Docstring for test_generate_new_virtual_waypoint
    
    :param agent_pos: Description
    :type agent_pos: list[float]
    :param risk_region_vertices: Description
    :type risk_region_vertices: list[list[float]]
    :param config: Description
    :type config: dict[str, float]
    :param expected_vw_point: Description
    :type expected_vw_point: list[float]
    """
    ctx = MagicMock(spec=RuntimeContext)

    from hybrid_automaton._runtime import _Runtime
    ContinuousState = _Runtime.Context.ContinuousState
    AuxiliaryState = _Runtime.Context.AuxiliaryState

    continuous_state = MagicMock(spec=ContinuousState) 
    continuous_state.latest.return_value = agent_pos
    ctx.continuous_state = continuous_state

    risk_region_vertices_aux_state = MagicMock(spec=AuxiliaryState)
    risk_region_vertices_aux_state.latest.return_value = risk_region_vertices
    waypoints_aux_state = MagicMock(spec=AuxiliaryState)
    waypoints_aux_state.latest.return_value = []

    ctx.auxiliary_states = {
        'unsafe_region': risk_region_vertices_aux_state,
        'waypoints': waypoints_aux_state
    }  
    ctx.configuration = config

    updated_ctx: RuntimeContext = generate_new_virtual_waypoint(ctx) 

    called_arg = waypoints_aux_state.add.call_args[0][0]
    npt.assert_allclose(called_arg, np.array(expected_vw_point), atol=1e-6)
    assert updated_ctx is ctx, "Expected the same context object to be returned after reset execution."


@pytest.mark.parametrize(
    "waypoints, should_raise",
    [
        ([], True),
        ([[10.0, 10.0]], True),
        ([[10.0, 10.0], [20.0, 20.0]], False),
    ],
)
def test_pop_virtual_waypoint(waypoints, should_raise):
    ctx = MagicMock(spec=RuntimeContext)

    from hybrid_automaton._runtime import _Runtime

    AuxiliaryState = _Runtime.Context.AuxiliaryState

    # Initialize with dummy, then manually add each waypoint
    waypoints_aux = AuxiliaryState(
        name="waypoints",
        aux0=None,
        aux_buffer_len=10,
        expected_update_hz=1,
    )

    # Clear the initial None entry
    waypoints_aux.aux_buffer.clear()
    waypoints_aux.aux_update_stamps.clear()

    for wp in waypoints:
        waypoints_aux.add(wp)

    ctx.auxiliary_states = {"waypoints": waypoints_aux}

    if should_raise:
        with pytest.raises(IndexError):
            pop_virtual_waypoint(ctx=ctx)
    else:
        before = len(waypoints_aux.aux_buffer)
        out = pop_virtual_waypoint(ctx=ctx)
        after = len(out.auxiliary_states["waypoints"].aux_buffer)
        assert after == before - 1


if __name__ == '__main__':
    pytest.main([__file__])
