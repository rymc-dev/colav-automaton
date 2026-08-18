import os 
import sys 

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from hybrid_automaton import RuntimeContext, ContinuousState, AuxiliaryState
from unittest.mock import MagicMock
from colav_automaton.resets import generate_new_virtual_waypoint
from colav_automaton.resets import pop_virtual_waypoint
import numpy as np
import numpy.testing as npt


@pytest.mark.parametrize(
    "agent_pos, risk_region_vertices, goal, config, maneuver_bias, expected_vw_point",
    [
        ([0.0,0.0, 0.0], [[30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]], [100.0, 0.0], {}, None, [30.0, 20.0]),
        ([0.0,0.0, 0.0], [[30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]], [100.0, 0.0], {'longitudinal_offset_distance': 5.0, 'lateral_offset_distance': 5.0}, None, [30.0, 15.0]),
        # Port-side vertex is the geometrically "easier" one here (much
        # smaller heading change for a similar distance) - no maneuver_bias,
        # so the ease-of-navigation heuristic should pick port over the
        # default starboard tie-break.
        ([0.0, 0.0, 0.0], [[5.0, -50.0], [50.0, 5.0], [27.0, -20.0]], [100.0, 0.0], {'lateral_offset_distance': 10.0}, None, [50.0, 15.0]),
        # maneuver_bias forces starboard (the geometrically "harder" side
        # here) at full urgency - COLREGs compliance overrides the
        # ease-of-navigation default, and the offset is doubled (1+urgency).
        ([0.0, 0.0, 0.0], [[5.0, -50.0], [50.0, 5.0], [27.0, -20.0]], [100.0, 0.0], {'lateral_offset_distance': 10.0}, {'side': 'starboard', 'urgency': 1.0}, [5.0, -70.0]),
        # maneuver_bias confirms port (agreeing with the geometric default
        # here) at partial urgency - offset scaled by (1+urgency).
        ([0.0, 0.0, 0.0], [[5.0, -50.0], [50.0, 5.0], [27.0, -20.0]], [100.0, 0.0], {'lateral_offset_distance': 10.0}, {'side': 'port', 'urgency': 0.5}, [50.0, 20.0]),
    ],
    ids=[
         "Test Case 1: Agent at origin, risk region vertices form a polygon, expected virtual waypoint is the rightmost visible vertex, no config.",
         "Test Case 2: Agent at origin, risk region vertices form a polygon, expected virtual waypoint is the rightmost visible vertex, with config offsets.",
         "Test Case 3: No maneuver_bias - port side chosen because it's geometrically easier to navigate to.",
         "Test Case 4: maneuver_bias overrides the geometric default to starboard and scales the offset by urgency.",
         "Test Case 5: maneuver_bias agrees with the geometric default (port) and scales the offset by urgency.",
    ]
)
def test_generate_new_virtual_waypoint(agent_pos, risk_region_vertices, goal, config, maneuver_bias, expected_vw_point):
    """
    :param agent_pos: agent's [x, y, heading]
    :type agent_pos: list[float]
    :param risk_region_vertices: unsafe region polygon vertices
    :type risk_region_vertices: list[list[float]]
    :param goal: current target waypoint - used to pick which side (left/right)
        of the visible vertex to offset the new virtual waypoint toward
    :type goal: list[float]
    :param config: automaton configuration (offset distances)
    :type config: dict[str, float]
    :param maneuver_bias: optional COLREGs maneuver bias
        ({"side": "port"|"starboard", "urgency": float}) as would be
        produced by classification.Maneuver.as_bias()
    :type maneuver_bias: dict | None
    :param expected_vw_point: expected virtual waypoint pushed via .add()
    :type expected_vw_point: list[float]
    """
    ctx = MagicMock(spec=RuntimeContext)

    continuous_state = MagicMock(spec=ContinuousState)
    continuous_state.latest.return_value = agent_pos
    ctx.continuous_state = continuous_state

    risk_region_vertices_aux_state = MagicMock(spec=AuxiliaryState)
    risk_region_vertices_aux_state.latest.return_value = risk_region_vertices
    waypoints_aux_state = MagicMock(spec=AuxiliaryState)
    waypoints_aux_state.latest.return_value = goal
    # Only the goal on the buffer (no virtual waypoint queued yet) - the
    # reset should add without popping first.
    waypoints_aux_state.aux_buffer = [goal]

    ctx.auxiliary_states = {
        'unsafe_region': risk_region_vertices_aux_state,
        'waypoints': waypoints_aux_state
    }
    if maneuver_bias is not None:
        maneuver_bias_aux_state = MagicMock(spec=AuxiliaryState)
        maneuver_bias_aux_state.latest.return_value = maneuver_bias
        ctx.auxiliary_states['maneuver_bias'] = maneuver_bias_aux_state
    ctx.configuration = config

    updated_ctx: RuntimeContext = generate_new_virtual_waypoint(ctx)

    called_arg = waypoints_aux_state.add.call_args[0][0]
    npt.assert_allclose(called_arg, np.array(expected_vw_point), atol=1e-6)
    waypoints_aux_state.pop.assert_not_called()
    assert updated_ctx is ctx, "Expected the same context object to be returned after reset execution."


def test_generate_new_virtual_waypoint_replaces_existing_virtual_waypoint():
    """If a virtual waypoint is already queued ahead of the goal, a new
    reroute should replace it (pop then add) rather than stacking another
    one on top - otherwise repeated reroutes pile up a detour that has to
    be unwound leg by leg, doubling back on itself."""
    ctx = MagicMock(spec=RuntimeContext)

    continuous_state = MagicMock(spec=ContinuousState)
    continuous_state.latest.return_value = [0.0, 0.0, 0.0]
    ctx.continuous_state = continuous_state

    risk_region_vertices_aux_state = MagicMock(spec=AuxiliaryState)
    risk_region_vertices_aux_state.latest.return_value = [
        [30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]
    ]
    waypoints_aux_state = MagicMock(spec=AuxiliaryState)
    current_virtual_waypoint = [10.0, 5.0]
    waypoints_aux_state.latest.return_value = current_virtual_waypoint
    # A virtual waypoint is already queued in front of the goal.
    waypoints_aux_state.aux_buffer = [current_virtual_waypoint, [100.0, 0.0]]

    manager = MagicMock()
    manager.attach_mock(waypoints_aux_state.pop, 'pop')
    manager.attach_mock(waypoints_aux_state.add, 'add')

    ctx.auxiliary_states = {
        'unsafe_region': risk_region_vertices_aux_state,
        'waypoints': waypoints_aux_state
    }
    ctx.configuration = {}

    generate_new_virtual_waypoint(ctx)

    waypoints_aux_state.pop.assert_called_once()
    waypoints_aux_state.add.assert_called_once()
    assert [c[0] for c in manager.mock_calls] == ['pop', 'add'], (
        "Expected the stale virtual waypoint to be popped before the new one is added."
    )


def test_generate_new_virtual_waypoint_raises_with_no_current_waypoint():
    """If there's no current waypoint to offset relative to, fail loudly
    rather than silently producing a garbage offset."""
    ctx = MagicMock(spec=RuntimeContext)

    continuous_state = MagicMock(spec=ContinuousState)
    continuous_state.latest.return_value = [0.0, 0.0, 0.0]
    ctx.continuous_state = continuous_state

    risk_region_vertices_aux_state = MagicMock(spec=AuxiliaryState)
    risk_region_vertices_aux_state.latest.return_value = [
        [30.0, 20.0], [25.0, 30.0], [27.0, 40.0], [32.0, 25.0], [30.0, 20.0]
    ]
    waypoints_aux_state = MagicMock(spec=AuxiliaryState)
    waypoints_aux_state.latest.return_value = []

    ctx.auxiliary_states = {
        'unsafe_region': risk_region_vertices_aux_state,
        'waypoints': waypoints_aux_state
    }
    ctx.configuration = {}

    with pytest.raises(ValueError, match="no current waypoint"):
        generate_new_virtual_waypoint(ctx)

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
