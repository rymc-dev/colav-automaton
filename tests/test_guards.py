import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from colav_automaton.guards import (
    los_clear_to_waypoint_guard,
    unsafe_conditions_guard,
    safe_conditions_guard,
    virtual_waypoints_guard,
    waypoint_reached_guard,
)
from hybrid_automaton import RuntimeContext as _RuntimeContext
from hybrid_automaton import ContinuousState as _ContinuousState
from hybrid_automaton import AuxiliaryState as _AuxiliaryState
import pytest
from unittest.mock import MagicMock
import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_ctx(
    agent_pose: np.ndarray | None = None,
    waypoints: list | None = None,
    unsafe_region: list | None = None,
    configuration: dict | None = None,
) -> _RuntimeContext:
    """Build a minimal RuntimeContext mock wired up the way the guards expect."""
    ctx = MagicMock(spec=_RuntimeContext)

    # continuous_state
    mock_cs = MagicMock(spec=_ContinuousState)
    mock_cs.latest.return_value = agent_pose if agent_pose is not None else np.zeros(5)
    ctx.continuous_state = mock_cs

    # auxiliary_states – guards use ctx.auxiliary_states (plural)
    mock_waypoints_aux = MagicMock(spec=_AuxiliaryState)
    mock_waypoints_aux.latest.return_value = waypoints if waypoints is not None else []
    mock_waypoints_aux.aux_buffer = waypoints if waypoints is not None else []

    mock_unsafe_aux = MagicMock(spec=_AuxiliaryState)
    mock_unsafe_aux.latest.return_value = unsafe_region if unsafe_region is not None else []

    ctx.auxiliary_states = {
        "waypoints": mock_waypoints_aux,
        "unsafe_region": mock_unsafe_aux,
    }

    ctx.configuration = configuration if configuration is not None else {}

    return ctx


# ---------------------------------------------------------------------------
# los_clear_to_waypoint_guard
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "agent_pose, waypoint, unsafe_region_vertices, configuration, expected_eval",
    [
        # No vertices → Polygon is degenerate, no intersection → guard = False
        (
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
            [100.0, 0.0],
            [],
            {"los_distance_threshold": 50.0},
            False,
        ),
        # Unsafe polygon exists but lies far off the LOS → guard = False
        (
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
            [10.0, 0.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            {"los_distance_threshold": 50.0},
            False,
        ),
        # Unsafe polygon sits directly on the LOS and within threshold → guard = True
        (
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
            [20.0, 0.0],
            [
                [5.0, -1.0],
                [8.0, -1.0],
                [8.0,  1.0],
                [5.0,  1.0],
            ],
            {"los_distance_threshold": 50.0},
            True,
        ),
        # Unsafe polygon on LOS but beyond threshold distance → guard = False
        (
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
            [200.0, 0.0],
            [
                [150.0, -1.0],
                [160.0, -1.0],
                [160.0,  1.0],
                [150.0,  1.0],
            ],
            {"los_distance_threshold": 10.0},
            False,
        ),
    ],
    ids=[
        "Test 1: No unsafe region vertices → LOS clear → False",
        "Test 2: Unsafe region away from LOS → LOS clear → False",
        "Test 3: Unsafe region on LOS within threshold → guard True",
        "Test 4: Unsafe region on LOS beyond threshold → guard False",
    ],
)
def test_los_clear_to_waypoint_guard(
    agent_pose, waypoint, unsafe_region_vertices, configuration, expected_eval
):
    ctx = _make_ctx(
        agent_pose=agent_pose,
        waypoints=waypoint,
        unsafe_region=unsafe_region_vertices,
        configuration=configuration,
    )
    result = los_clear_to_waypoint_guard(ctx)
    assert result == expected_eval


# ---------------------------------------------------------------------------
# virtual_waypoints_guard
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "waypoints, expected_eval",
    [
        # Only one waypoint → no virtual waypoints → False
        ([[10.0, 10.0]], False),
        # Two waypoints → virtual waypoints present → True
        ([[10.0, 10.0], [20.0, 20.0]], True),
        # Many waypoints → True
        ([[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]], True),
    ],
    ids=[
        "Test 1: Single waypoint → no virtual waypoints → False",
        "Test 2: Two waypoints → virtual waypoints present → True",
        "Test 3: Many waypoints → True",
    ],
)
def test_virtual_waypoints_guard(waypoints, expected_eval):
    ctx = _make_ctx(waypoints=waypoints)
    result = virtual_waypoints_guard(ctx)
    assert result == expected_eval


def test_virtual_waypoints_guard_raises_on_missing_waypoints():
    ctx = _make_ctx()
    ctx.auxiliary_states = None
    with pytest.raises(ValueError, match="invalid aux_x"):
        virtual_waypoints_guard(ctx)


# ---------------------------------------------------------------------------
# waypoint_reached_guard
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "agent_position, waypoint, acceptance_radius, expected_eval",
    [
        # Agent far from waypoint → not reached → False
        ([0.0, 0.0], [20.0, 20.0], 10.0, False),
        # Agent exactly at waypoint → reached → True
        ([5.0, 5.0], [5.0, 5.0], 10.0, True),
        # Agent within acceptance radius → reached → True
        ([0.0, 0.0], [5.0, 0.0], 10.0, True),
        # Agent just outside acceptance radius → not reached → False
        ([0.0, 0.0], [10.1, 0.0], 10.0, False),
    ],
    ids=[
        "Test 1: Waypoint outside acceptance radius → False",
        "Test 2: Agent at exact waypoint position → True",
        "Test 3: Agent within acceptance radius → True",
        "Test 4: Agent just outside acceptance radius → False",
    ],
)
def test_waypoint_reached_guard(agent_position, waypoint, acceptance_radius, expected_eval):
    agent_pose = np.array([agent_position[0], agent_position[1], 0.0, 0.0, 0.0])
    ctx = _make_ctx(
        agent_pose=agent_pose,
        waypoints=waypoint,
        configuration={"acceptance_radius": acceptance_radius},
    )
    result = waypoint_reached_guard(ctx)
    assert result == expected_eval


def test_waypoint_reached_guard_uses_default_acceptance_radius():
    """Guard defaults to 20.0 m when acceptance_radius is absent from configuration."""
    agent_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    ctx = _make_ctx(
        agent_pose=agent_pose,
        waypoints=[15.0, 0.0],   # 15 m away, inside default 20 m radius
        configuration={},         # no acceptance_radius key
    )
    result = waypoint_reached_guard(ctx)
    assert result is True


# ---------------------------------------------------------------------------
# unsafe_conditions_guard
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "agent_position, unsafe_region_vertices, agent_safety_radius, expected_eval",
    [
        # Agent circle completely clear of unsafe region → False
        (
            [0.0, 0.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            5.0,
            False,
        ),
        # Agent sitting inside the unsafe region → True
        (
            [105.0, 105.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            1.0,
            True,
        ),
        # Agent outside but safety radius overlaps the unsafe region → True
        (
            [98.0, 105.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            5.0,
            True,
        ),
    ],
    ids=[
        "Test 1: Agent clear of unsafe region → False",
        "Test 2: Agent inside unsafe region → True",
        "Test 3: Agent safety radius overlaps unsafe region → True",
    ],
)
def test_unsafe_conditions_guard(
    agent_position, unsafe_region_vertices, agent_safety_radius, expected_eval
):
    agent_pose = np.array([agent_position[0], agent_position[1], 0.0, 0.0, 0.0])
    configuration = {"agent_safety_radius": agent_safety_radius}
    ctx = _make_ctx(
        agent_pose=agent_pose,
        unsafe_region=unsafe_region_vertices,
        configuration=configuration,
    )
    result = unsafe_conditions_guard(ctx)
    assert result == expected_eval


# ---------------------------------------------------------------------------
# safe_conditions_guard (inverse of unsafe_conditions_guard)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "agent_position, unsafe_region_vertices, agent_safety_radius, expected_eval",
    [
        # Agent circle completely clear of unsafe region → conditions safe → True
        (
            [0.0, 0.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            5.0,
            True,
        ),
        # Agent sitting inside the unsafe region → still unsafe → False
        (
            [105.0, 105.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            1.0,
            False,
        ),
        # Agent outside but safety radius overlaps the unsafe region → still unsafe → False
        (
            [98.0, 105.0],
            [[100.0, 100.0], [110.0, 100.0], [110.0, 110.0], [100.0, 110.0]],
            5.0,
            False,
        ),
    ],
    ids=[
        "Test 1: Agent clear of unsafe region → True",
        "Test 2: Agent inside unsafe region → False",
        "Test 3: Agent safety radius overlaps unsafe region → False",
    ],
)
def test_safe_conditions_guard(
    agent_position, unsafe_region_vertices, agent_safety_radius, expected_eval
):
    agent_pose = np.array([agent_position[0], agent_position[1], 0.0, 0.0, 0.0])
    configuration = {"agent_safety_radius": agent_safety_radius}
    ctx = _make_ctx(
        agent_pose=agent_pose,
        unsafe_region=unsafe_region_vertices,
        configuration=configuration,
    )
    result = safe_conditions_guard(ctx)
    assert result == expected_eval


if __name__ == "__main__":
    pytest.main([__file__, "-v"])