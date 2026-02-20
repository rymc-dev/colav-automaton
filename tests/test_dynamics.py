import os 
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from typing import List
from unittest.mock import MagicMock
from hybrid_automaton._runtime import _Runtime

from colav_automaton.dynamics import flow_los_heading
from colav_automaton.dynamics import constant_heading_dynamics

import numpy as np

_Runtime_Context = _Runtime.Context
_ContinuousState = _Runtime_Context.ContinuousState
_AuxiliaryState = _Runtime_Context.AuxiliaryState

import math
import numpy as np
import pytest


@pytest.mark.parametrize(
    "agent_state, config, expected",
    [
        # Test 1: Zero velocity → no motion, accelerate toward default v_des=2
        (
            np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
            {},
            np.array([0.0, 0.0, 0.0, 2.0, 0.0]),
        ),

        # Test 2: Heading = 0 → motion purely in +x
        (
            np.array([0.0, 0.0, 0.0, 3.0, 0.0]),
            {"constant_velocity": 3.0},
            np.array([3.0, 0.0, 0.0, 0.0, 0.0]),
        ),

        # Test 3: Heading = pi/2 → motion purely in +y
        (
            np.array([0.0, 0.0, math.pi / 2, 4.0, 0.0]),
            {"constant_velocity": 4.0},
            np.array([0.0, 4.0, 0.0, 0.0, 0.0]),
        ),

        # Test 4: Velocity below target → positive v_dot
        (
            np.array([0.0, 0.0, 0.0, 1.0, 0.0]),
            {"constant_velocity": 3.0, "k_v": 2.0},
            np.array([1.0, 0.0, 0.0, 4.0, 0.0]),  # v_dot = 2*(3-1)=4
        ),

        # Test 5: Velocity above target → negative v_dot
        (
            np.array([0.0, 0.0, 0.0, 5.0, 0.0]),
            {"constant_velocity": 2.0, "k_v": 1.5},
            np.array([5.0, 0.0, 0.0, -4.5, 0.0]),  # v_dot = 1.5*(2-5)=-4.5
        ),
    ],
    ids=[
        "Zero velocity accelerates to default target",
        "Heading zero gives pure x motion",
        "Heading 90deg gives pure y motion",
        "Below target velocity accelerates upward",
        "Above target velocity decelerates",
    ],
)
def test_constant_heading_dynamics(agent_state, config, expected):
    ctx = MagicMock(spec=_Runtime_Context)
    mock_continuous_state = MagicMock(spec=_ContinuousState)
    mock_continuous_state.latest.return_value = agent_state
    ctx.continuous_state = mock_continuous_state
    ctx.configuration = config
    
    result = constant_heading_dynamics(ctx)

    np.testing.assert_allclose(result, expected, atol=1e-6)

@pytest.mark.parametrize(
    "agent_state, waypoint, config, expected",
    [
        # Test 1: Waypoint straight ahead → zero heading correction
        (
            np.array([0.0, 0.0, 0.0, 2.0, 0.0]),   # facing +x
            [[10.0, 0.0]],                         # waypoint straight ahead
            {"constant_velocity": 2.0},
            np.array([2.0, 0.0, 0.0, 0.0, 0.0]),
        ),

        # Test 2: Waypoint directly behind → error = -pi (wrapped)
        (
            np.array([0.0, 0.0, 0.0, 1.0, 0.0]),   # facing +x
            [[-10.0, 0.0]],                        # behind
            {"k_theta": 1.0, "constant_velocity": 1.0},
            np.array([1.0, 0.0, -math.pi, 0.0, 0.0]),
        ),

        # Test 3: Waypoint 90° to the left → positive pi/2 correction
        (
            np.array([0.0, 0.0, 0.0, 3.0, 0.0]),   # facing +x
            [[0.0, 10.0]],                         # directly above
            {"k_theta": 1.0, "constant_velocity": 3.0},
            np.array([3.0, 0.0, math.pi / 2, 0.0, 0.0]),
        ),

        # Test 4: Heading near +pi, waypoint slightly past -pi boundary
        # Ensures wrapping works properly
        (
            np.array([0.0, 0.0, math.pi - 0.1, 1.0, 0.0]),
            [[-10.0, 0.0]],
            {"k_theta": 1.0, "constant_velocity": 1.0},
            # desired_heading = pi → small positive error ≈ 0.1
            np.array([
                1.0 * math.cos(math.pi - 0.1),
                1.0 * math.sin(math.pi - 0.1),
                0.1,
                0.0,
                0.0,
            ]),
        ),

        # Test 5: Heading gain scales correction
        (
            np.array([0.0, 0.0, 0.0, 2.0, 0.0]),
            [[0.0, 10.0]],
            {"k_theta": 2.0, "constant_velocity": 2.0},
            np.array([2.0, 0.0, math.pi, 0.0, 0.0]),  # 2 * (pi/2)
        ),
    ],
    ids=[
        "Waypoint straight ahead",
        "Waypoint directly behind",
        "Waypoint 90deg left",
        "Heading wrap across pi boundary",
        "Heading gain scaling",
    ],
)
def test_flow_los_heading(agent_state, waypoint, config, expected):

    mock_ctx = MagicMock(spec=_Runtime_Context)

    # Mock continuous state
    mock_continuous_state = MagicMock(spec=_ContinuousState)
    mock_continuous_state.latest.return_value = agent_state
    mock_ctx.continuous_state = mock_continuous_state

    # Mock auxiliary state (waypoints)
    mock_waypoints_state = MagicMock(spec=_AuxiliaryState)
    mock_waypoints_state.latest.return_value = waypoint

    mock_ctx.auxiliary_states = {
        "waypoints": mock_waypoints_state
    }

    mock_ctx.configuration = config

    result = flow_los_heading(mock_ctx)

    np.testing.assert_allclose(result, expected, atol=1e-6)
    
    
if __name__ == '__main__':
    pytest.main([__file__])