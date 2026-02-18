import os 
import sys 

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from hybrid_automaton import RuntimeContext
from unittest.mock import MagicMock
from colav_automaton.invariants import is_goal_waypoint_invariant


@pytest.mark.parametrize(
    "waypoints, expected_eval",
    [
        ([], IndexError),
        ([[0.0, 0.0]], True),
        ([[0.0, 0.0], [1.0, 1.0]], False),
    ],
)
def test_is_goal_waypoint_invariant(waypoints, expected_eval):
    ctx = MagicMock(spec=RuntimeContext)

    from hybrid_automaton._runtime import _Runtime
    AuxiliaryState = _Runtime.Context.AuxiliaryState

    waypoints_aux = AuxiliaryState(
        name="waypoints",
        aux0=None,
        aux_buffer_len=10,
        expected_update_hz=1,
    )

    # Remove the initial None
    waypoints_aux.aux_buffer.clear()
    waypoints_aux.aux_update_stamps.clear()

    # Actually populate with param data
    for wp in waypoints:
        waypoints_aux.add(wp)

    ctx.auxiliary_states = {
        "waypoints": waypoints_aux
    }

    if isinstance(expected_eval, type):
        with pytest.raises(expected_eval):
            is_goal_waypoint_invariant(ctx)
    else:
        assert is_goal_waypoint_invariant(ctx) == expected_eval

        
if __name__ == '__main__':
    pytest.main([__file__])