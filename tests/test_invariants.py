import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from hybrid_automaton import RuntimeContext
from unittest.mock import MagicMock
from colav_automaton.invariants import failing_invariant


def test_failing_invariant_always_false():
    """failing_invariant is used to make Fallback/Waypoint_Reached states
    unable to idle - they must always have an active transition guard."""
    ctx = MagicMock(spec=RuntimeContext)
    assert failing_invariant(ctx) is False


if __name__ == '__main__':
    pytest.main([__file__])
