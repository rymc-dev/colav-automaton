import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import pytest
from hybrid_automaton import RuntimeContext
from unittest.mock import MagicMock
from colav_automaton.invariants import failing_invariant, fallback_recoverable_invariant


def test_failing_invariant_always_false():
    """failing_invariant is used to make Waypoint_Reached unable to idle -
    it must always have an active transition guard."""
    ctx = MagicMock(spec=RuntimeContext)
    assert failing_invariant(ctx) is False


@pytest.mark.parametrize(
    "elapsed, timeout, config, expected",
    [
        (5.0, 30.0, {"fallback_timeout": 30.0}, True),
        (29.999, 30.0, {"fallback_timeout": 30.0}, True),
        (30.0, 30.0, {"fallback_timeout": 30.0}, False),
        (45.0, 30.0, {"fallback_timeout": 30.0}, False),
        (5.0, 30.0, {}, True),   # no fallback_timeout configured -> default 30.0s, 5s is under it
    ],
    ids=[
        "well under timeout holds",
        "just under timeout holds",
        "exactly at timeout violates",
        "well over timeout violates",
        "missing config falls back to 30.0s default",
    ],
)
def test_fallback_recoverable_invariant(elapsed, timeout, config, expected):
    # @invariant's __call__ requires isinstance(ctx, Context), so this does
    # need spec=RuntimeContext - but `clock` is an instance attribute
    # Context sets in __init__, not a class attribute, so the spec doesn't
    # know about it and won't let us read it until we've explicitly set it
    # (assignment is always allowed on a Mock, spec only gates unset reads).
    ctx = MagicMock(spec=RuntimeContext)
    ctx.configuration = config
    ctx.clock = MagicMock()
    ctx.clock.get_time_elapsed_since_transition.return_value = elapsed
    assert fallback_recoverable_invariant(ctx) is expected


if __name__ == '__main__':
    pytest.main([__file__])
