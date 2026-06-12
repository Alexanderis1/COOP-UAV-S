"""P0-6: RngRegistry — name-keyed independent RNG streams (DESIGN_REVIEW 5.1).

A stream is a pure function of (run seed, stream name): creation order,
sibling-stream traffic, and consumer count must not shift anyone's draws.
That is the property that makes determinism survive executor reordering.
"""

from __future__ import annotations

import numpy as np
import pytest

from coopuavs.core.rng import RngRegistry


def test_stream_is_pure_function_of_seed_and_name():
    a = RngRegistry(7).stream("weather").random(8)
    b = RngRegistry(7).stream("weather").random(8)
    assert np.array_equal(a, b)


def test_streams_differ_by_name_and_seed():
    base = RngRegistry(7).stream("weather").random(8)
    other_name = RngRegistry(7).stream("comms").random(8)
    other_seed = RngRegistry(8).stream("weather").random(8)
    assert not np.array_equal(base, other_name)
    assert not np.array_equal(base, other_seed)


def test_creation_order_is_irrelevant():
    r1 = RngRegistry(3)
    x_first = r1.stream("x").random(8)
    y_second = r1.stream("y").random(8)

    r2 = RngRegistry(3)
    y_first = r2.stream("y").random(8)
    x_second = r2.stream("x").random(8)

    assert np.array_equal(x_first, x_second)
    assert np.array_equal(y_second, y_first)


def test_sibling_traffic_does_not_shift_a_stream():
    quiet = RngRegistry(5)
    expected = quiet.stream("victim").random(8)

    noisy = RngRegistry(5)
    noisy.stream("chatterbox").random(10_000)
    got = noisy.stream("victim").random(8)
    assert np.array_equal(expected, got)


def test_stream_is_stateful_and_cached():
    reg = RngRegistry(1)
    s = reg.stream("x")
    assert reg.stream("x") is s
    first = s.random(4)
    assert not np.array_equal(first, reg.stream("x").random(4))


def test_negative_seed_rejected():
    with pytest.raises(ValueError):
        RngRegistry(-1)
