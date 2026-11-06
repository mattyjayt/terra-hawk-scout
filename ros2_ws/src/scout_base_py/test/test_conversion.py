"""Test the scout_base_py.conversions module."""
from scout_base_py.conversions import (dps_to_radps, g_to_mps2, link_level,
                                       RateMeter, StaleDetector)


def test_dps_to_radps():
    """Test the dps_to_radps function."""
    dps = 180.0
    radps = dps_to_radps(dps)
    assert isinstance(radps, float)
    assert radps == 3.14159


def test_g_to_mps2():
    """Test the g_to_mps2 function."""
    g = 1.0
    mps2 = g_to_mps2(g)
    assert isinstance(mps2, float)
    assert mps2 == 9.80665


def test_stale_detector():
    """Test the StaleDetector class."""
    # IMU data = ax ay az gx gy gz
    # 9 identical values
    stale_detector = StaleDetector(limit=10)
    for _ in range(9):
        assert not stale_detector.update((0, 0, 0, 0, 0, 0))
    assert stale_detector.counter <= stale_detector.limit

    # 10 identical values
    for _ in range(10):
        assert stale_detector.update((0, 0, 0, 0, 0, 0))
    assert stale_detector.counter >= stale_detector.limit

    # 11 identical values
    assert stale_detector.update((0, 0, 0, 0, 0, 0))
    assert stale_detector.counter >= stale_detector.limit

    # Reset counter when a non-zero value is seen
    assert not stale_detector.update((0, 1, 0, 0, 0, 0))
    assert stale_detector.counter == 1


def test_rate_meter():
    """Test the RateMeter class."""
    rate_meter = RateMeter(window_s=1.0)
    for i in range(20):
        rate_meter.tick(i * 0.05)
    assert rate_meter.rate(1.0) == 20.0

    # no ticks at all
    rate_meter_empty = RateMeter(window_s=1.0)
    assert rate_meter_empty.rate(1.0) == 0.0

    # a tick exactly on the window boundary
    rate_meter_boundary = RateMeter(window_s=1.0)
    rate_meter_boundary.tick(0.0)
    assert rate_meter_boundary.rate(1.0) == 1.0

    # asking for rate() 2 s after the last tick
    rate_meter_late = RateMeter(window_s=1.0)
    rate_meter_late.tick(0.0)
    assert rate_meter_late.rate(2.0) == 0.0


def test_link_level():
    """Test the link_level function."""
    # Case 1: rate 19Hz -> Ok
    assert link_level(True, 19.0, 20.0, False, 0.05, 3.0, 2.0) == 0

    # Case 2: rate 15Hz -> Warning
    assert link_level(True, 15.0, 20.0, False, 0.05, 3.0, 2.0) == 1

    # Case 3: link down -> Error
    assert link_level(False, 0.0, 20.0, False, 0.05, 3.0, 2.0) == 2

    # Case 4: rate 19Hz, IMU frozen -> Error
    assert link_level(True, 19.0, 20.0, True, 0.05, 3.0, 2.0) == 2

    # Case 5: rate 19Hz, no I line for 0.6s -> Error
    assert link_level(True, 19.0, 20.0, False, 0.6, 3.0, 2.0) == 2


def test_stale_detector_real_frozen_sample():
    """The frozen sample seen on the robot (3 Nov) trips on the 10th repeat."""
    frozen = (0.054, 0.0, 1.061, 0.24, 0.57, 0.03)
    sd = StaleDetector(limit=10)
    results = [sd.update(frozen) for _ in range(12)]
    assert results[:9] == [False] * 9
    assert results[9:] == [True] * 3


def test_stale_detector_live_noise_never_trips():
    """Changing values (a live sensor) never count as stale."""
    sd = StaleDetector(limit=10)
    for i in range(100):
        assert not sd.update((0.05, 0.0, 1.06 + i * 1e-3, 0.1, 0.5, 0.0))


def test_stale_detector_recovers():
    """One new value after a freeze clears the flag."""
    sd = StaleDetector(limit=3)
    for _ in range(5):
        sd.update((1, 2, 3))
    assert sd.update((1, 2, 4)) is False


def test_rate_meter_sliding_across_window_boundary():
    """A steady 20 Hz stream reads ~20 Hz at any moment, not only at 1 s."""
    rm = RateMeter(window_s=1.0)
    t = 0.0
    readings = []
    for i in range(200):          # 10 s at 20 Hz
        t = i * 0.05
        rm.tick(t)
        if t >= 1.0:
            readings.append(rm.rate(t))
    assert min(readings) >= 19.0
    assert max(readings) <= 21.0


def test_rate_meter_drops_to_zero_when_silent():
    """If the R4 goes quiet, the rate falls to 0 without new ticks."""
    rm = RateMeter(window_s=1.0)
    for i in range(20):
        rm.tick(i * 0.05)
    assert rm.rate(0.95) == 20.0
    assert rm.rate(3.0) == 0.0
