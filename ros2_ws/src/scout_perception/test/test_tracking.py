"""
Tests for the pan control law (Lab 8.5).

Two worked examples; you write the rest.
"""
import pytest
from scout_perception.tracking import bearing_error, next_pan, PanParams

FX = 492.76
P = PanParams()


def test_bearing_error_person_right_is_positive():
    """Test bearing error."""
    # person 100 px RIGHT of centre -> +0.2003 rad (11.5 deg)
    # wrong values: -0.2003 (cx0 - cx), 100.0 (no /fx), 11.47 (degrees)
    assert bearing_error(420.0, 320.0, FX) == pytest.approx(0.2003, abs=1e-3)


def test_feed_forward_cancels_pure_base_rotation():
    """Test feed forward."""
    # person centred (err 0), base turns LEFT at 0.5 rad/s for 0.05 s
    # -> camera must turn RIGHT by 0.025 rad: pan 0.100 -> 0.075
    # wrong values: 0.125 (sign of ff flipped), 0.100 (ff not applied)
    assert next_pan(0.100, 0.0, 0.5, 0.05, P) == pytest.approx(0.075, abs=1e-6)
