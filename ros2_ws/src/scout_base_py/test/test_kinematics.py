"""Tests for scout base kinematics."""
from scout_base_py.kinematics import (pan_rad_to_servo_deg,
                                      servo_deg_to_pan_rad,
                                      twist_to_pwm)


def test_twist_to_pwm():
    """Test the twist_to_pwm function."""
    v = 0.0  # m/s
    w = 0.0  # rad/s
    track_width = 0.15  # m
    v_max = 0.5  # m/s
    pwm_min = 60

    pwm_left, pwm_right = twist_to_pwm(v, w, track_width, v_max, pwm_min)

    assert isinstance(pwm_left, int)
    assert isinstance(pwm_right, int)
    assert pwm_left == 0
    assert pwm_right == 0

    v = 0.25
    w = 0

    pwm_left, pwm_right = twist_to_pwm(v, w, track_width, v_max, pwm_min)

    assert pwm_left == pwm_right
    assert pwm_left > 0

    v = 0
    w = 1.0

    pwm_left, pwm_right = twist_to_pwm(v, w, track_width, v_max, pwm_min)

    assert pwm_left < 0
    assert pwm_right > 0


def test_pan_rad_to_servo_deg():
    """Test the pan_rad_to_servo_deg function."""
    rad = 0.0
    pan_sign = 1
    pan_min = 0.0
    pan_max = 180.0

    servo_deg = pan_rad_to_servo_deg(rad, pan_sign, pan_min, pan_max)

    assert isinstance(servo_deg, int)
    assert servo_deg == 90


def test_servo_deg_to_pan_rad():
    """Test the servo_deg_to_pan_rad function."""
    deg = 90  # degrees
    pan_sign = 1

    rad = servo_deg_to_pan_rad(deg, pan_sign)

    assert isinstance(rad, float)
    assert -3.14159 <= rad <= 3.14159
