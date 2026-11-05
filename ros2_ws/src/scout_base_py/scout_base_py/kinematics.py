"""Algorithms for scout base kinematics."""

import math


def twist_to_pwm(v: float, w: float, track_width: float, v_max: float,
                 pwm_min: int) -> tuple[int, int]:
    """
    Convert a Twist message to left and right PWM values.

    Args:
    ----
    v: Linear velocity (m/s).
    w: Angular velocity (rad/s).
    track_width: Distance between the wheels (m).
    v_max: Maximum linear velocity (m/s).
    pwm_min: Minimum PWM value (0-255).

    """
    # Calculate the left and right wheel velocities (m/s)
    v_left = v - (w * track_width / 2)
    v_right = v + (w * track_width / 2)

    # Convert to PWM values
    pwm_left = int((v_left / v_max) * 255)
    pwm_right = int((v_right / v_max) * 255)

    # Clamp to [-255, 255]
    pwm_left = max(-255, min(255, pwm_left))
    pwm_right = max(-255, min(255, pwm_right))

    # Ensure minimum PWM value is respected
    if abs(pwm_left) < pwm_min and pwm_left != 0:
        pwm_left = pwm_min if pwm_left > 0 else -pwm_min
    if abs(pwm_right) < pwm_min and pwm_right != 0:
        pwm_right = pwm_min if pwm_right > 0 else -pwm_min

    return pwm_left, pwm_right


def pan_rad_to_servo_deg(rad: float, pan_sign: int, pan_min: float,
                         pan_max: float) -> int:
    """Convert a pan angle in radians to a servo position in degrees."""
    rad = max(pan_min, min(pan_max, rad))
    deg = round(90 + pan_sign * math.degrees(rad))
    return int(deg)


def servo_deg_to_pan_rad(deg: int, pan_sign: int) -> float:
    """Convert a servo position in degrees to a pan angle in radians."""
    # Apply the pan sign (flip the direction if needed)
    deg -= 90
    deg *= pan_sign

    # Convert degrees to radians
    rad = math.radians(deg)

    return rad
