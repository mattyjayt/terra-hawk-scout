"""
Pan tracking control law: pure functions.

no ROS (tested in test/test_tracking.py).
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Target:
    """One detected person, in image pixels."""

    track_id: str    # tracker id ('' when tracking is off)
    cx: float        # px, box centre x in the original image
    cy: float        # px
    score: float     # 0..1


@dataclass(frozen=True)
class PanParams:
    """Tuning of the pan controller (node parameters end up here)."""

    kp: float = 3.0
    deadband: float = math.radians(2.0)
    max_rate: float = 1.5
    pan_min: float = -1.5708
    pan_max: float = 1.5708
    use_gyro_ff: bool = True


def fx_from_hfov(width: int, hfov_rad: float) -> float:
    """
    Return the focal length in px.

    from the image width and the horizontal FOV.
    """
    return (width/2)/math.tan(hfov_rad/2)


def select_target(targets: list[Target], last_id: str | None) -> Target | None:
    """
    Return the target to follow: keep last_id.

    if present, else the highest score.
    """
    if not targets:
        return None
    if last_id:
        for target in targets:
            if target.track_id == last_id:
                return target          # stick with the person we follow
    return max(targets, key=lambda t: t.score)


def bearing_error(cx: float, cx0: float, fx: float) -> float:
    """
    Return the bearing of the target in rad.

    positive when it is RIGHT of cx0.
    """
    return math.atan((cx - cx0)/fx)


def yaw_rate_in_base(q: tuple[float, float, float, float],
                     w: tuple[float, float, float]) -> float:
    """
    Return the z component of angular velocity w.

    after rotating it by q = (x, y, z, w).
    """
    x, y, z, qw = q          # quaternion of base_link <- imu_link
    wx, wy, wz = w           # gyro in the IMU's own axes (rad/s)
    # third row of the rotation matrix R(q): how much of each IMU axis
    # points along the robot's z (up) axis
    r20 = 2.0 * (x * z - qw * y)
    r21 = 2.0 * (y * z + qw * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return r20 * wx + r21 * wy + r22 * wz


def step_toward(value: float, target: float, max_step: float) -> float:
    """Move value toward target by at most max_step, never past it."""
    if abs(target - value) <= max_step:
        return target                  # close enough: land exactly on it
    if target > value:
        return value + max_step
    return value - max_step


def next_pan(pan_now: float, err: float,
             gyro_z: float, dt: float, p: PanParams) -> float:
    """Return the next pan command in rad (REP-103: + = left)."""
    e = 0.0 if abs(err) < p.deadband else err
    rate = -p.kp * e
    if p.use_gyro_ff:
        rate -= gyro_z
    rate = np.clip(rate, -p.max_rate, p.max_rate)
    return np.clip(pan_now + rate * dt, p.pan_min, p.pan_max)
