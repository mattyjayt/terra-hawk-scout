"""Load and save named map poses (waypoints.yaml)."""

import math

import yaml


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) of a rotation by yaw about +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw (rad, REP-103: + = left) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_waypoints(path: str) -> dict[str, tuple[float, float, float]]:
    """
    Read waypoints.yaml and return {name: (x, y, yaw)}.

    Args:
    ----
    path: absolute path of the yaml file.

    """
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if data.get('frame_id', 'map') != 'map':
        raise ValueError('waypoints must be in the map frame')
    return {name: (float(p['x']), float(p['y']), float(p['yaw']))
            for name, p in (data.get('waypoints') or {}).items()}


def save_waypoint(path: str, name: str,
                  x: float, y: float, yaw: float) -> None:
    """
    Add or replace one waypoint in waypoints.yaml, keeping the others.

    Args:
    ----
    path: absolute path of the yaml file (created if missing).
    name: waypoint name, e.g. 'pickup'.
    x: position in the map frame (m).
    y: position in the map frame (m).
    yaw: heading in the map frame (rad).

    """
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {}
    data['frame_id'] = 'map'
    data.setdefault('waypoints', {})
    data['waypoints'][name] = {
        'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
