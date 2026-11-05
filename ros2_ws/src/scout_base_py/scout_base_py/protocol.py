"""Protocol definitions and parsing helpers for the Scout base bridge."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ImuSample:
    """Single inertial measurement sample from the Scout base."""

    time_ms: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    yaw_deg: float
    servo_deg: int
    batt_voltage: float


@dataclass(frozen=True)
class Ack:
    """Acknowledgement message sent by the base bridge."""

    text: str


@dataclass(frozen=True)
class WatchdogStop:
    """Sentinel indicating the robot watchdog has stopped the base."""

    pass


def parse_line(line: str) -> ImuSample | Ack | WatchdogStop | None:
    """Parse a line from the Scout base bridge (Python) protocol."""
    if line.startswith('I '):
        parts = line.split()
        if len(parts) != 11:
            return None
        try:
            return ImuSample(
                time_ms=int(parts[1]),
                accel_x=float(parts[2]),
                accel_y=float(parts[3]),
                accel_z=float(parts[4]),
                gyro_x=float(parts[5]),
                gyro_y=float(parts[6]),
                gyro_z=float(parts[7]),
                yaw_deg=float(parts[8]),
                servo_deg=int(parts[9]),
                batt_voltage=float(parts[10])
            )
        except ValueError:
            return None
    elif line.startswith('A '):
        # A <text>
        return Ack(text=line[2:])
    elif line == 'W stop':
        return WatchdogStop()
    else:
        return None


def format_drive(left: int, right: int) -> str:
    """
    Format a drive command for the Scout base bridge (Python) protocol.

    Clamped to [-255, 255] and rounded to nearest integer.
    """
    left = max(-255, min(255, left))
    right = max(-255, min(255, right))
    return f'M {left} {right}\n'


def format_servo(servo_deg: int) -> str:
    """
    Format a servo command for the Scout base bridge (Python) protocol.

    Clamped to [0, 180] and rounded to nearest integer.
    """
    servo_deg = max(0, min(180, servo_deg))
    return f'V {servo_deg}\n'
