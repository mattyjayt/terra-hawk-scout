"""Algorithmic conversions between Scout base bridge (Python).

protocol and ROS2 messages.
"""

G = 9.80665  # m/s^2, standard gravity


def g_to_mps2(g: float) -> float:
    """Convert acceleration from g to m/s^2."""
    return g * G


def dps_to_radps(w_dps: float) -> float:
    """Convert angular velocity from degrees/s to radians/s."""
    return w_dps * (3.14159 / 180.0)


class StaleDetector:
    """Detects if a stream of values has been stale for a.

    certain number of updates.
    """

    def __init__(self, limit: int = 10):
        """Initialize the stale detector."""
        self.limit = limit
        self.counter = 0

    def update(self, values: tuple) -> bool:
        """Update the stale detector with new values."""
        if all(v == 0 for v in values):
            self.counter += 1
        else:
            self.counter = 0
        return self.counter >= self.limit


class RateMeter:
    """Measures the rate of incoming messages."""

    def __init__(self, window_s: float = 1.0):
        """Initialize the rate meter."""
        self.last_time = None
        self.count = 0
        self.window_s = window_s

    def tick(self, now: float) -> None:
        """Record a new tick at the given time."""
        if self.last_time is None:
            self.last_time = now
            self.count = 1
        else:
            if now - self.last_time > self.window_s:
                # Reset count if outside the window
                self.count = 1
                self.last_time = now
            else:
                self.count += 1

    def rate(self, now: float) -> float:
        """Calculate the rate of ticks per second."""
        if self.last_time is None:
            return 0.0
        if now - self.last_time > self.window_s:
            return 0.0
        return self.count / self.window_s


def link_level(connected: bool, rate_hz: float, expected_hz: float,
               imu_frozen: bool, s_since_line: float) -> int:
    """Determine the link level based on connection status.

    message rate, and IMU status.
    """
    # 0 OK / 1 WARN / 2 ERROR
    if not connected or s_since_line > 0.5 or imu_frozen:
        return 2
    if rate_hz < expected_hz * 0.9:
        return 1
    return 0
