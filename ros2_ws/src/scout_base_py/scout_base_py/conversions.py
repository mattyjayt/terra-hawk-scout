"""
Algorithmic conversions between Scout base bridge (Python).

protocol and ROS2 messages.
"""

import collections

G = 9.80665  # m/s^2, standard gravity


def g_to_mps2(g: float) -> float:
    """Convert acceleration from g to m/s^2."""
    return g * G


def dps_to_radps(w_dps: float) -> float:
    """Convert angular velocity from degrees/s to radians/s."""
    return w_dps * (3.14159 / 180.0)


class StaleDetector:
    """
    Detect a frozen sensor: the same values repeated `limit` times in a row.

    A real MEMS sensor always has noise in its last digit, so identical
    consecutive samples mean the reading has stopped updating.
    """

    def __init__(self, limit: int = 10):
        """Create a detector that trips after `limit` identical samples."""
        self.limit = limit
        self.counter = 0        # how many times in a row we've seen _last
        self._last = None       # the previous sample (None = nothing seen yet)

    def update(self, values: tuple) -> bool:
        """Feed one sample; return True while the stream is stale."""
        if values == self._last:
            self.counter += 1
        else:
            self._last = values
            self.counter = 1    # a new value has now been seen once
        return self.counter >= self.limit


class RateMeter:
    """Messages per second over a sliding time window."""

    def __init__(self, window_s: float = 1.0):
        """Create a meter that counts ticks in the last `window_s` seconds."""
        self.window_s = window_s
        self._times = collections.deque()   # timestamps of recent ticks

    def tick(self, now: float) -> None:
        """Record one message arriving at time `now` (seconds)."""
        self._times.append(now)

    def rate(self, now: float) -> float:
        """Return ticks per second in the window (now - window_s, now]."""
        cutoff = now - self.window_s
        while self._times and self._times[0] < cutoff:
            self._times.popleft()           # forget ticks older than the window
        return len(self._times) / self.window_s


def link_level(connected: bool, rate_hz: float, expected_hz: float,
               imu_frozen: bool, s_since_line: float) -> int:
    """
    Determine the link level based on connection status.

    message rate, and IMU status.
    """
    # 0 OK / 1 WARN / 2 ERROR
    if not connected or s_since_line > 0.5 or imu_frozen:
        return 2
    if rate_hz < expected_hz * 0.9:
        return 1
    return 0
