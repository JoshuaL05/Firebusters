"""
sensor.py - readings that come back from the ESP32.

The ESP32 streams one telemetry line ~20 times a second:
    T <flame> <x_mm> <y_mm> <moving> <spraying> <homed> <estop>
e.g.
    T 2310 120.5 88.0 0 1 1 0

This file parses those lines and keeps a short history of flame readings so
decisions use an average instead of one noisy sample.
"""

from collections import deque
from dataclasses import dataclass


@dataclass
class Telemetry:
    flame: int = 0             # 0-4095, higher = more flame (ESP32 already inverts the module)
    x_mm: float = 0.0          # carriage position
    y_mm: float = 0.0
    moving: bool = False       # gantry moving or homing
    spraying: bool = False
    homed: bool = False
    estop: bool = False        # emergency stop button pressed
    stamp: float = 0.0         # time.monotonic() when received


def parse_telemetry(line, stamp):
    """Parse one 'T ...' line. Returns Telemetry, or None if the line is malformed."""
    parts = line.split()
    if len(parts) != 8 or parts[0] != "T":
        return None
    try:
        return Telemetry(
            flame=int(float(parts[1])),
            x_mm=float(parts[2]),
            y_mm=float(parts[3]),
            moving=parts[4] == "1",
            spraying=parts[5] == "1",
            homed=parts[6] == "1",
            estop=parts[7] == "1",
            stamp=stamp,
        )
    except ValueError:
        return None


class FlameWindow:
    """Recent flame readings, so decisions use an average instead of one noisy sample."""

    def __init__(self, maxlen=400):
        self._samples = deque(maxlen=maxlen)  # (time, value)

    def add(self, t, value):
        self._samples.append((t, value))

    def mean(self, since):
        vals = [v for t, v in self._samples if t >= since]
        if not vals:
            return self._samples[-1][1] if self._samples else 0.0
        return sum(vals) / len(vals)

    @property
    def latest(self):
        return self._samples[-1][1] if self._samples else 0.0
