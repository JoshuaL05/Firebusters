"""
MLX90614 Temperature readings
"""

"""
sensor.py - readings that come back from the ESP32.

The ESP32 streams one telemetry line ~20 times a second:
    T <flame> <temp_c> <humidity> <rail_mm> <moving> <spraying> <homed> <estop>
e.g.
    T 2310 23.0 41.0 120.5 0 1 1 0

This file parses those lines and turns the raw numbers into things the
state machine cares about: a smoothed flame level and a "room is heating up"
alert from the DHT11.
"""

import math
import statistics
from collections import deque
from dataclasses import dataclass

import config as cfg


@dataclass
class Telemetry:
    flame: int = 0             # 0-4095, higher = more flame (ESP32 already inverts the module)
    temp_c: float = math.nan   # DHT11 room temperature
    humidity: float = math.nan # DHT11 relative humidity, %
    rail_mm: float = 0.0       # carriage position
    moving: bool = False       # rail moving or homing
    spraying: bool = False
    homed: bool = False
    estop: bool = False        # emergency stop button pressed
    stamp: float = 0.0         # time.monotonic() when received


def parse_telemetry(line, stamp):
    """Parse one 'T ...' line. Returns Telemetry, or None if the line is malformed."""
    parts = line.split()
    if len(parts) != 9 or parts[0] != "T":
        return None
    try:
        return Telemetry(
            flame=int(float(parts[1])),
            temp_c=float(parts[2]),
            humidity=float(parts[3]),
            rail_mm=float(parts[4]),
            moving=parts[5] == "1",
            spraying=parts[6] == "1",
            homed=parts[7] == "1",
            estop=parts[8] == "1",
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


class EnvMonitor:
    """Learns the room's normal temperature/humidity for the first few seconds,
    then raises `alert` if the room warms up or dries out (signs of a fire)."""

    def __init__(self, baseline_s=cfg.ENV_BASELINE_S):
        self.baseline_s = baseline_s
        self._start = None
        self._samples = []
        self.base_temp = None
        self.base_hum = None
        self.temp = math.nan
        self.hum = math.nan

    def update(self, tel, now):
        if math.isnan(tel.temp_c) or math.isnan(tel.humidity):
            return
        self.temp, self.hum = tel.temp_c, tel.humidity
        if self.base_temp is not None:
            return
        if self._start is None:
            self._start = now
        self._samples.append((tel.temp_c, tel.humidity))
        if now - self._start >= self.baseline_s:
            self.base_temp = statistics.median(s[0] for s in self._samples)
            self.base_hum = statistics.median(s[1] for s in self._samples)

    @property
    def alert(self):
        if self.base_temp is None or math.isnan(self.temp):
            return False
        return (self.temp - self.base_temp >= cfg.ENV_TEMP_RISE_C
                or self.base_hum - self.hum >= cfg.ENV_HUMIDITY_DROP)

    def describe(self):
        if math.isnan(self.temp):
            return "room ?"
        txt = f"room {self.temp:.0f}C/{self.hum:.0f}%"
        return txt + (" ALERT" if self.alert else "")
