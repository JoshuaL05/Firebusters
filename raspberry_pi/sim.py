"""
sim.py - a fake world for testing the whole system without hardware.

Simulates:
  - two real fires (one slow, one fast-growing) and a third that appears later
  - a hot mug with NO flame (should be rejected as a false positive by the flame sensor)
  - a person-temperature blob at ~34 C (below fire temperature, should be ignored)
  - the ESP32: x/y gantry, spray, flame sensor pointing down from the carriage

Spraying only shrinks a fire if the nozzle is actually over it, using the same
pixel -> floor -> gantry maths as targeting.py, so bad positioning logic shows
up as fires not going out.

Used by: python3 main.py --sim [--show]
"""

import math
import random
import time
from dataclasses import dataclass

import numpy as np

import config as cfg
from sensor import Telemetry

AMBIENT_C = 21.0
GANTRY_SPEED_MM_S = 120.0
HIT_CM = 2.0                # nozzle within this distance of a fire = direct hit
SPRAY_RATE = 0.35           # fire radius (sensor px) removed per second of direct hit
FLAME_VIEW_CM = 6.0         # flame sensor sees flames within this distance of the carriage


@dataclass
class SimObject:
    px: float               # position in sensor pixels (0..31, 0..23)
    py: float
    radius: float           # size in sensor pixels
    temp: float             # peak temperature, C
    flame: bool             # real flame (visible to the flame sensor)
    growth: float = 0.0     # radius growth per second
    name: str = ""
    cooling: float = 0.0    # deg C lost per second on its own (leftover wax)


class SimWorld:
    def __init__(self, targeting, seed=1):
        random.seed(seed)
        self.targeting = targeting
        self.t = 0.0
        self.objects = [
            SimObject(8, 7, 1.6, 350, True, 0.02, "slow fire"),
            SimObject(24, 15, 1.0, 320, True, 0.07, "fast fire"),
            SimObject(27, 4, 1.3, 85, False, 0.0, "hot mug"),
            SimObject(4, 19, 1.8, 34, False, 0.0, "person"),
        ]
        self._spawned_late = False
        # what the ESP32 would be doing
        self.x_mm = self.y_mm = 0.0
        self.tx_mm = self.ty_mm = 0.0
        self.spraying = False
        self.homed = False
        self.estop = False
        self._ys, self._xs = np.indices((cfg.SENSOR_H, cfg.SENSOR_W))

    def nozzle_distance_cm(self, obj):
        fx, fy = self.targeting.pixel_to_floor(obj.px * cfg.SCALE, obj.py * cfg.SCALE)
        nx, ny = self.targeting.gantry_to_floor(self.x_mm, self.y_mm)
        return math.hypot(fx - nx, fy - ny)

    def step(self, dt):
        self.t += dt
        if not self._spawned_late and self.t > 25:
            self.objects.append(SimObject(15, 12, 1.2, 300, True, 0.05, "late fire"))
            self._spawned_late = True

        step = GANTRY_SPEED_MM_S * dt
        self.x_mm += max(-step, min(step, self.tx_mm - self.x_mm))
        self.y_mm += max(-step, min(step, self.ty_mm - self.y_mm))

        for o in self.objects:
            if o.flame:
                o.radius = min(4.0, o.radius + o.growth * dt)
            o.temp -= o.cooling * dt
            if self.spraying and o.temp > 50:
                d = self.nozzle_distance_cm(o)
                if d < HIT_CM + o.radius:              # spray cone covers part of the fire
                    hit = max(0.3, 1 - d / (HIT_CM + o.radius))
                    if o.flame:
                        o.radius -= SPRAY_RATE * hit * dt
                    else:
                        o.temp -= 15 * hit * dt        # water cools the mug
        # A put-out candle leaves a warm patch of melted wax behind for a while.
        for o in self.objects:
            if o.flame and o.radius <= 0.35:
                self.objects.append(SimObject(o.px, o.py, 1.2, 90, False, 0.0, "warm wax", cooling=0.6))
        self.objects = [o for o in self.objects
                        if (o.radius > 0.35 if o.flame else (o.temp > 40 or o.name == "person"))]

    def render(self):
        temps = np.full((cfg.SENSOR_H, cfg.SENSOR_W), AMBIENT_C, np.float32)
        temps += np.random.normal(0, 0.3, temps.shape).astype(np.float32)
        for o in self.objects:
            d2 = (self._xs - o.px) ** 2 + (self._ys - o.py) ** 2
            sigma = max(o.radius, 0.3) * 0.8
            blob = AMBIENT_C + (o.temp - AMBIENT_C) * np.exp(-d2 / (2 * sigma ** 2))
            temps = np.maximum(temps, blob.astype(np.float32))
        return temps

    def flame_reading(self):
        level = 0.0
        for o in self.objects:
            if o.flame:
                d = self.nozzle_distance_cm(o)
                if d < FLAME_VIEW_CM:
                    level += 4095 * min(1.0, o.radius / 2.0) * (1 - d / FLAME_VIEW_CM)
        level += random.gauss(150, 40)         # background IR noise
        return int(max(0, min(4095, level)))


class SimThermal:
    """Stands in for scanner.ThermalCamera."""

    def __init__(self, world, fps=8):
        self.world = world
        self.fps = fps
        self.source = "simulator"
        self._last = time.monotonic()

    def read(self):
        time.sleep(1 / self.fps)
        now = time.monotonic()
        self.world.step(now - self._last)
        self._last = now
        return self.world.render()

    def close(self):
        pass


class SimESP32:
    """Stands in for motion.ESP32Link (same methods)."""

    def __init__(self, world):
        self.world = world
        self.connected = True

    @property
    def telemetry(self):
        w = self.world
        moving = abs(w.x_mm - w.tx_mm) > 0.5 or abs(w.y_mm - w.ty_mm) > 0.5
        return Telemetry(flame=w.flame_reading(), x_mm=w.x_mm, y_mm=w.y_mm, moving=moving,
                         spraying=w.spraying, homed=w.homed, estop=w.estop,
                         stamp=time.monotonic())

    def ping(self):
        return True

    def home(self, timeout=60.0):
        w = self.world
        w.x_mm = w.y_mm = w.tx_mm = w.ty_mm = 0.0
        w.homed = True
        return True

    def move(self, x_mm, y_mm, wait=True):
        self.world.tx_mm = max(0.0, min(cfg.X_TRAVEL_MM, x_mm))
        self.world.ty_mm = max(0.0, min(cfg.Y_TRAVEL_MM, y_mm))
        return True

    def at(self, x_mm, y_mm):
        t = self.telemetry
        return (not t.moving and abs(t.x_mm - x_mm) <= cfg.POS_TOLERANCE_MM
                and abs(t.y_mm - y_mm) <= cfg.POS_TOLERANCE_MM)

    def spray(self, on):
        self.world.spraying = bool(on)
        return True

    def shutdown(self):
        self.spray(False)
