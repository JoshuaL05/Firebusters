"""
sim.py - a fake world for testing the whole system without hardware.

Simulates:
  - two real fires (one slow, one fast-growing) and a third that appears later
  - a hot mug with NO flame (should be rejected as a false positive by the flame sensor)
  - a person at ~34 C (below fire temperature, should be ignored)
  - the ESP32: rail, pan-tilt, spray, flame sensor pointed where the nozzle aims, DHT11

Spraying only shrinks a fire if the nozzle is actually aimed at it, using the
same geometry as targeting.py - so bad aiming logic shows up as fires not going out.

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
RAIL_SPEED_MM_S = 120.0
HIT_DEG = 8.0               # aim error under which the spray hits
SPRAY_RATE = 0.35           # fire radius (sensor px) removed per second of direct hit
FLAME_FOV_DEG = 25.0        # flame sensor cone (with a tube on it)


@dataclass
class SimObject:
    px: float               # position in sensor pixels (0..31, 0..23)
    py: float
    radius: float           # size in sensor pixels
    temp: float             # peak temperature, C
    flame: bool             # real flame (visible to the flame sensor)
    growth: float = 0.0     # radius growth per second
    name: str = ""


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
        # actuator state (what the ESP32 would be doing)
        self.pan, self.tilt = cfg.REST_PAN_DEG, cfg.REST_TILT_DEG
        self.rail_mm = 0.0
        self.rail_target = 0.0
        self.spraying = False
        self.homed = False
        self.estop = False
        self._ys, self._xs = np.indices((cfg.SENSOR_H, cfg.SENSOR_W))

    def aim_error(self, obj):
        x, y = self.targeting.pixel_to_floor(obj.px * cfg.SCALE, obj.py * cfg.SCALE)
        need_pan, need_tilt = self.targeting.angles(self.targeting.nozzle_x(self.rail_mm), x, y)
        return math.hypot(self.pan - need_pan, self.tilt - need_tilt)

    def step(self, dt):
        self.t += dt
        if not self._spawned_late and self.t > 25:
            self.objects.append(SimObject(15, 12, 1.2, 300, True, 0.05, "late fire"))
            self._spawned_late = True

        step = RAIL_SPEED_MM_S * dt
        self.rail_mm += max(-step, min(step, self.rail_target - self.rail_mm))

        for o in self.objects:
            if o.flame:
                o.radius = min(4.0, o.radius + o.growth * dt)
            if self.spraying and o.temp > 50:
                err = self.aim_error(o)
                if err < HIT_DEG:
                    hit = 1 - err / HIT_DEG
                    if o.flame:
                        o.radius -= SPRAY_RATE * hit * dt
                    else:
                        o.temp -= 15 * hit * dt   # water cools the mug
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
                err = self.aim_error(o)
                if err < FLAME_FOV_DEG:
                    level += 4095 * min(1.0, o.radius / 2.0) * (1 - err / FLAME_FOV_DEG)
        level += random.gauss(150, 40)         # background IR noise
        return int(max(0, min(4095, level)))

    def room(self):
        fire_size = sum(o.radius for o in self.objects if o.flame)
        return AMBIENT_C + 0.8 * fire_size, 42.0 - 1.5 * fire_size


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

    def __init__(self, world, rail_enabled=cfg.RAIL_ENABLED):
        self.world = world
        self.rail_enabled = rail_enabled
        self.connected = True

    @property
    def telemetry(self):
        w = self.world
        temp, hum = w.room()
        return Telemetry(flame=w.flame_reading(), temp_c=temp, humidity=hum,
                         rail_mm=w.rail_mm, moving=abs(w.rail_mm - w.rail_target) > 0.5,
                         spraying=w.spraying, homed=w.homed, estop=w.estop,
                         stamp=time.monotonic())

    def ping(self):
        return True

    def home(self, timeout=30.0):
        self.world.rail_mm = self.world.rail_target = 0.0
        self.world.homed = True
        return True

    def move_rail(self, mm):
        if self.rail_enabled:
            self.world.rail_target = max(0.0, min(cfg.RAIL_LENGTH_MM, mm))
        return True

    def rail_done(self, mm):
        if not self.rail_enabled:
            return True
        t = self.telemetry
        return not t.moving and abs(t.rail_mm - mm) <= cfg.RAIL_TOLERANCE_MM

    def aim(self, pan, tilt, wait=True):
        self.world.pan = max(0.0, min(180.0, pan))
        self.world.tilt = max(0.0, min(180.0, tilt))
        return True

    def spray(self, on):
        self.world.spraying = bool(on)
        return True

    def shutdown(self):
        self.spray(False)
        self.aim(cfg.REST_PAN_DEG, cfg.REST_TILT_DEG)
