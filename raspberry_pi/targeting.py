"""
Warm target detection and confirmation
"""
"""
targeting.py - which fire to hit, and how to point at it.

  1. pixel -> floor:  thermal-image pixel to floor (x, y) in cm, using the
     perspective matrix saved by calibrate.py (or a simple scale until then)
  2. floor -> motors: rail position (mm) + pan/tilt servo angles
  3. priority:        bigger and faster-growing fires first, small penalty
                      for long rail trips
"""

import json
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np

import config as cfg

WORK_W = cfg.SENSOR_W * cfg.SCALE   # working-image size from scanner.py
WORK_H = cfg.SENSOR_H * cfg.SCALE


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class AimPlan:
    rail_mm: float
    pan: float
    tilt: float
    floor_xy: tuple


class Targeting:
    def __init__(self, calibration_file=cfg.CALIBRATION_FILE, rail_enabled=cfg.RAIL_ENABLED):
        self.rail_enabled = rail_enabled
        self.H = None
        self.H_inv = None
        if calibration_file and os.path.exists(calibration_file):
            with open(calibration_file) as f:
                data = json.load(f)
            self.H = np.array(data["homography"], dtype=np.float64)
            self.H_inv = np.linalg.inv(self.H)

    @property
    def calibrated(self):
        return self.H is not None

    # ---- pixel <-> floor -------------------------------------------------
    def pixel_to_floor(self, cx, cy):
        if self.H is not None:
            p = cv2.perspectiveTransform(np.array([[[cx, cy]]], np.float64), self.H)[0, 0]
            return float(p[0]), float(p[1])
        # Uncalibrated: assume the camera sees exactly the demo area, top of image = far edge.
        return cx / WORK_W * cfg.AREA_W_CM, (1 - cy / WORK_H) * cfg.AREA_H_CM

    def floor_to_pixel(self, x, y):
        if self.H_inv is not None:
            p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float64), self.H_inv)[0, 0]
            return float(p[0]), float(p[1])
        return x / cfg.AREA_W_CM * WORK_W, (1 - y / cfg.AREA_H_CM) * WORK_H

    # ---- floor -> motors ---------------------------------------------------
    def rail_for(self, x):
        if not self.rail_enabled:
            return 0.0
        return _clamp((x - cfg.RAIL_ORIGIN_X_CM) * 10.0, 0.0, cfg.RAIL_LENGTH_MM)

    def nozzle_x(self, rail_mm):
        if not self.rail_enabled:
            return cfg.FIXED_NOZZLE_X_CM
        return cfg.RAIL_ORIGIN_X_CM + rail_mm / 10.0

    def angles(self, nozzle_x, x, y):
        """Servo angles to hit floor point (x, y) from a nozzle at nozzle_x."""
        dx = x - nozzle_x
        dy = max(y - cfg.NOZZLE_Y_CM, 1.0)
        pan_rel = math.degrees(math.atan2(dx, dy))                        # 0 = straight out
        depression = math.degrees(math.atan2(cfg.NOZZLE_HEIGHT_CM, math.hypot(dx, dy)))
        pan = cfg.PAN_CENTER_DEG + cfg.PAN_DIR * pan_rel + cfg.PAN_TRIM_DEG
        tilt = (cfg.TILT_HORIZONTAL_DEG + cfg.TILT_DIR * (depression - cfg.TILT_LOB_DEG)
                + cfg.TILT_TRIM_DEG)
        return (_clamp(pan, cfg.PAN_MIN_DEG, cfg.PAN_MAX_DEG),
                _clamp(tilt, cfg.TILT_MIN_DEG, cfg.TILT_MAX_DEG))

    def plan_floor(self, x, y):
        rail = self.rail_for(x)
        pan, tilt = self.angles(self.nozzle_x(rail), x, y)
        return AimPlan(rail, pan, tilt, (x, y))

    def plan(self, cx, cy):
        """Full plan from a thermal pixel (working-image coords)."""
        return self.plan_floor(*self.pixel_to_floor(cx, cy))

    # ---- priority ----------------------------------------------------------
    def score(self, fire, current_rail_mm=None):
        s = fire.area * (1 + max(fire.growth_rate, 0.0))
        if self.rail_enabled and current_rail_mm is not None:
            x, _ = self.pixel_to_floor(fire.cx, fire.cy)
            travel_cm = abs(self.rail_for(x) - current_rail_mm) / 10.0
            s /= 1 + cfg.DISTANCE_PENALTY * travel_cm
        return s

    def rank(self, fires, current_rail_mm=None):
        return sorted(fires, key=lambda f: self.score(f, current_rail_mm), reverse=True)
