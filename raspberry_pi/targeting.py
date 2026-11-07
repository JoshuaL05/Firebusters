"""
targeting.py - which fire to hit, and where to drive the gantry.

  1. pixel -> floor:  thermal-image pixel to floor (x, y) in cm, using the
     perspective matrix saved by calibrate.py (or a simple scale until then)
  2. floor -> gantry: carriage position in mm so the nozzle sits right above it
  3. priority:        bigger and faster-growing fires first, small penalty
                      for long trips
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
class MovePlan:
    x_mm: float
    y_mm: float
    floor_xy: tuple


class Targeting:
    def __init__(self, calibration_file=cfg.CALIBRATION_FILE):
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
        # Uncalibrated: assume the camera sees exactly the area, top of image = far (+y) edge.
        return cx / WORK_W * cfg.AREA_W_CM, (1 - cy / WORK_H) * cfg.AREA_H_CM

    def floor_to_pixel(self, x, y):
        if self.H_inv is not None:
            p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float64), self.H_inv)[0, 0]
            return float(p[0]), float(p[1])
        return x / cfg.AREA_W_CM * WORK_W, (1 - y / cfg.AREA_H_CM) * WORK_H

    # ---- floor <-> gantry --------------------------------------------------
    def floor_to_gantry(self, x, y):
        return (_clamp((x - cfg.HOME_FLOOR_X_CM) * 10.0, 0.0, cfg.X_TRAVEL_MM),
                _clamp((y - cfg.HOME_FLOOR_Y_CM) * 10.0, 0.0, cfg.Y_TRAVEL_MM))

    def gantry_to_floor(self, x_mm, y_mm):
        return cfg.HOME_FLOOR_X_CM + x_mm / 10.0, cfg.HOME_FLOOR_Y_CM + y_mm / 10.0

    def plan_floor(self, x, y):
        gx, gy = self.floor_to_gantry(x, y)
        return MovePlan(gx, gy, (x, y))

    def plan(self, cx, cy):
        """Where to drive the gantry for a fire at thermal pixel (cx, cy)."""
        return self.plan_floor(*self.pixel_to_floor(cx, cy))

    # ---- priority ----------------------------------------------------------
    def score(self, fire, gantry_xy=None):
        s = fire.area * (1 + max(fire.growth_rate, 0.0))
        if gantry_xy is not None:
            gx, gy = self.floor_to_gantry(*self.pixel_to_floor(fire.cx, fire.cy))
            travel_cm = math.hypot(gx - gantry_xy[0], gy - gantry_xy[1]) / 10.0
            s /= 1 + cfg.DISTANCE_PENALTY * travel_cm
        return s

    def rank(self, fires, gantry_xy=None):
        return sorted(fires, key=lambda f: self.score(f, gantry_xy), reverse=True)
