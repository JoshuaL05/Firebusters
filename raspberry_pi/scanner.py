"""
scanner.py - thermal fire detection and tracking (MLX90640).

Reads 32x24 temperature frames from an MLX90640 thermal camera, finds hot
blobs, and tracks them over time so the later stages (targeting / state
machine) get a list of stable fire candidates with position, size, temperature
and growth rate.

Sources:
  - MLX90640 over I2C on the Raspberry Pi (default)
  - a built-in simulator with fake growing fires (--sim), for testing on a laptop
  - a saved frame (.npy) from the 's' key (--frame file.npy)

Try it:
  python scanner.py --sim                  # laptop test, no sensor needed
  python scanner.py --headless             # Pi over SSH: prints detections
  python scanner.py --fire-temp 45         # lower threshold (e.g. hot-water mugs)
  python scanner.py --frame thermal_123.npy

Keys in the preview window:  q = quit,  p = print slider values,  s = save raw frame (.npy)
"""

import argparse
import time
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

# All tunable numbers live in config.py.
from config import (
    SENSOR_W, SENSOR_H, SCALE, REFRESH_HZ, FLIP_H, FLIP_V,
    FIRE_TEMP_C, AMBIENT_DELTA_C, MIN_AREA,
    MAX_MATCH_DIST, CONFIRM_FRAMES, MAX_MISSED_FRAMES, GROWTH_WINDOW_S,
    DISPLAY_MIN_C, DISPLAY_MAX_C,
)


def default_params():
    return {
        "fire_temp": FIRE_TEMP_C,
        "ambient_delta": AMBIENT_DELTA_C,
        "min_area": MIN_AREA,
    }


# ---------------------------------------------------------------------------
# Detection (single frame)
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    cx: float         # center x, working-image pixels (0..319)
    cy: float         # center y, working-image pixels (0..239)
    area: float       # blob area, working-image pixels^2
    bbox: tuple       # (x, y, w, h)
    max_temp: float   # hottest point in the blob, deg C
    mean_temp: float  # average blob temperature, deg C


def upscale(temps):
    """24x32 temperature grid -> smooth 240x320 float image (still in deg C)."""
    return cv2.resize(temps.astype(np.float32),
                      (SENSOR_W * SCALE, SENSOR_H * SCALE),
                      interpolation=cv2.INTER_CUBIC)


def ambient_temp(temps):
    """Room temperature estimate: the median pixel (fires are a small part of the view)."""
    return float(np.median(temps))


def make_mask(temps_big, ambient, params=None):
    """White = hot enough to be fire. Hot must be above the absolute threshold
    AND well above room temperature, so a warm room doesn't trigger everything."""
    p = params or default_params()
    threshold = max(p["fire_temp"], ambient + p["ambient_delta"])
    mask = (temps_big >= threshold).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect(temps, params=None):
    """Find hot blobs in one 24x32 frame. Returns (detections, mask, temps_big)."""
    p = params or default_params()
    temps_big = upscale(temps)
    mask = make_mask(temps_big, ambient_temp(temps), p)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p["min_area"]:
            continue
        x, y, w, h = cv2.boundingRect(c)
        blob = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        blob_temps = temps_big[blob > 0]
        if blob_temps.size == 0:
            continue

        # Heat-weighted center: aims at the hottest part, not just the outline's middle.
        weights = np.where(blob > 0, np.clip(temps_big - p["fire_temp"], 0, None) + 1e-3, 0)
        total = weights.sum()
        ys, xs = np.indices(weights.shape)
        cx = float((weights * xs).sum() / total) if total > 0 else x + w / 2
        cy = float((weights * ys).sum() / total) if total > 0 else y + h / 2

        detections.append(Detection(cx, cy, area, (x, y, w, h),
                                    float(blob_temps.max()), float(blob_temps.mean())))
    return detections, mask, temps_big


# ---------------------------------------------------------------------------
# Tracking (across frames) - same idea as before
# ---------------------------------------------------------------------------
@dataclass
class Fire:
    id: int
    cx: float
    cy: float
    area: float
    bbox: tuple
    max_temp: float
    mean_temp: float
    frames_seen: int = 1
    missed: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=300))  # (time, area)

    @property
    def confirmed(self):
        return self.frames_seen >= CONFIRM_FRAMES

    @property
    def visible(self):
        return self.missed == 0

    @property
    def sensor_xy(self):
        """Center in original sensor pixels (0..31, 0..23), handy for calibration."""
        return self.cx / SCALE, self.cy / SCALE

    @property
    def growth_rate(self):
        """Fractional area change per second over the last GROWTH_WINDOW_S.
        0.5 = growing 50% per second; negative = shrinking."""
        if len(self.history) < 2:
            return 0.0
        t_now, a_now = self.history[-1]
        for t_old, a_old in self.history:
            if t_now - t_old <= GROWTH_WINDOW_S:
                break
        dt = t_now - t_old
        if dt <= 0 or a_old <= 0:
            return 0.0
        return (a_now - a_old) / a_old / dt

    def _apply(self, det, t):
        self.cx, self.cy, self.area, self.bbox = det.cx, det.cy, det.area, det.bbox
        self.max_temp, self.mean_temp = det.max_temp, det.mean_temp
        self.history.append((t, det.area))


class FireTracker:
    """Keeps an ID on each fire across frames. A fire survives a few missed
    frames before being dropped, since readings can be noisy."""

    def __init__(self):
        self.fires = []
        self._next_id = 1

    def update(self, detections, t=None):
        t = time.monotonic() if t is None else t

        pairs = []
        for fi, fire in enumerate(self.fires):
            for di, det in enumerate(detections):
                d = ((fire.cx - det.cx) ** 2 + (fire.cy - det.cy) ** 2) ** 0.5
                if d <= MAX_MATCH_DIST:
                    pairs.append((d, fi, di))
        pairs.sort()

        used_fires, used_dets = set(), set()
        for _, fi, di in pairs:
            if fi in used_fires or di in used_dets:
                continue
            used_fires.add(fi)
            used_dets.add(di)
            fire = self.fires[fi]
            fire._apply(detections[di], t)
            fire.frames_seen += 1
            fire.missed = 0

        kept = []
        for fi, fire in enumerate(self.fires):
            if fi not in used_fires:
                fire.missed += 1
            if fire.missed <= MAX_MISSED_FRAMES:
                kept.append(fire)
        self.fires = kept

        for di, det in enumerate(detections):
            if di not in used_dets:
                fire = Fire(self._next_id, det.cx, det.cy, det.area, det.bbox,
                            det.max_temp, det.mean_temp)
                fire.history.append((t, det.area))
                self.fires.append(fire)
                self._next_id += 1
        return self.fires

    def candidates(self):
        """Confirmed fires visible in the latest frame (what targeting should use)."""
        return [f for f in self.fires if f.confirmed and f.visible]

    def forget(self, fire_id):
        """Remove a fire, e.g. once it's out or marked a false positive."""
        self.fires = [f for f in self.fires if f.id != fire_id]


# ---------------------------------------------------------------------------
# Thermal sources
# ---------------------------------------------------------------------------
class ThermalCamera:
    """MLX90640 over I2C. Returns 24x32 numpy arrays of deg C."""

    def __init__(self, refresh_hz=REFRESH_HZ, flip_h=FLIP_H, flip_v=FLIP_V):
        import board
        import busio
        import adafruit_mlx90640

        rates = {
            2: adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
            4: adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
            8: adafruit_mlx90640.RefreshRate.REFRESH_8_HZ,
            16: adafruit_mlx90640.RefreshRate.REFRESH_16_HZ,
        }
        i2c = busio.I2C(board.SCL, board.SDA, frequency=1_000_000)
        self._mlx = adafruit_mlx90640.MLX90640(i2c)
        self._mlx.refresh_rate = rates.get(refresh_hz, rates[4])
        self._buf = [0.0] * (SENSOR_W * SENSOR_H)
        self.flip_h, self.flip_v = flip_h, flip_v
        self.source = f"MLX90640 @ {refresh_hz} Hz"

    def read(self):
        for _ in range(3):  # the sensor occasionally returns a bad frame; just retry
            try:
                self._mlx.getFrame(self._buf)
                break
            except (ValueError, RuntimeError, OSError):
                continue
        else:
            return None
        temps = np.array(self._buf, dtype=np.float32).reshape(SENSOR_H, SENSOR_W)
        if self.flip_h:
            temps = temps[:, ::-1]
        if self.flip_v:
            temps = temps[::-1, :]
        return temps

    def close(self):
        pass


class SimulatedThermal:
    """Fake sensor for testing without hardware: a 22 deg C room, a person-
    temperature blob (should be ignored), and two fires that grow over time."""

    def __init__(self, fps=8):
        self.t0 = time.monotonic()
        self.fps = fps
        self.source = "simulator"
        self._ys, self._xs = np.indices((SENSOR_H, SENSOR_W))

    def _blob(self, x, y, radius, peak, base):
        d2 = (self._xs - x) ** 2 + (self._ys - y) ** 2
        return (peak - base) * np.exp(-d2 / (2 * radius ** 2))

    def read(self):
        time.sleep(1 / self.fps)
        t = time.monotonic() - self.t0
        base = 22.0
        temps = np.full((SENSOR_H, SENSOR_W), base, np.float32)
        temps += np.random.normal(0, 0.3, temps.shape).astype(np.float32)
        temps += self._blob(4, 18, 1.5, 34, base)                      # person: ~34 C
        temps += self._blob(8, 8, 1.2 + 0.05 * t, 200, base)           # slow fire
        if t > 3:                                                      # 2nd fire appears, grows fast
            temps += self._blob(24, 14, 0.8 + 0.25 * (t - 3), 300, base)
        return temps

    def close(self):
        pass


# ---------------------------------------------------------------------------
# Preview / debugging
# ---------------------------------------------------------------------------
def preview_score(fire):
    """Rough priority just for the preview labels. targeting.py will own the real one."""
    return fire.area * (1 + max(fire.growth_rate, 0))


def colorize(temps_big):
    norm = np.clip((temps_big - DISPLAY_MIN_C) / (DISPLAY_MAX_C - DISPLAY_MIN_C), 0, 1)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)


def draw_overlay(temps_big, fires):
    out = colorize(temps_big)
    ranked = sorted([f for f in fires if f.confirmed and f.visible], key=preview_score, reverse=True)
    rank_of = {f.id: i + 1 for i, f in enumerate(ranked)}
    for f in fires:
        if not f.visible:
            continue
        x, y, w, h = f.bbox
        color = (0, 255, 0) if f.confirmed else (160, 160, 160)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 1)
        cv2.drawMarker(out, (int(f.cx), int(f.cy)), color, cv2.MARKER_CROSS, 10, 1)
        label = f"#{f.id} {f.max_temp:.0f}C a={int(f.area)} g={f.growth_rate:+.2f}"
        if f.id in rank_of:
            label = f"P{rank_of[f.id]} " + label
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
        tx = max(0, min(x, out.shape[1] - tw - 2))          # keep labels inside the image
        cv2.putText(out, label, (tx, max(y - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    return out


SLIDERS = [  # (name, params key, max)
    ("fire temp C", "fire_temp", 300),
    ("above room C", "ambient_delta", 100),
    ("min area", "min_area", 2000),
]


def make_sliders(window, params):
    for name, key, maxv in SLIDERS:
        cv2.createTrackbar(name, window, int(params[key]), maxv, lambda _v: None)


def read_sliders(window, params):
    for name, key, _ in SLIDERS:
        params[key] = cv2.getTrackbarPos(name, window)


def summary(fires):
    if not fires:
        return "no fires"
    return " | ".join(
        f"#{f.id} ({f.cx:.0f},{f.cy:.0f}) {f.max_temp:.0f}C a={f.area:.0f} g={f.growth_rate:+.2f}"
        for f in fires)


def run(args):
    if args.frame:
        frame = np.load(args.frame)
        source = None
        print(f"Using saved frame {args.frame}")
    else:
        source = (SimulatedThermal() if args.sim
                else ThermalCamera(args.hz, args.flip_h or FLIP_H, args.flip_v or FLIP_V))
        print(f"Using {source.source}")

    tracker = FireTracker()
    params = default_params()
    if args.fire_temp is not None:
        params["fire_temp"] = args.fire_temp

    win = "thermal"
    if not args.headless:
        cv2.namedWindow(win)
        make_sliders(win, params)
        print("Keys: q = quit, p = print slider values, s = save raw frame")

    last_print = 0.0
    try:
        while True:
            temps = frame if source is None else source.read()
            if temps is None:
                print("Sensor returned no frame, retrying...")
                continue

            if not args.headless:
                read_sliders(win, params)

            detections, mask, temps_big = detect(temps, params)
            tracker.update(detections)

            if args.headless:
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    cands = sorted(tracker.candidates(), key=preview_score, reverse=True)
                    print(f"room {ambient_temp(temps):.1f}C, max {temps.max():.1f}C -> {summary(cands)}")
                if source is None:
                    time.sleep(0.1)
                continue

            cv2.imshow(win, draw_overlay(temps_big, tracker.fires))
            cv2.imshow("mask", mask)
            key = cv2.waitKey(1 if source else 50) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p"):
                print(params)
            if key == ord("s"):
                name = f"thermal_{int(time.time())}.npy"
                np.save(name, temps)
                print(f"Saved {name}")
    except KeyboardInterrupt:
        pass
    finally:
        if source:
            source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Thermal fire detection test tool")
    ap.add_argument("--sim", action="store_true", help="use the built-in simulator (no sensor)")
    ap.add_argument("--frame", help="run on a saved .npy frame")
    ap.add_argument("--headless", action="store_true", help="no windows, print detections")
    ap.add_argument("--fire-temp", type=float, help=f"fire threshold in C (default {FIRE_TEMP_C})")
    ap.add_argument("--hz", type=int, default=REFRESH_HZ, help="sensor refresh rate: 2, 4, 8, 16")
    ap.add_argument("--flip-h", action="store_true", help="mirror image left-right")
    ap.add_argument("--flip-v", action="store_true", help="flip image upside down")
    run(ap.parse_args())
