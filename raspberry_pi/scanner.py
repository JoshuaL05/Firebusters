"""
scanner.py - camera-based flame detection and tracking.

Finds bright flame-colored blobs in camera frames and tracks them over time,
so the later stages (targeting / state machine) get a list of stable fire
candidates with position, size and growth rate.

Works with:
  - a laptop webcam (OpenCV VideoCapture)
  - the Raspberry Pi Camera V2.1 (picamera2), used automatically when available
  - a still image, for quick testing

Try it:
  python scanner.py                    # live camera, preview window + tuning sliders
  python scanner.py --image test.jpg   # run detection on one photo
  python scanner.py --headless         # Pi over SSH (no screen): prints detections
  python scanner.py --exposure 5000    # Pi camera only: fixed short exposure (microseconds)

Keys in the preview window:  q = quit,  p = print current slider values,  s = save snapshot
"""

import argparse
import time
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Tunable settings (move these into config.py once that file is set up)
# ---------------------------------------------------------------------------
CAMERA_INDEX = 0          # webcam index for OpenCV
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# HSV thresholds. OpenCV hue runs 0-179 (red ~0, orange ~10-20, yellow ~25-35).
HUE_MIN = 0
HUE_MAX = 35
SAT_MIN = 80
VAL_MIN = 200
# Flame cores are often so bright they wash out to near-white. Those pixels
# count as flame only if they sit right next to flame-colored pixels, so a
# plain white light or white paper doesn't trigger detection on its own.
CORE_VAL_MIN = 245
CORE_SAT_MAX = 60

MIN_AREA = 40             # px^2: ignore blobs smaller than this
MAX_MATCH_DIST = 60       # px: max movement between frames to count as the same fire
CONFIRM_FRAMES = 5        # matched frames needed before a fire counts as a candidate
MAX_MISSED_FRAMES = 5     # drop a fire after this many frames without seeing it
GROWTH_WINDOW_S = 2.0     # seconds of history used to compute growth rate


def default_params():
    """Detection thresholds as a dict, so they can be changed live by sliders."""
    return {
        "hue_min": HUE_MIN,
        "hue_max": HUE_MAX,
        "sat_min": SAT_MIN,
        "val_min": VAL_MIN,
        "core_val_min": CORE_VAL_MIN,
        "core_sat_max": CORE_SAT_MAX,
        "min_area": MIN_AREA,
    }


# ---------------------------------------------------------------------------
# Detection (single frame)
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    cx: float     # center x, pixels
    cy: float     # center y, pixels
    area: float   # blob area, pixels^2
    bbox: tuple   # (x, y, w, h)


def make_mask(frame_bgr, params=None):
    """Return a black/white mask where white = flame-like pixels."""
    p = params or default_params()
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    colored = cv2.inRange(
        hsv,
        (p["hue_min"], p["sat_min"], p["val_min"]),
        (p["hue_max"], 255, 255),
    )

    core = cv2.inRange(hsv, (0, 0, p["core_val_min"]), (179, p["core_sat_max"], 255))
    near_colored = cv2.dilate(colored, np.ones((15, 15), np.uint8))
    core = cv2.bitwise_and(core, near_colored)

    mask = cv2.bitwise_or(colored, core)

    # Clean up speckle noise and fill small holes inside flames.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def detect(frame_bgr, params=None):
    """Find flame blobs in one frame. Returns (detections, mask)."""
    p = params or default_params()
    mask = make_mask(frame_bgr, p)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p["min_area"]:
            continue
        x, y, w, h = cv2.boundingRect(c)
        m = cv2.moments(c)
        if m["m00"] > 0:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        else:
            cx, cy = x + w / 2, y + h / 2
        detections.append(Detection(cx, cy, area, (x, y, w, h)))
    return detections, mask


# ---------------------------------------------------------------------------
# Tracking (across frames)
# ---------------------------------------------------------------------------
@dataclass
class Fire:
    id: int
    cx: float
    cy: float
    area: float
    bbox: tuple
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
    def growth_rate(self):
        """Fractional area change per second over the last GROWTH_WINDOW_S.
        0.5 means growing 50% per second; negative means shrinking."""
        if len(self.history) < 2:
            return 0.0
        t_now, a_now = self.history[-1]
        for t_old, a_old in self.history:          # oldest sample inside the window
            if t_now - t_old <= GROWTH_WINDOW_S:
                break
        dt = t_now - t_old
        if dt <= 0 or a_old <= 0:
            return 0.0
        return (a_now - a_old) / a_old / dt


class FireTracker:
    """Keeps an ID on each fire across frames so we know which ones are stable
    and how they're growing. Flames flicker, so a fire survives a few missed
    frames before being dropped."""

    def __init__(self):
        self.fires = []
        self._next_id = 1

    def update(self, detections, t=None):
        t = time.monotonic() if t is None else t

        # Greedy matching: closest fire/detection pairs first.
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
            fire, det = self.fires[fi], detections[di]
            fire.cx, fire.cy, fire.area, fire.bbox = det.cx, det.cy, det.area, det.bbox
            fire.frames_seen += 1
            fire.missed = 0
            fire.history.append((t, det.area))

        # Fires not seen this frame: count the miss, drop if gone too long.
        kept = []
        for fi, fire in enumerate(self.fires):
            if fi not in used_fires:
                fire.missed += 1
            if fire.missed <= MAX_MISSED_FRAMES:
                kept.append(fire)
        self.fires = kept

        # New detections become new fires.
        for di, det in enumerate(detections):
            if di not in used_dets:
                fire = Fire(self._next_id, det.cx, det.cy, det.area, det.bbox)
                fire.history.append((t, det.area))
                self.fires.append(fire)
                self._next_id += 1

        return self.fires

    def candidates(self):
        """Confirmed fires visible in the latest frame (what targeting should use)."""
        return [f for f in self.fires if f.confirmed and f.visible]

    def forget(self, fire_id):
        """Remove a fire, e.g. once it's been put out or marked a false positive."""
        self.fires = [f for f in self.fires if f.id != fire_id]


# ---------------------------------------------------------------------------
# Camera (Pi camera if available, otherwise webcam)
# ---------------------------------------------------------------------------
class Camera:
    def __init__(self, index=CAMERA_INDEX, width=FRAME_WIDTH, height=FRAME_HEIGHT, force_webcam=False, exposure_us=None):
        self._picam = None
        self._cap = None
        self.source = None

        if not force_webcam:
            try:
                from picamera2 import Picamera2
                self._picam = Picamera2()
                # "RGB888" in picamera2 is stored in B,G,R order, which is what OpenCV expects.
                cfg = self._picam.create_preview_configuration(
                    main={"size": (width, height), "format": "RGB888"})
                self._picam.configure(cfg)
                self._picam.start()
                if exposure_us:
                    # A short fixed exposure keeps flames orange instead of
                    # washing out to white, and darkens the background.
                    self._picam.set_controls({"AeEnable": False, "ExposureTime": int(exposure_us)})
                time.sleep(1.0)  # let the sensor settle
                self.source = "picamera2"
            except Exception:
                if self._picam is not None:
                    try:
                        self._picam.close()
                    except Exception:
                        pass
                self._picam = None

        if self._picam is None:
            self._cap = cv2.VideoCapture(index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self._cap.isOpened():
                raise RuntimeError(f"Couldn't open webcam {index}")
            self.source = f"webcam {index}"

    def read(self):
        """Return the latest frame as a BGR numpy array, or None on failure."""
        if self._picam is not None:
            return self._picam.capture_array()
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self):
        if self._picam is not None:
            self._picam.stop()
            self._picam.close()
        if self._cap is not None:
            self._cap.release()


# ---------------------------------------------------------------------------
# Preview / debugging
# ---------------------------------------------------------------------------
def preview_score(fire):
    """Rough priority just for the preview labels. targeting.py will own the real one."""
    return fire.area * (1 + max(fire.growth_rate, 0))


def draw_overlay(frame, fires):
    out = frame.copy()
    ranked = sorted([f for f in fires if f.confirmed and f.visible], key=preview_score, reverse=True)
    rank_of = {f.id: i + 1 for i, f in enumerate(ranked)}

    for f in fires:
        if not f.visible:
            continue
        x, y, w, h = f.bbox
        color = (0, 255, 0) if f.confirmed else (160, 160, 160)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        cv2.circle(out, (int(f.cx), int(f.cy)), 3, color, -1)
        label = f"#{f.id} a={int(f.area)} g={f.growth_rate:+.2f}"
        if f.id in rank_of:
            label = f"P{rank_of[f.id]} " + label
        cv2.putText(out, label, (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return out


SLIDERS = [  # (name, params key, max value)
    ("hue min", "hue_min", 179),
    ("hue max", "hue_max", 179),
    ("sat min", "sat_min", 255),
    ("val min", "val_min", 255),
    ("min area", "min_area", 2000),
]


def make_sliders(window, params):
    for name, key, maxv in SLIDERS:
        cv2.createTrackbar(name, window, int(params[key]), maxv, lambda _v: None)


def read_sliders(window, params):
    for name, key, _ in SLIDERS:
        params[key] = cv2.getTrackbarPos(name, window)


def run_image(path):
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit(f"Couldn't read image: {path}")
    params = default_params()
    win = "scanner"
    cv2.namedWindow(win)
    make_sliders(win, params)
    print("Adjust sliders; press q to quit, p to print values.")
    while True:
        read_sliders(win, params)
        detections, mask = detect(frame, params)
        out = frame.copy()
        for d in sorted(detections, key=lambda d: d.area, reverse=True):
            x, y, w, h = d.bbox
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(out, f"a={int(d.area)}", (x, max(y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.imshow(win, out)
        cv2.imshow("mask", mask)
        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            break
        if key == ord("p"):
            print(params)
    cv2.destroyAllWindows()


def run_live(args):
    cam = Camera(index=args.camera, force_webcam=args.webcam, exposure_us=args.exposure)
    print(f"Using {cam.source}")
    tracker = FireTracker()
    params = default_params()

    win = "scanner"
    if not args.headless:
        cv2.namedWindow(win)
        make_sliders(win, params)
        print("Keys: q = quit, p = print slider values, s = save snapshot")

    last_print = 0.0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("Camera returned no frame, stopping.")
                break

            if not args.headless:
                read_sliders(win, params)

            detections, mask = detect(frame, params)
            tracker.update(detections)

            if args.headless:
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    cands = sorted(tracker.candidates(), key=preview_score, reverse=True)
                    if cands:
                        print(" | ".join(
                            f"#{f.id} ({f.cx:.0f},{f.cy:.0f}) a={f.area:.0f} g={f.growth_rate:+.2f}"
                            for f in cands))
                    else:
                        print("no fires")
                continue

            cv2.imshow(win, draw_overlay(frame, tracker.fires))
            cv2.imshow("mask", mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p"):
                print(params)
            if key == ord("s"):
                name = f"snapshot_{int(time.time())}.jpg"
                cv2.imwrite(name, frame)
                print(f"Saved {name}")
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Flame detection test tool")
    ap.add_argument("--image", help="run on a still image instead of a camera")
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX, help="webcam index")
    ap.add_argument("--webcam", action="store_true", help="use webcam even if picamera2 exists")
    ap.add_argument("--headless", action="store_true", help="no windows, print detections")
    ap.add_argument("--exposure", type=int, help="Pi camera fixed exposure in microseconds")
    args = ap.parse_args()

    if args.image:
        run_image(args.image)
    else:
        run_live(args)