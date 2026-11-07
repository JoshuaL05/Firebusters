"""
calibrate.py - line up the thermal camera with the real floor, then check the aim.

Step 1 (camera -> floor), needs only the MLX90640:
    python3 calibrate.py
  It asks you to put a hot object (a mug of hot water works) at 4 marked
  spots on the floor, one at a time. It finds the object in the thermal image
  each time and saves calibration.json, which main.py loads automatically.
  Mark the 4 spots with tape first; they're CALIBRATION_POINTS_CM in config.py.

Step 2 (floor -> nozzle), needs the ESP32 too:
    python3 calibrate.py --aim-test
  Type floor coordinates (e.g. "30 20") and the nozzle points there. If it's
  consistently off, adjust PAN_TRIM_DEG / TILT_TRIM_DEG / TILT_LOB_DEG in
  config.py. Type "spray" to test-spray for 0.5 s, "q" to quit.
"""

import argparse
import json
import time

import cv2
import numpy as np

import config as cfg
import scanner
from targeting import Targeting


def locate_hot_object(cam, params, seconds=2.0):
    """Average position of the hottest blob over a couple of seconds."""
    xs, ys = [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        temps = cam.read()
        if temps is None:
            continue
        dets, _, _ = scanner.detect(temps, params)
        if dets:
            hottest = max(dets, key=lambda d: d.max_temp)
            xs.append(hottest.cx)
            ys.append(hottest.cy)
    if len(xs) < 3:
        return None
    return float(np.median(xs)), float(np.median(ys))


def calibrate_camera(args):
    cam = scanner.ThermalCamera()
    params = scanner.default_params()
    params["fire_temp"] = args.fire_temp
    params["ambient_delta"] = 10
    pixels, floor = [], []
    try:
        for x, y in cfg.CALIBRATION_POINTS_CM:
            while True:
                input(f"\nPut the hot object on the spot at x={x} cm, y={y} cm, step away, then press Enter...")
                found = locate_hot_object(cam, params)
                if found:
                    print(f"  found it at pixel ({found[0]:.0f}, {found[1]:.0f})")
                    pixels.append(found)
                    floor.append((x, y))
                    break
                print(f"  Couldn't see anything above {args.fire_temp} C. Hotter object, or try --fire-temp 35.")
    finally:
        cam.close()

    H = cv2.getPerspectiveTransform(np.array(pixels, np.float32), np.array(floor, np.float32))
    with open(cfg.CALIBRATION_FILE, "w") as f:
        json.dump({"homography": H.tolist(), "pixels": pixels, "floor_cm": floor,
                   "created": time.strftime("%Y-%m-%d %H:%M")}, f, indent=2)
    print(f"\nSaved {cfg.CALIBRATION_FILE}")

    t = Targeting()
    print("Check (should match the spots you used):")
    for (px, py), (x, y) in zip(pixels, floor):
        fx, fy = t.pixel_to_floor(px, py)
        print(f"  pixel ({px:.0f},{py:.0f}) -> ({fx:.1f}, {fy:.1f}) cm   expected ({x}, {y})")


def aim_test(args):
    from motion import ESP32Link
    t = Targeting(rail_enabled=cfg.RAIL_ENABLED and not args.no_rail)
    hw = ESP32Link(port=args.port, rail_enabled=t.rail_enabled)
    try:
        if t.rail_enabled and not hw.home():
            print("Rail didn't home - check the limit switch. Use --no-rail to skip it.")
            return
        print('Type "x y" in cm to aim, "spray" to test-spray, "q" to quit.')
        while True:
            cmd = input("> ").strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd == "spray":
                hw.spray(True)
                time.sleep(0.5)
                hw.spray(False)
                continue
            try:
                x, y = (float(v) for v in cmd.replace(",", " ").split())
            except ValueError:
                print('  expected two numbers, e.g. "30 20"')
                continue
            plan = t.plan_floor(x, y)
            hw.move_rail(plan.rail_mm)
            deadline = time.monotonic() + 15
            while not hw.rail_done(plan.rail_mm) and time.monotonic() < deadline:
                time.sleep(0.05)
            hw.aim(plan.pan, plan.tilt)
            print(f"  rail {plan.rail_mm:.0f} mm, pan {plan.pan:.1f}, tilt {plan.tilt:.1f}")
    finally:
        hw.shutdown()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Calibrate camera and aim")
    ap.add_argument("--aim-test", action="store_true", help="interactive aim test (needs ESP32)")
    ap.add_argument("--fire-temp", type=float, default=40.0, help="detection threshold for the hot object")
    ap.add_argument("--port", help="ESP32 serial port")
    ap.add_argument("--no-rail", action="store_true")
    a = ap.parse_args()
    aim_test(a) if a.aim_test else calibrate_camera(a)
