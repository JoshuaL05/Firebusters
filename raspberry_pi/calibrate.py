"""
calibrate.py - line up the thermal camera with the real floor, then check the aim.

Step 1 (camera -> floor), needs only the MLX90640:
    python3 calibrate.py
  It asks you to put a hot object (a mug of hot water works) at 4 marked
  spots on the floor, one at a time. It finds the object in the thermal image
  each time and saves calibration.json, which main.py loads automatically.
  Mark the 4 spots with tape first; they're CALIBRATION_POINTS_CM in config.py.

Step 2 (floor -> gantry), needs the ESP32 too:
    python3 calibrate.py --aim-test
  Homes the gantry, then:
    "0 0"      -> drive to gantry home. Measure the floor spot under the nozzle and
                  put it in HOME_FLOOR_X_CM / HOME_FLOOR_Y_CM in config.py.
    "15 15"    -> drive so the nozzle is over floor point (15, 15) cm. Check it lines up
                  with a taped spot.
    "g 100 50" -> drive to raw gantry position (100, 50) mm.
    "spray"    -> test-spray for 0.5 s.     "q" -> quit.
"""

import argparse
import json
import logging
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
    t = Targeting()
    hw = ESP32Link(port=args.port)
    try:
        print("Homing... (both axes should drive toward their endstops; gives up after 60 s)")
        if not hw.home():
            t = hw.telemetry
            print("Gantry didn't home. Last status: x=%.1f y=%.1f moving=%s homed=%s estop=%s"
                  % (t.x_mm, t.y_mm, t.moving, t.homed, t.estop))
            print("  estop=True  -> e-stop is open (or not fitted: set ESTOP_FITTED=false in main.cpp)")
            print("  nothing moved -> check 24 V, A4988 RESET-SLEEP jumper, current limit, and the SW command")
            return
        print('Type "x y" (floor cm), "g x y" (gantry mm), "spray", or "q".')
        while True:
            cmd = input("> ").strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd == "spray":
                hw.spray(True)
                time.sleep(0.5)
                hw.spray(False)
                continue
            raw = cmd.startswith("g ")
            try:
                a, b = (float(v) for v in cmd.lstrip("g ").replace(",", " ").split())
            except ValueError:
                print('  expected two numbers, e.g. "15 15" or "g 100 50"')
                continue
            if raw:
                gx, gy = a, b
            else:
                plan = t.plan_floor(a, b)
                gx, gy = plan.x_mm, plan.y_mm
            if not hw.move(gx, gy):
                print("  ESP32 refused the move (see the warning above: not_homed or estop)")
                continue
            deadline = time.monotonic() + 20
            while not hw.at(gx, gy) and time.monotonic() < deadline:
                time.sleep(0.05)
            tel = hw.telemetry
            if not hw.at(gx, gy):
                print(f"  didn't get there: ESP32 thinks it's at ({tel.x_mm:.0f}, {tel.y_mm:.0f}) mm, "
                      f"moving={tel.moving}")
            fx, fy = t.gantry_to_floor(gx, gy)
            print(f"  gantry ({gx:.0f}, {gy:.0f}) mm  -> nozzle should be over floor ({fx:.1f}, {fy:.1f}) cm")
            print("  (if the ESP32 reached it but nothing physically moved: motor power / A4988 problem)")
    finally:
        hw.shutdown()


if __name__ == "__main__":
    # show messages from the ESP32 (e.g. "X homed", "homing timeout") while we work
    logging.basicConfig(level=logging.INFO, format="  %(name)s: %(message)s")
    ap = argparse.ArgumentParser(description="Calibrate camera and aim")
    ap.add_argument("--aim-test", action="store_true", help="interactive aim test (needs ESP32)")
    ap.add_argument("--fire-temp", type=float, default=40.0, help="detection threshold for the hot object")
    ap.add_argument("--port", help="ESP32 serial port")
    a = ap.parse_args()
    aim_test(a) if a.aim_test else calibrate_camera(a)
