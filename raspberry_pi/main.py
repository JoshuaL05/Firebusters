"""
Main Program entry point

"""
"""
main.py - start the whole system.

  python3 main.py                       # real hardware (MLX90640 + ESP32)
  python3 main.py --sim --show          # laptop: simulated fires, live heat map window
  python3 main.py --sim                 # laptop/SSH: simulated, text status only
  python3 main.py --no-flame-confirm    # testing with hot mugs (no real flame)
  python3 main.py --no-rail             # pan-tilt only, no stepper rail

Ctrl+C stops cleanly: spray off, nozzle to rest.
"""

import argparse
import logging
import signal
import sys
import time

import config as cfg
import scanner
from state_machine import FireSuppressor
from targeting import Targeting


def parse_args():
    ap = argparse.ArgumentParser(description="FireBusters fire suppression")
    ap.add_argument("--sim", action="store_true", help="simulate camera, fires and ESP32")
    ap.add_argument("--show", action="store_true", help="show live heat-map window (needs a screen)")
    ap.add_argument("--no-flame-confirm", action="store_true", help="thermal-only (hot-mug testing)")
    ap.add_argument("--no-rail", action="store_true", help="no stepper rail, pan-tilt only")
    ap.add_argument("--port", help="ESP32 serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--fire-temp", type=float, help=f"fire threshold C (default {cfg.FIRE_TEMP_C})")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--log-file", help="also write the log to this file")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap.parse_args()


def setup_logging(args):
    handlers = [logging.StreamHandler(sys.stdout)]
    if args.log_file:
        handlers.append(logging.FileHandler(args.log_file))
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)-7s %(message)s", datefmt="%H:%M:%S",
                        handlers=handlers)


def build(args):
    targeting = Targeting(rail_enabled=cfg.RAIL_ENABLED and not args.no_rail)
    if args.sim:
        from sim import SimESP32, SimThermal, SimWorld
        world = SimWorld(targeting)
        thermal, hw = SimThermal(world), SimESP32(world, targeting.rail_enabled)
    else:
        from motion import ESP32Link
        thermal = scanner.ThermalCamera()
        hw = ESP32Link(port=args.port, rail_enabled=targeting.rail_enabled)

    params = scanner.default_params()
    if args.fire_temp is not None:
        params["fire_temp"] = args.fire_temp
    require_flame = cfg.REQUIRE_FLAME_CONFIRM and not args.no_flame_confirm
    return FireSuppressor(thermal, hw, targeting, require_flame=require_flame, params=params)


def draw(machine):
    import cv2
    if machine.temps_big is None:
        return True
    img = scanner.draw_overlay(machine.temps_big, machine.tracker.fires)
    img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    if machine.target_xy and machine.state.value != "SCAN":
        cx, cy = (int(v * 2) for v in machine.target_xy)
        cv2.circle(img, (cx, cy), 24, (255, 255, 0), 2)
    color = (0, 0, 255) if machine.state.value in ("SUPPRESS", "ESTOP") else (255, 255, 255)
    cv2.putText(img, machine.state.value, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    cv2.putText(img, f"fires out: {machine.stats['put_out']}",
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.imshow("FireBusters", img)
    return (cv2.waitKey(1) & 0xFF) != ord("q")


def main():
    args = parse_args()
    setup_logging(args)
    log = logging.getLogger("main")

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    machine = build(args)
    machine.start()
    started = last_status = time.monotonic()
    try:
        while True:
            machine.tick()
            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                log.info(machine.status())
            if args.show and not draw(machine):
                break
            if args.duration and now - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down: spray off, nozzle to rest")
        machine.stop()
        try:
            machine.thermal.close()
        except Exception:
            pass
        if args.show:
            import cv2
            cv2.destroyAllWindows()
        log.info("Results: %s", machine.stats)


if __name__ == "__main__":
    main()
