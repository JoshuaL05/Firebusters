"""
main.py - start the whole system.

  python3 main.py                       # real hardware (MLX90640 + ESP32)
  python3 main.py --web                 # ...plus live view at http://heatseek.local:8080
  python3 main.py --sim --web           # simulated fires, live view in the browser
  python3 main.py --sim --show          # laptop: simulated fires in a desktop window
  python3 main.py --no-flame-confirm    # testing with hot mugs (no real flame)

Ctrl+C stops cleanly: spray off, gantry stops.
"""

import argparse
import logging
import os
import signal
import sys
import time

import cv2

import config as cfg
import scanner
from state_machine import FireSuppressor
from targeting import Targeting


def parse_args():
    ap = argparse.ArgumentParser(description="FireBusters fire suppression")
    ap.add_argument("--sim", action="store_true", help="simulate camera, fires and ESP32")
    ap.add_argument("--web", action="store_true", help="live view in a web browser (port 8080)")
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--show", action="store_true", help="live view in a desktop window (needs a screen)")
    ap.add_argument("--no-flame-confirm", action="store_true", help="thermal-only (hot-mug testing)")
    ap.add_argument("--call", action="store_true",
                    help="really phone the number in .env when a fire is confirmed")
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
    targeting = Targeting()
    if args.sim:
        from sim import SimESP32, SimThermal, SimWorld
        world = SimWorld(targeting)
        thermal, hw = SimThermal(world), SimESP32(world)
    else:
        from motion import ESP32Link
        thermal = scanner.ThermalCamera()
        hw = ESP32Link(port=args.port)

    params = scanner.default_params()
    if args.fire_temp is not None:
        params["fire_temp"] = args.fire_temp
    require_flame = cfg.REQUIRE_FLAME_CONFIRM and not args.no_flame_confirm
    from alerts import Alerter
    alerter = Alerter(enabled=args.call)
    return FireSuppressor(thermal, hw, targeting, require_flame=require_flame, params=params,
                          alerter=alerter)


def render(machine):
    """Heat map with fire boxes, target ring, nozzle marker and state. None until the first frame."""
    if machine.temps_big is None:
        return None
    t = machine.targeting
    grid = t.sector_grid_pixels() if cfg.SHOW_SECTOR_GRID else None
    img = scanner.draw_overlay(machine.temps_big, machine.tracker.fires, grid=grid,
                               sector_of=lambda f: t.sector_of_pixel(f.cx, f.cy))
    img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    if machine.target_xy and machine.state.value != "SCAN":
        cx, cy = (int(v * 2) for v in machine.target_xy)
        cv2.circle(img, (cx, cy), 24, (255, 255, 0), 2)
    tel = machine.hw.telemetry
    nx, ny = machine.targeting.floor_to_pixel(*machine.targeting.gantry_to_floor(tel.x_mm, tel.y_mm))
    nozzle_color = (255, 128, 0) if tel.spraying else (200, 200, 200)
    cv2.drawMarker(img, (int(nx * 2), int(ny * 2)), nozzle_color, cv2.MARKER_TILTED_CROSS, 18, 2)
    color = (0, 0, 255) if machine.state.value in ("SUPPRESS", "ESTOP") else (255, 255, 255)
    cv2.putText(img, machine.state.value, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    cv2.putText(img, f"fires out: {machine.stats['put_out']}",
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def web_status(machine):
    tel = machine.hw.telemetry
    busy = machine.target_id is not None and machine.state.value != "SCAN"
    return {
        "state": machine.state.value,
        "fires": len(machine.tracker.candidates()),
        "target": f"#{machine.target_id} in {machine.target_sector()}" if busy else "-",
        "flame": int(tel.flame),
        "pos": f"{tel.x_mm:.0f}, {tel.y_mm:.0f}",
        "out": machine.stats["put_out"],
        "false": machine.stats["false_alarms"],
        "alert": machine.alerter.status if machine.alerter else "-",
        "events": list(machine.events)[-20:],     # spoken by the page (voice narration)
    }


def main():
    args = parse_args()
    setup_logging(args)
    log = logging.getLogger("main")

    # Over SSH there's no screen; opening a window would crash the program.
    if args.show and sys.platform.startswith("linux") \
            and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        log.warning("--show needs a screen (none found - are you on SSH?). Use --web instead. "
                    "Running without the window.")
        args.show = False

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    web = None
    if args.web:
        from web import WebView
        try:
            web = WebView(args.web_port)
        except OSError:
            log.error("Port %d is busy - is another copy of main.py still running (check `tmux attach`)? "
                      "Stop it, or use --web-port 8081. Continuing without the live view.", args.web_port)

    try:
        machine = build(args)
        machine.start()
    except RuntimeError as e:
        log.error("Can't start: %s", e)
        if web:
            web.close()
        sys.exit(1)
    started = last_status = time.monotonic()
    try:
        while True:
            machine.tick()
            now = time.monotonic()
            if now - last_status >= 1.0:
                last_status = now
                log.info(machine.status())
            if args.show or web:
                img = render(machine)
                if img is not None:
                    if web:
                        web.update(img, web_status(machine))
                    if args.show:
                        cv2.imshow("FireBusters", img)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break
            if args.duration and now - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down: spray off, gantry stop")
        machine.stop()
        try:
            machine.thermal.close()
        except Exception:
            pass
        if web:
            web.close()
        if args.show:
            cv2.destroyAllWindows()
        log.info("Results: %s", machine.stats)


if __name__ == "__main__":
    main()
