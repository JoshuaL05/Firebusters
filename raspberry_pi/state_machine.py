"""
state_machine.py - the brain.

    SCAN -> MOVE -> CONFIRM -> SUPPRESS -> VERIFY -> SCAN
             ^                    |           |
             +-- re-position -----+           +-- still burning: retry (max MAX_ATTEMPTS)
    Any state -> ESTOP while the emergency button is held.

Every tick: read a thermal frame, update fire tracking, read ESP32 telemetry,
then run the current state's handler. Nothing blocks, so the camera keeps
watching all fires the whole time (including while spraying).
"""

import logging
import math
import time
from enum import Enum

import config as cfg
import scanner
from sensor import FlameWindow

log = logging.getLogger("brain")


class State(str, Enum):
    SCAN = "SCAN"
    MOVE = "MOVE"
    CONFIRM = "CONFIRM"
    SUPPRESS = "SUPPRESS"
    VERIFY = "VERIFY"
    ESTOP = "ESTOP"


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FireSuppressor:
    def __init__(self, thermal, hw, targeting, require_flame=cfg.REQUIRE_FLAME_CONFIRM, params=None):
        self.thermal = thermal
        self.hw = hw
        self.targeting = targeting
        self.require_flame = require_flame
        self.params = params or scanner.default_params()

        self.tracker = scanner.FireTracker()
        self.flame = FlameWindow()

        self.state = State.SCAN
        self.state_since = time.monotonic()
        self._entered = False

        self._spraying = False
        self.target_id = None
        self.target_xy = None          # last known position in the thermal image
        self.target_since = 0.0
        self.target_start_score = 0.0
        self.plan = None
        self.confirmed = False
        self.attempts = 0
        self.reaims = 0
        self.blocked = []              # (cx, cy, until): false positives / given-up fires
        self.recent_out = []           # (cx, cy, until): fires we put out (wax stays warm a while)
        self.ambient = 20.0
        self.detections = []
        self.temps_big = None
        self._last_warn = 0.0
        self.stats = {"put_out": 0, "false_alarms": 0, "gave_up": 0}

    # ======================================================================
    # Startup / shutdown
    # ======================================================================
    def start(self):
        if not self.hw.ping():
            raise RuntimeError("ESP32 not responding to PING")
        self.hw.spray(False)
        log.info("Homing gantry...")
        if not self.hw.home():
            raise RuntimeError("Gantry did not home - check both limit switches and the stepper wiring")
        log.info("Homed")
        self.hw.move(cfg.PARK_X_MM, cfg.PARK_Y_MM)
        if not self.targeting.calibrated:
            log.warning("No calibration.json - using rough area mapping. Run calibrate.py for accurate positioning.")
        log.info("Flame confirmation: %s", "ON" if self.require_flame else "OFF (thermal only)")
        self._go(State.SCAN, time.monotonic(), "ready")

    def stop(self):
        self._spray(False)
        self.hw.shutdown()

    # ======================================================================
    # Main tick
    # ======================================================================
    def tick(self):
        temps = self.thermal.read()
        now = time.monotonic()
        if temps is None:
            return

        self.detections, _mask, self.temps_big = scanner.detect(temps, self.params)
        self.tracker.update(self.detections, now)
        self.ambient = scanner.ambient_temp(temps)

        tel = self.hw.telemetry
        self.flame.add(now, tel.flame)

        if not getattr(self.hw, "connected", True):
            if now - self._last_warn > 2:
                log.error("No telemetry from ESP32 - pausing")
                self._last_warn = now
            self._spray(False)
            return

        if tel.estop and self.state != State.ESTOP:
            self._spray(False)
            self._go(State.ESTOP, now, "emergency stop pressed")

        getattr(self, "_" + self.state.value.lower())(now)

    # ======================================================================
    # States
    # ======================================================================
    def _scan(self, now):
        cands = [f for f in self.tracker.candidates() if not self._is_blocked(f, now)]
        if not cands:
            return
        ranked = self.targeting.rank(cands, self._gantry_xy())
        log.info("Fires: " + ", ".join(
            f"#{f.id} a={f.area:.0f} g={f.growth_rate:+.2f} {f.max_temp:.0f}C" for f in ranked))
        self._set_target(ranked[0])
        self.attempts = 0
        self.reaims = 0
        self.confirmed = False
        self._go(State.MOVE, now, f"target #{ranked[0].id}")

    def _move(self, now):
        fire = self._locate()
        if not self._entered:
            cx, cy = (fire.cx, fire.cy) if fire else self.target_xy
            self.plan = self.targeting.plan(cx, cy)
            self.hw.move(self.plan.x_mm, self.plan.y_mm)
            x, y = self.plan.floor_xy
            log.info("Move: fire at floor (%.1f, %.1f) cm -> gantry (%.0f, %.0f) mm",
                     x, y, self.plan.x_mm, self.plan.y_mm)
            self._entered = True

        # The carriage can block the camera's view as it arrives, so only give up on a
        # lost target if we haven't confirmed it yet.
        if fire is None and not self.confirmed and now - self.state_since > cfg.LOST_GRACE_S + 3:
            self._go(State.SCAN, now, "lost target while moving")
            return
        if self.hw.at(self.plan.x_mm, self.plan.y_mm) and now - self.state_since >= cfg.MOVE_SETTLE_S:
            self._go(State.SUPPRESS if self.confirmed else State.CONFIRM, now,
                     "back over the fire" if self.confirmed else "")
        elif now - self.state_since > cfg.MOVE_TIMEOUT_S:
            self._block(now, "couldn't reach target")
            self._go(State.SCAN, now, "move timeout")

    def _confirm(self, now):
        if now - self.state_since < cfg.CONFIRM_S:
            return
        if not self.require_flame:
            self.confirmed = True
            self._go(State.SUPPRESS, now, "confirmed by thermal")
            return
        level = self.flame.mean(since=self.state_since)
        threshold = cfg.FLAME_THRESHOLD
        if level >= threshold:
            self.confirmed = True
            self._go(State.SUPPRESS, now, f"flame sensor {level:.0f} >= {threshold}")
        else:
            self._park()
            if self._near_recent_out(now):
                # Melted wax / hot surface left over from a fire we already put out.
                self._block(now, "leftover heat")
                self._go(State.SCAN, now, f"just leftover heat from a fire we put out (flame {level:.0f})")
            else:
                self.stats["false_alarms"] += 1
                self._block(now, "false positive")
                self._go(State.SCAN, now, f"FALSE POSITIVE: hot but no flame ({level:.0f} < {threshold})")

    def _suppress(self, now):
        fire = self._locate()
        if not self._entered:
            self._spray(True)
            self.best_heat = self._heat(fire) if fire else math.inf
            self.best_flame = self.flame.mean(since=now - 0.5)
            self.last_progress = now
            self.out_since = None
            self._entered = True

        # --- progress: is the heat (thermal) or flame (sensor) dropping? ---
        flame_now = self.flame.mean(since=now - 0.5)
        if fire is not None:
            heat = self._heat(fire)
            if heat < self.best_heat * (1 - cfg.PROGRESS_MIN_DROP):
                self.best_heat = heat
                self.last_progress = now
        if self.require_flame and flame_now < self.best_flame * (1 - cfg.PROGRESS_MIN_DROP):
            self.best_flame = flame_now
            self.last_progress = now

        # --- out? ---
        # With the flame sensor (right above the fire): no flame = out, even if the
        # spot is still warm (melted wax). Without it: the hot spot has to disappear.
        if self.require_flame:
            looks_out = flame_now < cfg.FLAME_OUT_THRESHOLD
        else:
            looks_out = not self._visible_near_target()
        if looks_out:
            self.out_since = self.out_since or now
            if now - self.out_since >= cfg.OUT_CONFIRM_S:
                self._spray(False)
                self._go(State.VERIFY, now, "looks out")
                return
        else:
            self.out_since = None

        # --- small circles over the fire to cover the whole thing ---
        a = 2 * math.pi * (now - self.state_since) / cfg.SWEEP_PERIOD_S
        self.hw.move(_clamp(self.plan.x_mm + cfg.SWEEP_MM * math.cos(a), 0, cfg.X_TRAVEL_MM),
                     _clamp(self.plan.y_mm + cfg.SWEEP_MM * math.sin(a), 0, cfg.Y_TRAVEL_MM),
                     wait=False)

        # --- another fire got much worse? switch ---
        # Compare against this fire's size when we STARTED on it: it's supposed to
        # shrink while we spray, and that alone shouldn't make us abandon it.
        if fire is not None and now - self.target_since >= cfg.MIN_COMMIT_S:
            mine = max(self.targeting.score(fire), self.target_start_score)
            others = [f for f in self.tracker.candidates()
                      if f.id != fire.id and not self._is_blocked(f, now)]
            if others:
                best = max(others, key=self.targeting.score)
                if self.targeting.score(best) > cfg.SWITCH_RATIO * mine:
                    self._spray(False)
                    self._set_target(best)
                    self.confirmed = False
                    self.attempts = 0
                    self.reaims = 0
                    self._go(State.MOVE, now, f"switching to much bigger fire #{best.id}")
                    return

        # --- spraying isn't helping -> position is probably off, re-position ---
        if now - self.last_progress > cfg.NO_PROGRESS_S:
            self._spray(False)
            self.reaims += 1
            if self.reaims > cfg.MAX_REAIMS:
                self.stats["gave_up"] += 1
                self._block(now, "no progress after re-positioning")
                log.warning("Can't make progress on this fire - moving on")
                self._park()
                self._go(State.SCAN, now, "gave up on fire")
            else:
                self._go(State.MOVE, now, f"no progress, re-position {self.reaims}/{cfg.MAX_REAIMS}")

    def _verify(self, now):
        # With the flame sensor: stay over the spot and ask it "any flame left?".
        #   (The camera can't tell a relit candle from warm wax, the flame sensor can.)
        # Thermal-only: park out of the camera's way, then look for the hot spot.
        park = cfg.VERIFY_PARK and not self.require_flame
        if not self._entered:
            if park:
                self._park()
            self.verify_from = None
            self._entered = True
        if self.verify_from is None:
            if not park or self.hw.at(cfg.PARK_X_MM, cfg.PARK_Y_MM) \
                    or now - self.state_since > cfg.MOVE_TIMEOUT_S:
                self.verify_from = now
            return
        if now - self.verify_from < cfg.VERIFY_WAIT_S:
            return

        if self.require_flame:
            still_burning = self.flame.mean(since=now - 0.5) >= cfg.FLAME_THRESHOLD
        else:
            still_burning = any(_dist(d.cx, d.cy, *self.target_xy) < cfg.REACQUIRE_PX
                                for d in self.detections) if self.target_xy else False
        if still_burning:
            self.attempts += 1
            if self.attempts >= cfg.MAX_ATTEMPTS:
                self.stats["gave_up"] += 1
                self._block(now, "reignites")
                log.warning("Fire keeps reigniting - moving on")
                self._go(State.SCAN, now, "gave up on fire")
            else:
                self.confirmed = True
                self.reaims = 0
                self._go(State.MOVE, now, f"still burning, attempt {self.attempts + 1}/{cfg.MAX_ATTEMPTS}")
            return
        self.stats["put_out"] += 1
        if self.target_id is not None:
            self.tracker.forget(self.target_id)
        if self.target_xy:
            self.recent_out.append((*self.target_xy, now + cfg.WARM_SPOT_S))
        log.info("FIRE OUT (total %d)", self.stats["put_out"])
        self.target_id = None
        self._park()
        self._go(State.SCAN, now, "fire out - rescanning")

    def _estop(self, now):
        self._spray(False)
        if not self.hw.telemetry.estop:
            self._go(State.SCAN, now, "emergency stop released")

    # ======================================================================
    # Helpers
    # ======================================================================
    def _go(self, state, now, why=""):
        if state != self.state or why:
            log.info("%s -> %s%s", self.state.value, state.value, f"  ({why})" if why else "")
        self.state = state
        self.state_since = now
        self._entered = False

    def _spray(self, on):
        if on != self._spraying:
            self.hw.spray(on)
            self._spraying = on

    def _park(self):
        self.hw.move(cfg.PARK_X_MM, cfg.PARK_Y_MM, wait=False)

    def _gantry_xy(self):
        t = self.hw.telemetry
        return t.x_mm, t.y_mm

    def _set_target(self, fire):
        self.target_id = fire.id
        self.target_xy = (fire.cx, fire.cy)
        self.target_since = time.monotonic()
        self.target_start_score = self.targeting.score(fire)

    def _locate(self):
        """The current target fire, re-acquiring by position if its tracker ID changed."""
        for f in self.tracker.fires:
            if f.id == self.target_id and f.visible:
                self.target_xy = (f.cx, f.cy)
                return f
        if self.target_xy is None:
            return None
        best, best_d = None, cfg.REACQUIRE_PX
        for f in self.tracker.fires:
            if f.visible:
                d = _dist(f.cx, f.cy, *self.target_xy)
                if d < best_d:
                    best, best_d = f, d
        if best:
            self.target_id = best.id
            self.target_xy = (best.cx, best.cy)
        return best

    def _visible_near_target(self):
        if self.target_xy is None:
            return False
        return any(_dist(d.cx, d.cy, *self.target_xy) < cfg.REACQUIRE_PX for d in self.detections)

    def _heat(self, fire):
        """How much heat a fire gives off: area x how far above room temperature."""
        return fire.area * max(fire.mean_temp - self.ambient, 0.0)

    def _block(self, now, why):
        if self.target_xy:
            self.blocked.append((*self.target_xy, now + cfg.FALSE_POSITIVE_COOLDOWN_S))
            log.info("Ignoring that spot for %.0fs (%s)", cfg.FALSE_POSITIVE_COOLDOWN_S, why)

    def _near_recent_out(self, now):
        self.recent_out = [r for r in self.recent_out if r[2] > now]
        return self.target_xy is not None and any(
            _dist(*self.target_xy, rx, ry) < cfg.BLOCK_RADIUS_PX for rx, ry, _ in self.recent_out)

    def _is_blocked(self, fire, now):
        self.blocked = [b for b in self.blocked if b[2] > now]
        return any(_dist(fire.cx, fire.cy, bx, by) < cfg.BLOCK_RADIUS_PX for bx, by, _ in self.blocked)

    def status(self):
        tel = self.hw.telemetry
        tgt = f"#{self.target_id}" if self.target_id is not None and self.state != State.SCAN else "-"
        return (f"[{self.state.value:8}] fires={len(self.tracker.candidates())} target={tgt} "
                f"flame={tel.flame:4d} gantry=({tel.x_mm:3.0f},{tel.y_mm:3.0f})mm "
                f"out={self.stats['put_out']} false={self.stats['false_alarms']}")
