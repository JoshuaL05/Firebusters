"""
motion.py - talks to the ESP32 over USB serial.

Pi -> ESP32 (one command per line):
    PING                 -> PONG
    HOME                 -> OK HOME   (then telemetry shows homed=1 when done)
    MOVE <mm>            -> OK MOVE   (rail; telemetry shows moving until it arrives)
    AIM <pan> <tilt>     -> OK AIM
    SPRAY <0|1>          -> OK SPRAY
    STOP                 -> OK STOP   (spray off, rail stop)
    HB                   (heartbeat, no reply; ESP32 shuts spray off after 1 s without any command)
Errors come back as:  ERR <COMMAND> <reason>
ESP32 -> Pi also streams telemetry ("T ...", see sensor.py) and "LOG ..." messages.

Anything with the same methods (see sim.SimESP32) can stand in for ESP32Link.
"""

import logging
import queue
import threading
import time

import config as cfg
from sensor import Telemetry, parse_telemetry

log = logging.getLogger("motion")


def find_port():
    """Best guess at the ESP32's serial port."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        desc = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("cp210", "ch340", "ch910", "usb serial", "esp32", "silicon labs")):
            return p.device
    for p in ports:
        if "ttyusb" in p.device.lower() or "ttyacm" in p.device.lower():
            return p.device
    return None


class ESP32Link:
    def __init__(self, port=None, baud=cfg.SERIAL_BAUD, rail_enabled=cfg.RAIL_ENABLED):
        import serial

        self.port = port or cfg.SERIAL_PORT or find_port()
        if not self.port:
            raise RuntimeError("No ESP32 serial port found. Is it plugged in? Try --port /dev/ttyUSB0")
        self.rail_enabled = rail_enabled
        self._ser = serial.Serial(self.port, baud, timeout=0.1)
        time.sleep(2.0)                    # opening the port resets the ESP32; let it boot
        self._ser.reset_input_buffer()

        self.telemetry = Telemetry()
        self._acks = queue.Queue()
        self._write_lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        log.info("ESP32 on %s", self.port)

    # ---- background threads ------------------------------------------------
    def _reader(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except Exception as e:  # unplugged, etc.
                log.error("Serial read failed: %s", e)
                break
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if line.startswith("T "):
                t = parse_telemetry(line, time.monotonic())
                if t:
                    self.telemetry = t
            elif line.startswith(("OK", "ERR", "PONG")):
                self._acks.put(line)
            elif line.startswith("LOG"):
                log.info("ESP32: %s", line[4:])
            elif line:
                log.debug("ESP32 (unparsed): %s", line)

    def _heartbeat(self):
        while self._running:
            try:
                self._write("HB")
            except Exception:
                pass
            time.sleep(cfg.HEARTBEAT_S)

    def _write(self, text):
        with self._write_lock:
            self._ser.write((text + "\n").encode())

    # ---- commands ------------------------------------------------------------
    def command(self, text, wait=True, timeout=cfg.ACK_TIMEOUT_S):
        name = text.split()[0].upper()
        if wait:
            while not self._acks.empty():      # throw away stale replies
                self._acks.get_nowait()
        self._write(text)
        if not wait:
            return True
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                line = self._acks.get(timeout=remaining)
            except queue.Empty:
                break
            parts = line.split()
            if parts[0] == "PONG" and name == "PING":
                return True
            if len(parts) >= 2 and parts[1] == name:
                if parts[0] == "OK":
                    return True
                log.warning("ESP32 refused %r: %s", text, line)
                return False
        log.warning("No reply from ESP32 to %r", text)
        return False

    def ping(self):
        return self.command("PING")

    def home(self, timeout=30.0):
        """Home the rail against its limit switch. Returns True when homed."""
        if not self.rail_enabled:
            return True
        if not self.command("HOME"):
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.telemetry.homed and not self.telemetry.moving:
                return True
            time.sleep(0.1)
        return False

    def move_rail(self, mm):
        if not self.rail_enabled:
            return True
        return self.command(f"MOVE {mm:.1f}")

    def rail_done(self, mm):
        if not self.rail_enabled:
            return True
        t = self.telemetry
        return not t.moving and abs(t.rail_mm - mm) <= cfg.RAIL_TOLERANCE_MM

    def aim(self, pan, tilt, wait=True):
        return self.command(f"AIM {pan:.1f} {tilt:.1f}", wait=wait)

    def spray(self, on):
        return self.command(f"SPRAY {1 if on else 0}")

    @property
    def connected(self):
        return time.monotonic() - self.telemetry.stamp < cfg.TELEMETRY_STALE_S

    def shutdown(self):
        """Spray off, nozzle to rest, close the port."""
        for cmd in ("SPRAY 0", f"AIM {cfg.REST_PAN_DEG:.1f} {cfg.REST_TILT_DEG:.1f}"):
            try:
                self.command(cmd, timeout=0.5)
            except Exception:
                pass
        self._running = False
        time.sleep(0.2)
        try:
            self._ser.close()
        except Exception:
            pass
