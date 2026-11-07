"""
motion.py - talks to the ESP32 over USB serial.

Pi -> ESP32 (one command per line):
    PING                 -> PONG
    HOME                 -> OK HOME   (then telemetry shows homed=1 when both axes are done)
    MOVE <x_mm> <y_mm>   -> OK MOVE   (telemetry shows moving=1 until it arrives)
    SPRAY <0|1>          -> OK SPRAY
    STOP                 -> OK STOP   (spray off, gantry stops)
    HB                   (heartbeat, no reply; ESP32 stops spray + motion after 1 s without any command)
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
    def __init__(self, port=None, baud=cfg.SERIAL_BAUD):
        import serial

        self.port = port or cfg.SERIAL_PORT or find_port()
        if not self.port:
            raise RuntimeError("No ESP32 serial port found. Is it plugged in? Try --port /dev/ttyUSB0")
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

    def home(self, timeout=60.0):
        """Home both axes against their limit switches. Returns True when homed."""
        if not self.command("HOME"):
            return False
        deadline = time.monotonic() + timeout
        time.sleep(0.2)
        while time.monotonic() < deadline:
            if self.telemetry.homed and not self.telemetry.moving:
                return True
            time.sleep(0.1)
        return False

    def move(self, x_mm, y_mm, wait=True):
        return self.command(f"MOVE {x_mm:.1f} {y_mm:.1f}", wait=wait)

    def at(self, x_mm, y_mm):
        """True once the carriage has stopped at (x_mm, y_mm)."""
        t = self.telemetry
        return (not t.moving and abs(t.x_mm - x_mm) <= cfg.POS_TOLERANCE_MM
                and abs(t.y_mm - y_mm) <= cfg.POS_TOLERANCE_MM)

    def spray(self, on):
        return self.command(f"SPRAY {1 if on else 0}")

    @property
    def connected(self):
        return time.monotonic() - self.telemetry.stamp < cfg.TELEMETRY_STALE_S

    def shutdown(self):
        """Spray off, stop the gantry, close the port."""
        for cmd in ("SPRAY 0", "STOP"):
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
