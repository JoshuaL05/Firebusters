"""
alerts.py - phone call when a fire is confirmed (via Twilio).

The idea: in a real warehouse this would call the fire department / building
security. For the demo it calls a teammate's phone.

Setup (one time):
  1. Make a free Twilio account at twilio.com and verify your own phone number
     (trial accounts can only call verified numbers).
  2. Get a Twilio phone number (free on the trial) from the Twilio console.
  3. Copy .env.example to .env (next to this file) and fill in your Account SID,
     Auth Token, Twilio number and your phone. .env is in .gitignore - never commit it.
     (Real environment variables with the same names override the file.)
  4. Run with --call:   python3 main.py --call
     Without --call it only logs what it WOULD say (safe for testing).

Calls happen in the background so the fire-fighting loop never waits on the network.
"""

import base64
import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from xml.sax.saxutils import escape

import config as cfg

log = logging.getLogger("alert")

SECRET_FILE = os.path.join(cfg.HERE, ".env")
EMERGENCY_NUMBERS = {"911", "112", "999", "000", "110", "119"}   # never auto-dial these


def _read_env_file(path):
    """Minimal .env reader (KEY=value per line, # comments, optional quotes). No extra packages."""
    values = {}
    if not os.path.exists(path):
        return values
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip().removeprefix("export ").strip()
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            elif " #" in value:
                value = value.split(" #", 1)[0].strip()    # inline comment
            values[key] = value
    return values


def _load_secrets():
    data = _read_env_file(SECRET_FILE)

    def get(name):
        return os.environ.get(name) or data.get(name, "")

    return {
        "sid": get("TWILIO_ACCOUNT_SID"),
        "token": get("TWILIO_AUTH_TOKEN"),
        "from": get("TWILIO_FROM"),
        "to": get("ALERT_TO"),
    }


class Alerter:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.secrets = _load_secrets()
        # An "incident" starts with the first confirmed fire and ends after INCIDENT_CLEAR_S
        # with no confirmed fire activity. Exactly one call per incident.
        self.incident_active = False
        self.incident_count = 0
        self.last_activity = 0.0
        self.status = "standby" if enabled else "dry run"
        if enabled:
            missing = [k for k in ("sid", "token", "from", "to") if not self.secrets[k]]
            if missing:
                raise RuntimeError(
                    f"--call needs Twilio details ({', '.join(missing)} missing). "
                    f"Fill in {SECRET_FILE} (copy .env.example to .env).")
            digits = "".join(c for c in self.secrets["to"] if c.isdigit())
            if digits in EMERGENCY_NUMBERS or len(digits) < 7:
                raise RuntimeError("Refusing to auto-dial an emergency/short number. Use your own phone for the demo.")
            log.info("Phone alerts ON - will call %s when a fire is confirmed", self._masked())

    def _masked(self):
        n = self.secrets["to"]
        return "*" * max(0, len(n) - 4) + n[-4:]

    def fire_activity(self, now=None):
        """Call while a confirmed fire is being fought; keeps the current incident open."""
        self.last_activity = time.monotonic() if now is None else now

    def update(self, now=None):
        """Call every tick; closes the incident once things have been quiet long enough."""
        now = time.monotonic() if now is None else now
        if self.incident_active and now - self.last_activity >= cfg.INCIDENT_CLEAR_S:
            self.incident_active = False
            log.info("Incident #%d over (no confirmed fires for %.0f s) - next fire starts a new incident",
                     self.incident_count, cfg.INCIDENT_CLEAR_S)
            self.status = "standby" if self.enabled else "dry run"

    def fire_confirmed(self, sector, active_fires, now=None):
        """A fire was confirmed in `sector` (e.g. "B2"). Calls only if this starts a NEW incident."""
        now = time.monotonic() if now is None else now
        self.update(now)                # close the previous incident first if it's been quiet
        self.fire_activity(now)
        if self.incident_active:
            return                      # same incident, already called
        self.incident_active = True
        self.incident_count += 1
        log.info("Incident #%d started (sector %s)", self.incident_count, sector)
        spoken_sector = " ".join(sector)          # "B2" -> "B 2" so it's read as letter + number
        others = active_fires - 1
        # Kept short on purpose: Twilio bills every started minute, so the whole call
        # (trial announcement + message said twice) must stay well under 60 s.
        message = (
            f"Fire Busters alert. Fire confirmed in the {cfg.SITE_NAME}, sector {spoken_sector}. "
            + (f"{others} more possible fire{'s' if others != 1 else ''}. " if others > 0 else "")
            + "Suppression in progress."
        )
        if not self.enabled:
            log.info("(dry run, use --call to really call) would say: %s", message)
            self.status = time.strftime("would call %H:%M:%S")
            return
        self.status = "calling..."
        threading.Thread(target=self._call, args=(message,), daemon=True).start()

    def _call(self, message):
        s = self.secrets
        twiml = (f'<Response><Say voice="alice">{escape(message)}</Say>'
                 f'<Say voice="alice">Repeat. {escape(message)}</Say><Hangup/></Response>')
        url = f"https://api.twilio.com/2010-04-01/Accounts/{s['sid']}/Calls.json"
        body = urllib.parse.urlencode({
            "To": s["to"], "From": s["from"], "Twiml": twiml,
            "TimeLimit": str(cfg.CALL_TIME_LIMIT_S),   # Twilio hangs up after this many seconds, no matter what
        }).encode()
        req = urllib.request.Request(url, data=body, method="POST")
        auth = base64.b64encode(f"{s['sid']}:{s['token']}".encode()).decode()
        req.add_header("Authorization", f"Basic {auth}")
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                sid = json.load(resp).get("sid", "?")
            log.info("Called %s (Twilio call %s)", self._masked(), sid)
            self.status = time.strftime("called %H:%M:%S")
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="ignore")[:300]
            log.error("Twilio refused the call (%s): %s", e.code, detail)
            self.status = "call failed"
        except Exception as e:
            log.error("Couldn't reach Twilio (is the Pi online?): %s", e)
            self.status = "call failed"
