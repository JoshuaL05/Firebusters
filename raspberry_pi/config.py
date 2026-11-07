"""
System settings and calibration
"""
"""
config.py - every tunable number in one place.

Coordinate system (floor, in cm):
  x = along the rail (left -> right as seen from the rail)
  y = away from the rail (0 = edge of the demo area nearest the rail)
The nozzle sits on the rail carriage, behind the y=0 edge, at NOZZLE_HEIGHT_CM.

     y ^   (far edge, y = AREA_H_CM)
       |   +---------------------+
       |   |     demo area       |   <- thermal camera looks straight down at this
       |   +---------------------+
       |  ===[nozzle on carriage]===   <- rail, at y = NOZZLE_Y_CM (negative = behind the area)
       +----------------------------> x
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Serial link to the ESP32
# ---------------------------------------------------------------------------
SERIAL_PORT = None          # None = auto-detect (/dev/ttyUSB0, /dev/ttyACM0, ...)
SERIAL_BAUD = 115200
HEARTBEAT_S = 0.3           # Pi -> ESP32 "HB"; ESP32 turns spray off after 1 s of silence
ACK_TIMEOUT_S = 2.0
TELEMETRY_STALE_S = 2.0     # no telemetry for this long = treat ESP32 as disconnected

# ---------------------------------------------------------------------------
# Thermal camera (MLX90640) + detection  (used by scanner.py)
# ---------------------------------------------------------------------------
SENSOR_W, SENSOR_H = 32, 24
SCALE = 10                  # upscale -> 320x240 working image
REFRESH_HZ = 8              # 2, 4, 8 or 16 (8+ needs i2c_arm_baudrate=1000000)
FLIP_H = False              # mirror the thermal image if it comes out backwards
FLIP_V = False

FIRE_TEMP_C = 60.0          # pixels at/above this are "fire"
AMBIENT_DELTA_C = 15.0      # ...and must be this much hotter than the room
MIN_AREA = 30               # working px^2

MAX_MATCH_DIST = 50         # working px a fire can move between frames
CONFIRM_FRAMES = 3          # frames before a fire counts
MAX_MISSED_FRAMES = 3       # frames before a vanished fire is dropped
GROWTH_WINDOW_S = 2.0

DISPLAY_MIN_C, DISPLAY_MAX_C = 15.0, 100.0

# ---------------------------------------------------------------------------
# Geometry (cm)
# ---------------------------------------------------------------------------
AREA_W_CM = 60.0            # demo area the thermal camera covers (used until calibrated)
AREA_H_CM = 45.0
CALIBRATION_FILE = os.path.join(HERE, "calibration.json")
CALIBRATION_POINTS_CM = [(5, 5), (55, 5), (55, 40), (5, 40)]   # where calibrate.py asks for the hot object

RAIL_ENABLED = True         # False = nozzle stays at FIXED_NOZZLE_X_CM, pan-tilt only
RAIL_LENGTH_MM = 400.0
RAIL_ORIGIN_X_CM = 10.0     # floor x of the carriage when the rail is at 0 mm (home)
RAIL_TOLERANCE_MM = 2.0
FIXED_NOZZLE_X_CM = AREA_W_CM / 2

NOZZLE_Y_CM = -5.0          # nozzle position behind the near edge of the area
NOZZLE_HEIGHT_CM = 30.0     # nozzle height above the floor

# ---------------------------------------------------------------------------
# Servos (degrees, as sent to the ESP32)
# ---------------------------------------------------------------------------
PAN_CENTER_DEG = 90.0       # servo angle pointing straight out (+y)
PAN_DIR = 1                 # flip to -1 if the nozzle turns the wrong way
PAN_MIN_DEG, PAN_MAX_DEG = 0.0, 180.0
PAN_TRIM_DEG = 0.0          # small corrections after testing with calibrate.py --aim-test

TILT_HORIZONTAL_DEG = 90.0  # servo angle where the nozzle is level
TILT_DIR = 1                # +1 = bigger angle points further down
TILT_MIN_DEG, TILT_MAX_DEG = 60.0, 175.0
TILT_LOB_DEG = 0.0          # aim this much higher to compensate for spray drop
TILT_TRIM_DEG = 0.0

REST_PAN_DEG = PAN_CENTER_DEG
REST_TILT_DEG = 120.0

# ---------------------------------------------------------------------------
# Prioritization
# ---------------------------------------------------------------------------
DISTANCE_PENALTY = 0.01     # per cm of rail travel; stops it bouncing between similar fires
SWITCH_RATIO = 2.5          # abandon current fire if another scores this many times higher
MIN_COMMIT_S = 5.0          # ...but only after working on the current one at least this long

# ---------------------------------------------------------------------------
# Confirmation (flame sensor on the pan-tilt, 0-4095, higher = more flame)
# ---------------------------------------------------------------------------
REQUIRE_FLAME_CONFIRM = True   # set False (or --no-flame-confirm) when testing with hot mugs
FLAME_THRESHOLD = 1500
FLAME_THRESHOLD_ALERT = 1100   # used when the DHT11 says the room is heating up
FLAME_OUT_THRESHOLD = 800
CONFIRM_S = 0.6
FALSE_POSITIVE_COOLDOWN_S = 30.0
BLOCK_RADIUS_PX = 40

# ---------------------------------------------------------------------------
# Aiming
# ---------------------------------------------------------------------------
AIM_SETTLE_S = 0.4
AIM_TIMEOUT_S = 15.0
LOST_GRACE_S = 1.5
REACQUIRE_PX = 60

# ---------------------------------------------------------------------------
# Suppression
# ---------------------------------------------------------------------------
SWEEP_DEG = 4.0             # side-to-side sweep while spraying
SWEEP_PERIOD_S = 1.0
PROGRESS_MIN_DROP = 0.05    # 5% drop in heat or flame counts as progress
NO_PROGRESS_S = 3.0         # no progress for this long -> re-aim
MAX_REAIMS = 3
OUT_CONFIRM_S = 1.0         # fire must look out for this long before verifying
VERIFY_WAIT_S = 2.0
MAX_ATTEMPTS = 3

# ---------------------------------------------------------------------------
# Environment (DHT11)
# ---------------------------------------------------------------------------
ENV_BASELINE_S = 10.0
ENV_TEMP_RISE_C = 3.0
ENV_HUMIDITY_DROP = 10.0
