"""
config.py - every tunable number in one place.

Hardware: an overhead x, y gantry (like a 3D printer). The nozzle and the flame
sensor ride on the carriage and point straight DOWN. The thermal camera is
fixed to the frame and watches the whole floor.

Coordinates:
  floor (x, y) in cm  - where things are on the floor, from calibration
  gantry (x, y) in mm - carriage position; (0, 0) = home corner (limit switches)

  Looking down from above:
      +------------------------+
      |  y ^                   |
      |    |   floor area      |
      |    |                   |
      | (0,0) home ---> x      |
      +------------------------+
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# Serial link to the ESP32
# ---------------------------------------------------------------------------
SERIAL_PORT = None          # None = auto-detect (/dev/ttyUSB0, /dev/ttyACM0, ...)
SERIAL_BAUD = 115200
HEARTBEAT_S = 0.3           # Pi -> ESP32 "HB"; ESP32 stops spraying/moving after 1 s of silence
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
# Geometry
# ---------------------------------------------------------------------------
AREA_W_CM = 30.0            # floor area the camera covers (only used until calibrated)
AREA_H_CM = 30.0
CALIBRATION_FILE = os.path.join(HERE, "calibration.json")
CALIBRATION_POINTS_CM = [(3, 3), (27, 3), (27, 27), (3, 27)]   # tape these spots on the floor

X_TRAVEL_MM = 295.0         # how far the carriage can go from home on each axis (measure!)
Y_TRAVEL_MM = 230.0
HOME_FLOOR_X_CM = 0.0       # floor position directly under the NOZZLE when the gantry is at (0, 0)
HOME_FLOOR_Y_CM = 0.0       # (measure once after homing; calibrate.py --aim-test helps)
POS_TOLERANCE_MM = 2.0

PARK_X_MM = 0.0             # where the carriage waits while scanning/verifying
PARK_Y_MM = 0.0             # (keep it out of the camera's view of the fires)

# ---------------------------------------------------------------------------
# Prioritization
# ---------------------------------------------------------------------------
DISTANCE_PENALTY = 0.01     # per cm of travel; stops it bouncing between similar fires
SWITCH_RATIO = 2.5          # abandon current fire if another scores this many times higher
MIN_COMMIT_S = 5.0          # ...but only after working on the current one at least this long

# ---------------------------------------------------------------------------
# Confirmation (flame sensor on the carriage pointing down, 0-4095, higher = more flame)
# ---------------------------------------------------------------------------
REQUIRE_FLAME_CONFIRM = True   # set False (or --no-flame-confirm) when testing with hot mugs
FLAME_THRESHOLD = 1500
FLAME_THRESHOLD_ALERT = 1100   # used when the DHT11 says the room is heating up
FLAME_OUT_THRESHOLD = 800
CONFIRM_S = 0.6
FALSE_POSITIVE_COOLDOWN_S = 30.0
BLOCK_RADIUS_PX = 40

# ---------------------------------------------------------------------------
# Moving to a fire
# ---------------------------------------------------------------------------
MOVE_SETTLE_S = 0.3
MOVE_TIMEOUT_S = 15.0
LOST_GRACE_S = 1.5
REACQUIRE_PX = 60

# ---------------------------------------------------------------------------
# Suppression
# ---------------------------------------------------------------------------
SWEEP_MM = 8.0              # small circles over the fire while spraying (covers aim error)
SWEEP_PERIOD_S = 1.5
PROGRESS_MIN_DROP = 0.05    # 5% drop in heat or flame counts as progress
NO_PROGRESS_S = 3.0         # no progress for this long -> re-position over the fire
MAX_REAIMS = 3
OUT_CONFIRM_S = 1.0         # fire must look out for this long before verifying
VERIFY_PARK = True          # thermal-only mode: move out of the camera's way before double-checking
                            # (with the flame sensor it stays put and asks the flame sensor instead)
VERIFY_WAIT_S = 2.0
MAX_ATTEMPTS = 3
WARM_SPOT_S = 60.0          # a spot we put out may stay hot (melted wax) for this long;
                            # if it gets checked again with no flame, it's not a "false alarm"

# ---------------------------------------------------------------------------
# Environment (DHT11)
# ---------------------------------------------------------------------------
ENV_BASELINE_S = 10.0
ENV_TEMP_RISE_C = 3.0
ENV_HUMIDITY_DROP = 10.0
