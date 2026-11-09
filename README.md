# FireBusters

An overhead thermal camera watches a floor area, finds fires, drives an x, y gantry over
the biggest (and fastest-growing) one, sprays it, double-checks it's out, then moves to the next.

```
 MLX90640 thermal cam ──I2C──► Raspberry Pi 4 ──USB serial──► ESP32 ──► X stepper (carriage on beam, belt)
 (fixed to the frame)           (decides)                      (does)  ──► Y stepper (beam, shaft + belts)
                                                                        ──► spray trigger servo
                                                               ◄── flame sensor (on the carriage, pointing down)
                                                               ◄── 2 limit switches, e-stop
```

## How it decides

`SCAN → MOVE → CONFIRM → SUPPRESS → VERIFY → SCAN`

1. **SCAN**: the thermal camera finds hot blobs (≥ 60 °C and well above room temp) and tracks them.
   A blob has to show up for 3 frames before it counts. Fires are ranked by `area × (1 + growth rate)`,
   with a small penalty for long trips.
2. **MOVE**: pixel → floor position (calibration) → gantry position. It drives the carriage over the fire.
3. **CONFIRM**: the flame sensor on the carriage now points down at the fire.
   No flame means a false positive (a hot mug, a heater). That spot is ignored for 30 s.
4. **SUPPRESS**: spray while circling slightly over the fire. It keeps going as long as the fire keeps shrinking.
   If it stops making progress for 3 s, it re-positions. It only switches fires if another one gets *much* bigger.
5. **VERIFY**: spray off, wait 2 s, and ask the flame sensor if any flame is left (the camera can't tell a
   relit candle from warm wax). If it's still burning, it retries (3 attempts), then logs a warning and
   moves on. If it's out, it parks and does a full rescan. Warm wax it sees later is recognized as leftover
   heat, not a false alarm.

Always on: ESP32 watchdog (spray off and gantry stops if the Pi goes quiet for 1 s) and an e-stop button.

## Files

| File | What it does |
|---|---|
| `raspberry_pi/main.py` | Starts everything. Run this. |
| `raspberry_pi/config.py` | **Every tunable number.** Thresholds, gantry travel, positions, timings. |
| `raspberry_pi/scanner.py` | MLX90640 reading, hot-blob detection, fire tracking. Also runs standalone. |
| `raspberry_pi/targeting.py` | Priority score, pixel → floor → gantry position. |
| `raspberry_pi/state_machine.py` | The decision loop above. |
| `raspberry_pi/motion.py` | Serial link to the ESP32. |
| `raspberry_pi/sensor.py` | Parses ESP32 telemetry, flame averaging. |
| `raspberry_pi/web.py` | Live view in a browser (`--web`): heat map, fire boxes, nozzle, state. |
| `raspberry_pi/sim.py` | Fake fires + fake gantry for testing without hardware. |
| `raspberry_pi/calibrate.py` | Camera-to-floor calibration and interactive gantry test. |
| `esp32/main.cpp` | ESP32 firmware: 2 steppers, homing, trigger servo, sensors, watchdog, e-stop. |

## Try it right now (no hardware)

```bash
cd raspberry_pi
pip install numpy opencv-python
python main.py --sim --show
```
You'll see the nozzle marker (✕) drive to each fire biggest-first and put out 3 of them. It also rejects
a hot mug as a false positive, ignores a person-temperature blob, and catches a fire that appears ~25 s in.

## Live view for the demo

```bash
python3 main.py --web          # on the Pi
```
Then open **http://heatseek.local:8080** on any laptop or phone on the same Wi-Fi. You'll see the live heat map
with fire boxes, priority labels (P1, P2...), the target ring, the nozzle (✕, orange while spraying), and the
current state. Works with `--sim` too. (`--show` opens a desktop window instead, but that needs a screen
plugged into the Pi, so it won't work over SSH.)

## Mounting

- **Nozzle + flame sensor:** on the carriage, both pointing straight down. Wrap the flame sensor in a short
  tube (a straw or black paper) so it only sees what's directly below.
- **Thermal camera:** fixed to the frame, seeing the whole floor. Best spot: a top corner, a little **below**
  the gantry, angled at the center. That way the moving carriage never blocks its view.
  Calibration corrects for the angle. The 110° wide-angle MLX90640 is easiest in a frame this size.
- **Limit switches:** one at the x = 0 end and one at the y = 0 end, so the carriage presses them when homing.

## Wiring (ESP32)

| Part | ESP32 pin | Notes |
|---|---|---|
| X A4988 STEP / DIR | GPIO 26 / 27 | Carriage along the beam (belt). |
| Y A4988 STEP / DIR | GPIO 18 / 19 | Beam front/back (motor + shaft). |
| Both A4988 EN | GPIO 25 | Or tie EN to GND. **You need 2 A4988s, one per motor.** |
| X limit switch | GPIO 32 → GND | |
| Y limit switch | GPIO 33 → GND | |
| E-stop button | GPIO 17 → GND | Optional; leave unconnected if you don't have one. |
| Trigger servo signal | GPIO 21 | Presses the spray bottle. Servo on its own 5 V; **share GND**. Tune `TRIGGER_ON/OFF_DEG`. |
| Flame sensor AO | GPIO 34 | Power the module from **3.3 V**. |

A4988s: VMOT from the 3S 18650 pack with a 100 µF cap across VMOT/GND, VDD 3.3 V, MS1–MS3 high (1/16 steps).
**Set each driver's current limit before connecting its motor.**

**Pi ↔ MLX90640:** VIN → pin 1 (3.3 V), GND → pin 6, SDA → pin 3, SCL → pin 5.
**Pi ↔ ESP32:** USB cable.

## Setup

### Raspberry Pi
```bash
sudo raspi-config            # Interface Options → I2C → Enable
echo "dtparam=i2c_arm_baudrate=1000000" | sudo tee -a /boot/firmware/config.txt
sudo apt install -y python3-opencv python3-numpy python3-serial i2c-tools git
sudo usermod -aG dialout $USER
sudo reboot
i2cdetect -y 1               # should show 33 (the MLX90640)
cd raspberry_pi
python3 -m venv --system-site-packages venv && source venv/bin/activate
pip install adafruit-circuitpython-mlx90640 adafruit-blinka
```

### ESP32
VS Code + PlatformIO: open the `esp32/` folder → Upload. (Arduino IDE: rename to `esp32.ino`
and install ESP32Servo and AccelStepper.) Open the serial monitor at 115200 baud
and you should see `LOG ready` followed by `T ...` lines.

## Bring-up order (test one thing at a time)

1. **ESP32 + gantry:** in the serial monitor, type `HOME`. Both axes should drive to their switches
   (`LOG X homed`, `LOG Y homed`). If an axis runs the wrong way, flip `X_INVERT_DIR` / `Y_INVERT_DIR`.
   Then try `MOVE 100 100`, `SPRAY 1`, `SPRAY 0`. Without a heartbeat, motion and spray stop after 1 s;
   that's the watchdog working.
2. **Check distances:** after `MOVE 100 0`, measure how far the carriage actually went. If it isn't 100 mm,
   fix `X_STEPS_PER_MM` (same for Y). Measure the full travel and set `X/Y_TRAVEL_MM` in both `main.cpp` and `config.py`.
3. **Camera:** `python3 scanner.py --headless`, holding something hot under it. Use `--flip-h` / `--flip-v` if it's mirrored.
4. **Calibrate:** `python3 calibrate.py`. Tape the 4 spots (`CALIBRATION_POINTS_CM`) and put a hot mug on each.
5. **Line up the gantry:** `python3 calibrate.py --aim-test`, then type `g 0 0`. Measure the floor position
   under the nozzle and put it in `HOME_FLOOR_X_CM` / `HOME_FLOOR_Y_CM`. Then type a taped spot like `15 15`
   and check the nozzle ends up right above it.
6. **Mug test:** `python3 main.py --no-flame-confirm --fire-temp 40`.
7. **The real thing:** `python3 main.py` (with flame confirmation).

Useful flags: `--port /dev/ttyUSB0`, `--log-file run.log`, `--show` (needs a screen).

## Tuning cheat sheet (`config.py`)

- Detecting non-fires: raise `FIRE_TEMP_C` or `AMBIENT_DELTA_C`.
- Missing small or far fires: lower `FIRE_TEMP_C` or `MIN_AREA`.
- Rejecting real fires: lower `FLAME_THRESHOLD`. Watch the `flame=` value in the status line while over a flame.
- Nozzle consistently off to one side: adjust `HOME_FLOOR_X_CM` / `HOME_FLOOR_Y_CM`.
- Carriage blocks the camera while scanning: move `PARK_X_MM` / `PARK_Y_MM` to a spot out of its view.