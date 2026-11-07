# FireBusters 🔥🧯

An overhead thermal camera watches a floor area, finds fires, puts out the biggest
(and fastest-growing) one first, double-checks it's out, then moves to the next.

```
 MLX90640 thermal cam ──I2C──► Raspberry Pi 4 ──USB serial──► ESP32 ──► pan/tilt servos
 (fixed, looking down)          (decides)                      (does)   ──► spray trigger servo
                                                                         ──► NEMA 17 rail (A4988)
                                                                ◄── flame sensor (on the pan-tilt)
                                                                ◄── DHT11, limit switch, e-stop
```

## How it decides

`SCAN → AIM → CONFIRM → SUPPRESS → VERIFY → SCAN`

1. **SCAN**: the thermal camera finds hot blobs (≥ 60 °C and well above room temp) and tracks them.
   A blob has to show up for 3 frames before it counts. Fires are ranked by `area × (1 + growth rate)`.
2. **AIM**: pixel → floor position (calibration) → rail position + pan/tilt angles.
3. **CONFIRM**: the flame sensor sits next to the nozzle, so it's now pointed at the fire.
   No flame means a false positive (a hot mug, a heater). That spot is ignored for 30 s.
4. **SUPPRESS**: spray while sweeping side to side. It keeps going as long as the fire keeps shrinking.
   If it stops making progress for 3 s, it re-aims. It only switches fires if another one gets *much* bigger.
5. **VERIFY**: spray off, wait 2 s, check again. If it's still burning, it retries (3 attempts),
   then logs a warning and moves on. If it's out, it does a full rescan, because other fires may have grown.

Always on: ESP32 watchdog (spray off if the Pi goes quiet for 1 s), e-stop button,
and a DHT11 room-heating alert that makes confirmation more sensitive.

## Files

| File | What it does |
|---|---|
| `raspberry pi/main.py` | Starts everything. Run this. |
| `raspberry pi/config.py` | **Every tunable number.** Thresholds, geometry, servo angles, timings. |
| `raspberry pi/scanner.py` | MLX90640 reading, hot-blob detection, fire tracking. Also runs standalone. |
| `raspberry pi/targeting.py` | Priority score, pixel → floor → rail/pan/tilt. |
| `raspberry pi/state_machine.py` | The decision loop above. |
| `raspberry pi/motion.py` | Serial link to the ESP32. |
| `raspberry pi/sensor.py` | Parses ESP32 telemetry, flame averaging, DHT11 alert. |
| `raspberry pi/sim.py` | Fake fires + fake ESP32 for testing without hardware. |
| `raspberry pi/calibrate.py` | Camera-to-floor calibration and interactive aim test. |
| `esp32/main.cpp` | ESP32 firmware: servos, stepper, sensors, watchdog, e-stop. |

## Try it right now (no hardware)

```bash
cd "raspberry pi"
pip install numpy opencv-python
python main.py --sim --show
```
You'll see it put out 3 fires biggest-first, reject a hot mug as a false positive, ignore a
person-temperature blob, and catch a fire that appears ~25 s in.

## Wiring (ESP32)

| Part | ESP32 pin | Notes |
|---|---|---|
| Pan servo signal | GPIO 18 | Servo power from a separate 5 V buck. **Share GND with the ESP32.** |
| Tilt servo signal | GPIO 19 | |
| Trigger servo signal | GPIO 21 | Presses the spray bottle trigger. Tune `TRIGGER_ON/OFF_DEG`. |
| A4988 STEP / DIR / EN | GPIO 26 / 27 / 25 | VMOT from the 3S 18650 pack + 100 µF cap. Set the current limit first! |
| Rail limit switch | GPIO 32 → GND | At the home (0 mm) end of the rail. |
| E-stop button | GPIO 33 → GND | Optional; leave unconnected if you don't have one. |
| Flame sensor AO | GPIO 34 | Power the module from **3.3 V**. Mount it next to the nozzle; a short tube narrows its view. |
| DHT11 data | GPIO 4 | 3.3 V |

**Pi ↔ MLX90640:** VIN → pin 1 (3.3 V), GND → pin 6, SDA → pin 3, SCL → pin 5.
**Pi ↔ ESP32:** USB cable.

**Power:** Pi from its USB-C supply (or a 5 V 5 A buck). Servos from their own 5 V buck. A4988 from the 3S pack.
Never power servos from the ESP32 or Pi pins.

## Setup

### Raspberry Pi
```bash
sudo raspi-config            # Interface Options → I2C → Enable
echo "dtparam=i2c_arm_baudrate=1000000" | sudo tee -a /boot/firmware/config.txt
sudo apt install -y python3-opencv python3-numpy python3-serial i2c-tools git
sudo usermod -aG dialout $USER
sudo reboot
i2cdetect -y 1               # should show 33 (the MLX90640)
cd "raspberry pi"
python3 -m venv --system-site-packages venv && source venv/bin/activate
pip install adafruit-circuitpython-mlx90640 adafruit-blinka
```

### ESP32
VS Code + PlatformIO: open the `esp32/` folder → Upload. (Arduino IDE: rename to `esp32.ino`
and install ESP32Servo, AccelStepper, and DHT sensor library.) Open the serial monitor at 115200 baud
and you should see `LOG ready` followed by `T ...` lines.

## Bring-up order (test one thing at a time)

1. `python3 scanner.py --headless`: hold something hot under the camera. Use `--flip-h` / `--flip-v` if it's mirrored.
2. ESP32 serial monitor: type `AIM 90 120`, `SPRAY 1`, `SPRAY 0`, `HOME`, `MOVE 100`.
   (No heartbeat from you means the spray turns itself off after 1 s. That's the watchdog working.)
3. `python3 calibrate.py`: tape 4 spots (see `CALIBRATION_POINTS_CM`), place a hot mug on each.
4. Measure `NOZZLE_HEIGHT_CM`, `NOZZLE_Y_CM`, `RAIL_ORIGIN_X_CM` and put them in `config.py`.
5. `python3 calibrate.py --aim-test`: type floor coords and check the nozzle points there. Adjust the trims.
6. `python3 main.py --no-flame-confirm --fire-temp 40`: full run with hot mugs.
7. `python3 main.py`: the real thing (with flame confirmation).

Useful flags: `--no-rail` (pan-tilt only), `--port /dev/ttyUSB0`, `--log-file run.log`, `--show` (needs a screen).

## Tuning cheat sheet (`config.py`)

- Detecting non-fires: raise `FIRE_TEMP_C` or `AMBIENT_DELTA_C`.
- Missing small or far fires: lower `FIRE_TEMP_C` or `MIN_AREA`.
- Rejecting real fires: lower `FLAME_THRESHOLD`. Watch the `flame=` value in the status line while pointing at a flame.
- Spray lands short: raise `TILT_LOB_DEG`.
- Turning the wrong way: flip `PAN_DIR` / `TILT_DIR`.

## Safety

Test with hot mugs and hand warmers. Use real flames only for the final demo, only if the venue allows it,
with a real extinguisher and a person standing by. Use a BMS and a fuse on the 18650 pack.