/*
 * FireBusters - ESP32 firmware (x, y gantry)
 *
 * The ESP32 is the "hands": it drives the two gantry steppers and the spray
 * trigger servo, reads the flame sensor, and streams readings to the
 * Raspberry Pi over USB serial. The Pi makes every decision.
 *
 * Libraries (Arduino IDE: Library Manager / PlatformIO: see platformio.ini):
 *   - ESP32Servo              (Kevin Harrington)
 *   - AccelStepper            (Mike McCauley)
 * Arduino IDE users: rename this file to esp32.ino inside a folder called esp32/.
 *
 * Serial protocol, 115200 baud, one line per message:
 *   Pi -> ESP32:  PING | HOME | MOVE <x_mm> <y_mm> | SPRAY <0|1> | STOP | HB | SW (wiring check)
 *   ESP32 -> Pi:  PONG | OK <CMD> | ERR <CMD> <reason> | LOG <text>
 *                 T <flame> <x_mm> <y_mm> <moving> <spraying> <homed> <estop>  (every 50 ms)
 *
 * Safety:
 *   - Watchdog: no command (incl. HB heartbeat) for 1 s -> spray off, gantry stops.
 *   - E-stop button (optional, to GND): spray off, gantry stops, commands refused while held.
 *
 * Board: Adafruit ESP32 Feather V2. Pin labels printed on the Feather in [ ].
 * Wiring - matches the team's "Flame Gantry Schematic":
 *   X  A4988 STEP / DIR   GPIO 26 [A0] / 27 [27]   carriage along the beam (belt)
 *   Y  A4988 STEP / DIR   GPIO 20 [SCL] / 19 [MO]  beam front/back (motor + shaft)
 *   Both A4988 EN         GPIO 25 [A1]             10k pull-up R1 to 3V3F (drivers off until we enable)
 *        A4988 VMOT: +24 V + 100 uF cap; VDD + MS1-MS3: 3V3F (1/16 microstepping); RESET tied to SLEEP.
 *   X endstop module S    GPIO 32 [32]   lever module (S / - / +) on 3V3F, at the x = 0 end
 *   Y endstop module S    GPIO 33 [33]   lever module on 3V3F, at the y = 0 end
 *   E-stop (latching NC)  GPIO 7 [RX]    NC contact to GND: closed = run, open/broken wire = STOP
 *                         + the Feather's own button (GPIO 38) also works as an e-stop
 *   Trigger servo signal  GPIO 21 [MI]   SERVO1 on the servo block, powered from 5V_SRV
 *   Flame sensor AO       GPIO 34 [A2]   on the carriage pointing DOWN; VCC on 3V3F
 * Type SW in the serial monitor to see the raw endstop / e-stop readings when checking wiring.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// ------------------------------------------------------------------ pins
const int PIN_X_STEP = 26;
const int PIN_X_DIR = 27;
const int PIN_Y_STEP = 20;         // [SCL]
const int PIN_Y_DIR = 19;
const int PIN_EN = 25;
const int PIN_X_LIMIT = 32;
const int PIN_Y_LIMIT = 33;
const int PIN_ESTOP = 7;           // [RX] latching NC e-stop to GND
const int PIN_ESTOP_ONBOARD = 38;  // the Feather's built-in button (has its own pull-up)
const int PIN_TRIGGER = 21;
const int PIN_FLAME_AO = 34;

// ------------------------------------------------------------------ input logic
// Endstop modules (S / - / +): most output LOW normally and HIGH when the lever is pressed.
// If homing stops instantly without moving, or never stops at the switch, flip this.
// Check with the SW command: press a lever and see which way its reading changes.
const int  ENDSTOP_HIT_LEVEL = HIGH;
// E-stop wired as a normally-closed contact to GND: LOW = run, HIGH = stop (or broken wire).
// No e-stop fitted? Set ESTOP_FITTED to false (or put a jumper from RX to GND).
const bool ESTOP_FITTED = false;

// ------------------------------------------------------------------ settings
const float X_STEPS_PER_MM   = 80.0f;   // GT2 belt + 20T pulley, 200 steps x 1/16 microstepping
const float Y_STEPS_PER_MM   = 80.0f;   // same if the Y shaft drives GT2 belts with 20T pulleys
const float X_TRAVEL_MM      = 295.0f;  // match config.py
const float Y_TRAVEL_MM      = 230.0f;
const bool  X_INVERT_DIR     = false;   // flip if an axis homes AWAY from its limit switch
const bool  Y_INVERT_DIR     = false;
const float MAX_SPEED_MM_S   = 100.0f;
const float ACCEL_MM_S2      = 400.0f;
const float HOMING_SPEED_MM_S = 25.0f;
const float HOME_BACKOFF_MM  = 3.0f;
const unsigned long HOMING_TIMEOUT_MS = 40000;

const float TRIGGER_OFF_DEG  = 20.0f;   // tune these two so the servo fully pulls / releases the trigger
const float TRIGGER_ON_DEG   = 80.0f;

const bool  FLAME_ACTIVE_LOW = true;    // most flame modules: AO voltage DROPS when they see flame
const unsigned long WATCHDOG_MS  = 1000;
const unsigned long TELEMETRY_MS = 50;
const unsigned long FLAME_MS     = 10;

// ------------------------------------------------------------------ state
struct Axis {
  AccelStepper stepper;
  int limitPin;
  float stepsPerMm;
  float travelMm;
  bool homing;
  bool homed;
};

Axis axX = {AccelStepper(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR), PIN_X_LIMIT, X_STEPS_PER_MM, X_TRAVEL_MM, false, false};
Axis axY = {AccelStepper(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR), PIN_Y_LIMIT, Y_STEPS_PER_MM, Y_TRAVEL_MM, false, false};

Servo triggerServo;

bool spraying = false;
bool estop = false;
bool watchdogTripped = false;
unsigned long homingStart = 0, lastCmdMs = 0, lastTelemetryMs = 0, lastFlameMs = 0;
float flameLevel = 0;
String lineBuf;

// ------------------------------------------------------------------ helpers
void writeAngle(Servo &s, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  s.writeMicroseconds(500 + (int)(deg / 180.0f * 1900.0f));
}

void setSpray(bool on) {
  if (on && estop) return;
  spraying = on;
  writeAngle(triggerServo, on ? TRIGGER_ON_DEG : TRIGGER_OFF_DEG);
}

void reply(const char *status, const String &cmd, const char *reason = nullptr) {
  Serial.print(status);
  Serial.print(' ');
  Serial.print(cmd);
  if (reason) { Serial.print(' '); Serial.print(reason); }
  Serial.println();
}

bool homed() { return axX.homed && axY.homed; }
bool homing() { return axX.homing || axY.homing; }
bool moving() { return homing() || axX.stepper.isRunning() || axY.stepper.isRunning(); }
float posMm(Axis &a) { return a.stepper.currentPosition() / a.stepsPerMm; }

void moveAxis(Axis &a, float mm) {
  mm = constrain(mm, 0.0f, a.travelMm);
  a.stepper.setMaxSpeed(MAX_SPEED_MM_S * a.stepsPerMm);
  a.stepper.moveTo(lround(mm * a.stepsPerMm));
}

void startHoming(Axis &a) {
  a.homed = false;
  a.homing = true;
  a.stepper.setMaxSpeed(HOMING_SPEED_MM_S * a.stepsPerMm);
  a.stepper.setSpeed(-HOMING_SPEED_MM_S * a.stepsPerMm);   // toward the limit switch
}

void stopAll() {
  axX.homing = axY.homing = false;
  axX.stepper.stop();
  axY.stepper.stop();
}

void readFlame() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_FLAME_AO);
  int raw = sum / 8;
  int v = FLAME_ACTIVE_LOW ? 4095 - raw : raw;    // higher always = more flame
  flameLevel = 0.7f * flameLevel + 0.3f * v;
}

// ------------------------------------------------------------------ commands
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  lastCmdMs = millis();
  watchdogTripped = false;

  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String args = sp < 0 ? "" : line.substring(sp + 1);
  cmd.toUpperCase();

  if (cmd == "HB") return;
  if (cmd == "PING") { Serial.println("PONG"); return; }

  if (cmd == "MOVE") {
    float x, y;
    if (sscanf(args.c_str(), "%f %f", &x, &y) != 2) { reply("ERR", cmd, "bad_args"); return; }
    if (estop) { reply("ERR", cmd, "estop"); return; }
    if (!homed()) { reply("ERR", cmd, "not_homed"); return; }
    moveAxis(axX, x);
    moveAxis(axY, y);
    reply("OK", cmd);
  } else if (cmd == "HOME") {
    if (estop) { reply("ERR", cmd, "estop"); return; }
    homingStart = millis();
    startHoming(axX);
    startHoming(axY);
    reply("OK", cmd);
  } else if (cmd == "SPRAY") {
    bool on = args.toInt() != 0;
    if (on && estop) { reply("ERR", cmd, "estop"); return; }
    setSpray(on);
    reply("OK", cmd);
  } else if (cmd == "SW") {
    // raw input readings, for checking endstop / e-stop wiring
    Serial.printf("LOG SW xend=%d yend=%d (hit=%d)  estop_pin=%d (stop=1)  onboard_btn=%d (pressed=0)\n",
                  digitalRead(PIN_X_LIMIT), digitalRead(PIN_Y_LIMIT), ENDSTOP_HIT_LEVEL,
                  digitalRead(PIN_ESTOP), digitalRead(PIN_ESTOP_ONBOARD));
    reply("OK", cmd);
  } else if (cmd == "STOP") {
    setSpray(false);
    stopAll();
    reply("OK", cmd);
  } else {
    reply("ERR", cmd, "unknown");
  }
}

// ------------------------------------------------------------------ homing (both axes at once)
void updateHoming(Axis &a, const char *name) {
  if (!a.homing) return;
  if (digitalRead(a.limitPin) == ENDSTOP_HIT_LEVEL) {   // lever pressed: this is 0 mm
    a.stepper.setCurrentPosition(0);
    a.stepper.setMaxSpeed(MAX_SPEED_MM_S * a.stepsPerMm);
    a.stepper.moveTo(lround(HOME_BACKOFF_MM * a.stepsPerMm));  // back off the switch
    a.homing = false;
    a.homed = true;
    Serial.printf("LOG %s homed\n", name);
    return;
  }
  if (millis() - homingStart > HOMING_TIMEOUT_MS) {
    a.homing = false;
    Serial.printf("LOG %s homing timeout - check its limit switch\n", name);
    return;
  }
  a.stepper.runSpeed();
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);

  pinMode(PIN_X_LIMIT, INPUT_PULLUP);
  pinMode(PIN_Y_LIMIT, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_ESTOP_ONBOARD, INPUT);              // input-only pin, pull-up is on the board
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);                      // A4988 enable is active-low

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_FLAME_AO, ADC_11db); // full 0-3.3 V range

  ESP32PWM::allocateTimer(0);
  triggerServo.setPeriodHertz(50);
  triggerServo.attach(PIN_TRIGGER, 500, 2400);
  setSpray(false);

  axX.stepper.setPinsInverted(X_INVERT_DIR, false, false);
  axY.stepper.setPinsInverted(Y_INVERT_DIR, false, false);
  Axis *axes[] = {&axX, &axY};
  for (Axis *a : axes) {
    a->stepper.setMaxSpeed(MAX_SPEED_MM_S * a->stepsPerMm);
    a->stepper.setAcceleration(ACCEL_MM_S2 * a->stepsPerMm);
  }

  lastCmdMs = millis();
  Serial.println("LOG ready");
}

void loop() {
  // 1. commands from the Pi
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleCommand(lineBuf); lineBuf = ""; }
    else if (c != '\r' && lineBuf.length() < 64) lineBuf += c;
  }

  unsigned long now = millis();

  // 2. emergency stop
  bool pressed = (ESTOP_FITTED && digitalRead(PIN_ESTOP) == HIGH)   // NC contact opened
                 || digitalRead(PIN_ESTOP_ONBOARD) == LOW;          // Feather's own button
  if (pressed && !estop) Serial.println("LOG E-STOP pressed");
  if (!pressed && estop) Serial.println("LOG E-STOP released");
  estop = pressed;
  if (estop) {
    if (spraying) setSpray(false);
    if (moving()) stopAll();
  }

  // 3. watchdog: Pi went quiet -> safe state (once per silence)
  if (now - lastCmdMs > WATCHDOG_MS) {
    if (!watchdogTripped && (spraying || moving())) {
      setSpray(false);
      stopAll();
      Serial.println("LOG watchdog: no heartbeat from Pi, spray off, gantry stopped");
    }
    watchdogTripped = true;
  }

  // 4. motion
  if (axX.homing) updateHoming(axX, "X"); else axX.stepper.run();
  if (axY.homing) updateHoming(axY, "Y"); else axY.stepper.run();

  // 5. sensors
  if (now - lastFlameMs >= FLAME_MS) { lastFlameMs = now; readFlame(); }

  // 6. telemetry
  if (now - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = now;
    Serial.printf("T %d %.1f %.1f %d %d %d %d\n",
                  (int)flameLevel, posMm(axX), posMm(axY),
                  moving() ? 1 : 0, spraying ? 1 : 0, homed() ? 1 : 0, estop ? 1 : 0);
  }
}
