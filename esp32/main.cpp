/*

    Fine Motor Control, after raspberry pi went initialize where to go for general location
*/
/*
 * FireBusters - ESP32 firmware
 *
 * The ESP32 is the "hands": it drives the pan-tilt, the spray trigger servo and
 * the rail stepper, reads the flame sensor + DHT11, and streams readings to the
 * Raspberry Pi over USB serial. The Pi makes every decision.
 *
 * Libraries (Arduino IDE: Library Manager / PlatformIO: see platformio.ini):
 *   - ESP32Servo              (Kevin Harrington)
 *   - AccelStepper            (Mike McCauley)
 *   - DHT sensor library      (Adafruit)  + Adafruit Unified Sensor
 * Arduino IDE users: rename this file to esp32.ino inside a folder called esp32/.
 *
 * Serial protocol, 115200 baud, one line per message:
 *   Pi -> ESP32:  PING | HOME | MOVE <mm> | AIM <pan> <tilt> | SPRAY <0|1> | STOP | HB
 *   ESP32 -> Pi:  PONG | OK <CMD> | ERR <CMD> <reason> | LOG <text>
 *                 T <flame> <tempC> <hum> <rail_mm> <moving> <spraying> <homed> <estop>   (every 50 ms)
 *
 * Safety:
 *   - Watchdog: no command (incl. HB heartbeat) for 1 s -> spray off, rail stops.
 *   - E-stop button (optional, to GND): spray off, rail stops, commands refused while held.
 *
 * Wiring (change the pin numbers below to match yours):
 *   Pan servo signal      GPIO 18     Servo power: separate 5 V supply, GND shared with ESP32!
 *   Tilt servo signal     GPIO 19
 *   Trigger servo signal  GPIO 21     (presses the spray bottle trigger)
 *   A4988 STEP            GPIO 26     A4988 VMOT: 8-35 V (3S 18650 pack), 100 uF cap across VMOT/GND
 *   A4988 DIR             GPIO 27     A4988 VDD: 3.3 V, set current limit BEFORE connecting motor
 *   A4988 EN              GPIO 25     (MS1-MS3 high = 1/16 microstepping, matches STEPS_PER_MM)
 *   Rail limit switch     GPIO 32     switch between pin and GND, at the HOME end of the rail
 *   E-stop button         GPIO 33     button between pin and GND (leave unconnected if none)
 *   Flame sensor AO       GPIO 34     power the module from 3.3 V so AO never exceeds 3.3 V
 *   DHT11 data            GPIO 4      power from 3.3 V (10k pull-up if your module lacks one)
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <DHT.h>

// ------------------------------------------------------------------ pins
const int PIN_PAN = 18;
const int PIN_TILT = 19;
const int PIN_TRIGGER = 21;
const int PIN_STEP = 26;
const int PIN_DIR = 27;
const int PIN_EN = 25;
const int PIN_LIMIT = 32;
const int PIN_ESTOP = 33;
const int PIN_FLAME_AO = 34;
const int PIN_DHT = 4;

// ------------------------------------------------------------------ settings
const bool  RAIL_ENABLED        = true;    // false if you have no rail / no limit switch yet
const float STEPS_PER_MM        = 80.0f;   // 200 steps x 16 microsteps / 40 mm (GT2 belt, 20T pulley)
                                           // lead screw T8 (8 mm/rev): 200*16/8 = 400
const float RAIL_LENGTH_MM      = 400.0f;
const float RAIL_MAX_SPEED_MM_S = 80.0f;
const float RAIL_ACCEL_MM_S2    = 200.0f;
const bool  RAIL_INVERT_DIR     = false;   // flip if the carriage moves away from the limit switch when homing
const float HOMING_SPEED_MM_S   = 20.0f;
const float HOME_BACKOFF_MM     = 3.0f;
const unsigned long HOMING_TIMEOUT_MS = 30000;

const float PAN_REST_DEG     = 90.0f;
const float TILT_REST_DEG    = 120.0f;
const float TRIGGER_OFF_DEG  = 20.0f;      // tune these two so the servo fully pulls / releases the trigger
const float TRIGGER_ON_DEG   = 80.0f;

const bool  FLAME_ACTIVE_LOW = true;       // most flame modules: AO voltage DROPS when they see flame
const unsigned long WATCHDOG_MS  = 1000;
const unsigned long TELEMETRY_MS = 50;
const unsigned long DHT_MS       = 2000;   // DHT11 can't be read faster than ~1 Hz
const unsigned long FLAME_MS     = 10;

// ------------------------------------------------------------------ state
Servo panServo, tiltServo, triggerServo;
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
DHT dht(PIN_DHT, DHT11);

bool spraying = false;
bool homed = !RAIL_ENABLED;
bool homing = false;
bool estop = false;
bool watchdogTripped = false;
unsigned long homingStart = 0, lastCmdMs = 0, lastTelemetryMs = 0, lastDhtMs = 0, lastFlameMs = 0;
float tempC = NAN, humidity = NAN, flameLevel = 0;
String lineBuf;

// ------------------------------------------------------------------ helpers
void writeAngle(Servo &s, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  s.writeMicroseconds(500 + (int)(deg / 180.0f * 1900.0f));   // finer than write(int)
}

void setSpray(bool on) {
  if (on && estop) return;
  spraying = on;
  writeAngle(triggerServo, on ? TRIGGER_ON_DEG : TRIGGER_OFF_DEG);
}

long mmToSteps(float mm) { return lround(mm * STEPS_PER_MM); }

void reply(const char *status, const String &cmd, const char *reason = nullptr) {
  Serial.print(status);
  Serial.print(' ');
  Serial.print(cmd);
  if (reason) { Serial.print(' '); Serial.print(reason); }
  Serial.println();
}

void readFlame() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_FLAME_AO);
  int raw = sum / 8;
  int v = FLAME_ACTIVE_LOW ? 4095 - raw : raw;    // higher always = more flame
  flameLevel = 0.7f * flameLevel + 0.3f * v;      // light smoothing
}

void stopRail() {
  homing = false;
  stepper.stop();
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

  if (cmd == "AIM") {
    float pan, tilt;
    if (sscanf(args.c_str(), "%f %f", &pan, &tilt) != 2) { reply("ERR", cmd, "bad_args"); return; }
    writeAngle(panServo, pan);
    writeAngle(tiltServo, tilt);
    reply("OK", cmd);
  } else if (cmd == "MOVE") {
    if (!RAIL_ENABLED) { reply("OK", cmd); return; }
    if (estop) { reply("ERR", cmd, "estop"); return; }
    if (!homed) { reply("ERR", cmd, "not_homed"); return; }
    float mm = constrain(args.toFloat(), 0.0f, RAIL_LENGTH_MM);
    stepper.setMaxSpeed(RAIL_MAX_SPEED_MM_S * STEPS_PER_MM);
    stepper.moveTo(mmToSteps(mm));
    reply("OK", cmd);
  } else if (cmd == "HOME") {
    if (!RAIL_ENABLED) { homed = true; reply("OK", cmd); return; }
    if (estop) { reply("ERR", cmd, "estop"); return; }
    homed = false;
    homing = true;
    homingStart = millis();
    stepper.setMaxSpeed(HOMING_SPEED_MM_S * STEPS_PER_MM);
    stepper.setSpeed(-HOMING_SPEED_MM_S * STEPS_PER_MM);   // toward the limit switch
    reply("OK", cmd);
  } else if (cmd == "SPRAY") {
    bool on = args.toInt() != 0;
    if (on && estop) { reply("ERR", cmd, "estop"); return; }
    setSpray(on);
    reply("OK", cmd);
  } else if (cmd == "STOP") {
    setSpray(false);
    stopRail();
    reply("OK", cmd);
  } else {
    reply("ERR", cmd, "unknown");
  }
}

// ------------------------------------------------------------------ homing
void updateHoming() {
  if (digitalRead(PIN_LIMIT) == LOW) {           // switch pressed: this is 0 mm
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(RAIL_MAX_SPEED_MM_S * STEPS_PER_MM);
    stepper.moveTo(mmToSteps(HOME_BACKOFF_MM));  // back off so the switch isn't held
    homing = false;
    homed = true;
    Serial.println("LOG homed");
    return;
  }
  if (millis() - homingStart > HOMING_TIMEOUT_MS) {
    homing = false;
    Serial.println("LOG homing timeout - check the limit switch wiring");
    return;
  }
  stepper.runSpeed();
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LIMIT, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);                      // A4988 enable is active-low

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_FLAME_AO, ADC_11db); // full 0-3.3 V range

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Servo *servos[] = {&panServo, &tiltServo, &triggerServo};
  int pins[] = {PIN_PAN, PIN_TILT, PIN_TRIGGER};
  for (int i = 0; i < 3; i++) {
    servos[i]->setPeriodHertz(50);
    servos[i]->attach(pins[i], 500, 2400);
  }
  writeAngle(panServo, PAN_REST_DEG);
  writeAngle(tiltServo, TILT_REST_DEG);
  setSpray(false);

  stepper.setPinsInverted(RAIL_INVERT_DIR, false, false);
  stepper.setMaxSpeed(RAIL_MAX_SPEED_MM_S * STEPS_PER_MM);
  stepper.setAcceleration(RAIL_ACCEL_MM_S2 * STEPS_PER_MM);

  dht.begin();
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
  bool pressed = digitalRead(PIN_ESTOP) == LOW;
  if (pressed && !estop) Serial.println("LOG E-STOP pressed");
  if (!pressed && estop) Serial.println("LOG E-STOP released");
  estop = pressed;
  if (estop) {
    if (spraying) setSpray(false);
    if (homing || stepper.distanceToGo() != 0) stopRail();
  }

  // 3. watchdog: Pi went quiet -> safe state (once per silence)
  if (now - lastCmdMs > WATCHDOG_MS) {
    if (!watchdogTripped && (spraying || stepper.distanceToGo() != 0 || homing)) {
      setSpray(false);
      stopRail();
      Serial.println("LOG watchdog: no heartbeat from Pi, spray off");
    }
    watchdogTripped = true;
  }

  // 4. rail
  if (homing) updateHoming();
  else stepper.run();

  // 5. sensors
  if (now - lastFlameMs >= FLAME_MS) { lastFlameMs = now; readFlame(); }
  if (!stepper.isRunning() && !homing && now - lastDhtMs >= DHT_MS) {   // DHT read blocks ~25 ms
    lastDhtMs = now;
    float t = dht.readTemperature(), h = dht.readHumidity();
    if (!isnan(t)) tempC = t;
    if (!isnan(h)) humidity = h;
  }

  // 6. telemetry
  if (now - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = now;
    Serial.printf("T %d %.1f %.1f %.1f %d %d %d %d\n",
                  (int)flameLevel, tempC, humidity,
                  stepper.currentPosition() / STEPS_PER_MM,
                  (stepper.isRunning() || homing) ? 1 : 0,
                  spraying ? 1 : 0, homed ? 1 : 0, estop ? 1 : 0);
  }
}
