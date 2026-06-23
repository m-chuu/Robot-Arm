/*
 * 6-DOF Robotic Arm - Servo Calibration Tool
 * ------------------------------------------------------------------
 * Hardware : ESP32-S3 (DFRobot Romeo ESP32-S3)
 *            PCA9685 16-ch PWM driver (I2C)
 *            6x TD8125MG digital servos (~180 deg)
 *
 * Goal:
 *   1) MEASURE the safe min/max pulse each servo can travel to, by
 *      jogging it by hand (no auto-slamming into the mechanical stops).
 *   2) MEASURE the real physical angle with a drawn protractor/compass:
 *      tape a pointer to the servo horn, place it over a printed angle
 *      dial, move the joint to the marked min & max, read the degrees,
 *      and type them in. The tool fits a 2-point linear pulse<->angle
 *      mapping for every joint and prints a paste-ready table.
 *
 * Everything is driven over the USB serial monitor at 115200 baud using
 * SINGLE KEYPRESSES (use the PlatformIO monitor so keys send instantly).
 * Press 'h' for the key map.
 *
 * POWER NOTE: feed the PCA9685 V+ from a separate 5-6V supply rated for
 * the stall current of all 6 TD8125MG servos. Tie its GND to the ESP32
 * GND. Do NOT power the servos from the ESP32 5V pin.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== USER CONFIG =================================
// I2C address of the PCA9685 (default 0x40; set by the A0..A5 jumpers).
static const uint8_t PCA9685_ADDR = 0x40;

// I2C pins. Leave both as -1 to use the board's default Wire pins.
// Set to real GPIO numbers if your PCA9685 is wired elsewhere.
static const int I2C_SDA_PIN = -1;
static const int I2C_SCL_PIN = -1;

// Number of joints and which PCA9685 channel drives each one.
// Index 0 = joint 1 (base) ... index 5 = joint 6 (gripper), for example.
static const uint8_t NUM_SERVOS = 6;
static const uint8_t SERVO_CHANNEL[NUM_SERVOS] = {0, 1, 2, 3, 4, 5};

// Joint names, matching your calibration worksheet (Base..Gripper).
static const char* SERVO_NAME[NUM_SERVOS] = {
  "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"};

// How the "PWM" column is reported:
//   false -> pulse width in microseconds (us)  [used by writeMicroseconds()]
//   true  -> raw PCA9685 12-bit count 0..4095  [used by setPWM()]
static const bool SHOW_PWM_AS_COUNTS = false;

// PWM frequency for analog/digital RC servos. 50 Hz is standard.
static const float SERVO_FREQ_HZ = 50.0f;

// PCA9685 internal oscillator. Adafruit ships ~25 MHz but it varies a
// little per chip. If a scope/meter shows the output isn't 50 Hz, nudge
// this until it is, so that microsecond commands are accurate.
static const uint32_t PCA9685_OSC_HZ = 27000000;

// Absolute safety clamp (microseconds). A jog can NEVER command outside
// this window, so you can't drive a joint past the servo electronics.
// Widen only if you know the servo accepts it.
static const int PULSE_MIN_US = 400;
static const int PULSE_MAX_US = 2600;

// Where every joint parks on boot, and the default fine jog step.
static const int START_PULSE_US = 1500;  // mechanical center for most servos
static const int FINE_STEP_US   = 10;    // fine jog (Up/Down arrow, w/s)
static const int COARSE_STEP_US = 50;    // coarse jog (Right/Left arrow, d/a)

// Ramped-move smoothing for big jumps (go-to / center / gmin / gmax).
static const int RAMP_STEP_US  = 12;     // us per ramp step
static const int RAMP_DELAY_MS = 12;     // pause between ramp steps
// ==================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Per-joint calibration record.
struct ServoCal {
  uint8_t channel;
  int     currentUs;   // last commanded pulse

  int     minUs;       // marked lower endpoint
  int     maxUs;       // marked upper endpoint
  bool    minSet;
  bool    maxSet;

  // Two physical-angle reference points captured from the protractor.
  int     calPulseA;
  float   calAngleA;
  bool    calASet;
  int     calPulseB;
  float   calAngleB;
  bool    calBSet;
};

ServoCal servos[NUM_SERVOS];
uint8_t  activeIdx = 0;   // index into servos[] currently being driven

// ---- keypad input state ----
enum InputMode  { MODE_KEYS, MODE_NUM };               // typing keys vs a number
enum PendingNum { PEND_NONE, PEND_ANGLE_CAP,           // what the number is for
                  PEND_ANGLE_GO, PEND_PULSE_GO };
InputMode  inputMode  = MODE_KEYS;
PendingNum pendingNum = PEND_NONE;
String     numBuf;                                     // digits typed so far

// ---------- helpers ----------

int clampUs(int us) {
  if (us < PULSE_MIN_US) return PULSE_MIN_US;
  if (us > PULSE_MAX_US) return PULSE_MAX_US;
  return us;
}

// Approximate PCA9685 tick count (0..4095) for a pulse, for display only.
int usToTicks(int us) {
  return (int)lroundf((float)us * 4096.0f * SERVO_FREQ_HZ / 1000000.0f);
}

// The "PWM" value as it should appear in the worksheet, per SHOW_PWM_AS_COUNTS.
int         reportPwm(int us) { return SHOW_PWM_AS_COUNTS ? usToTicks(us) : us; }
const char* pwmUnit()         { return SHOW_PWM_AS_COUNTS ? "cnt" : "us"; }

bool calReady(const ServoCal &s) {
  return s.calASet && s.calBSet && (s.calPulseB != s.calPulseA);
}

// us -> physical degrees, using the 2-point linear fit (if available).
bool usToAngle(const ServoCal &s, int us, float &deg) {
  if (!calReady(s)) return false;
  float slope = (s.calAngleB - s.calAngleA) / (float)(s.calPulseB - s.calPulseA);
  deg = s.calAngleA + slope * (float)(us - s.calPulseA);
  return true;
}

// physical degrees -> us, inverse of the fit.
bool angleToUs(const ServoCal &s, float deg, int &us) {
  if (!calReady(s)) return false;
  float slope = (s.calAngleB - s.calAngleA) / (float)(s.calPulseB - s.calPulseA);
  if (slope == 0.0f) return false;
  us = s.calPulseA + (int)lroundf((deg - s.calAngleA) / slope);
  return true;
}

// Drive a channel to an exact pulse immediately (already clamped).
void writeUs(ServoCal &s, int us) {
  s.currentUs = us;
  pwm.writeMicroseconds(s.channel, (uint16_t)us);
}

// Smoothly ramp the active joint from its current pulse to a target.
void rampTo(ServoCal &s, int targetUs) {
  targetUs = clampUs(targetUs);
  int from = s.currentUs;
  int dir  = (targetUs >= from) ? RAMP_STEP_US : -RAMP_STEP_US;
  for (int us = from; (dir > 0) ? (us < targetUs) : (us > targetUs); us += dir) {
    writeUs(s, us);
    delay(RAMP_DELAY_MS);
  }
  writeUs(s, targetUs);
}

void printStatus() {
  ServoCal &s = servos[activeIdx];
  float deg;
  Serial.println();
  Serial.printf("[%s | joint %u | PCA ch %u]  PWM = %d %s   (= %d us / %d cnt)\n",
                SERVO_NAME[activeIdx], activeIdx + 1, s.channel,
                reportPwm(s.currentUs), pwmUnit(), s.currentUs, usToTicks(s.currentUs));
  if (usToAngle(s, s.currentUs, deg))
    Serial.printf("  estimated angle = %.1f deg\n", deg);
  Serial.printf("  min PWM %s%d %s   max PWM %s%d %s\n",
                s.minSet ? "= " : "(unset) ", reportPwm(s.minUs), pwmUnit(),
                s.maxSet ? "= " : "(unset) ", reportPwm(s.maxUs), pwmUnit());
}

void printHelp() {
  Serial.println(F("\n=============== 6-DOF Servo Keypad Controls ==============="));
  Serial.println(F("Just press a key (PlatformIO monitor sends it instantly)."));
  Serial.println(F("  1..6     select joint: 1=Base 2=Shoulder 3=Elbow"));
  Serial.println(F("                         4=Wrist Pitch 5=Wrist Roll 6=Gripper"));
  Serial.println(F("--- move (clamped to the safe pulse window) ---"));
  Serial.println(F("  Up/Down  (or w / s)  jog FINE   (+/- 10 us)"));
  Serial.println(F("  Rgt/Lft  (or d / a)  jog COARSE (+/- 50 us)"));
  Serial.println(F("  c        go to center (1500 us)"));
  Serial.println(F("  t        go to a typed pulse in us, then Enter"));
  Serial.println(F("  p        jump joint 5 (Wrist Roll) to 1050 us"));
  Serial.println(F("--- step 1: capture the safe travel limits (the PWM values) ---"));
  Serial.println(F("  [        mark current pulse as MIN PWM"));
  Serial.println(F("  ]        mark current pulse as MAX PWM"));
  Serial.println(F("  ,        go to MIN        .  go to MAX"));
  Serial.println(F("--- step 2: read degrees off your compass/protractor ---"));
  Serial.println(F("  g        capture the dial angle here (type deg + Enter)"));
  Serial.println(F("           (1st g = point A, 2nd g = point B -> fit done)"));
  Serial.println(F("  v        verify: drive to a typed angle (deg + Enter)"));
  Serial.println(F("--- misc / results ---"));
  Serial.println(F("  o release torque (limp)     r report this joint"));
  Serial.println(F("  P print full summary table  h or ? this help"));
  Serial.println(F("=========================================================="));
  Serial.println(F("Per joint:  press N -> jog to low end -> '['  ->  jog to high end -> ']'"));
  Serial.println(F("  -> ',' read dial press 'g' type deg -> '.' read dial 'g' deg -> 'r'."));
  Serial.println(F("  Do all six, then 'p' for the table."));
}

void cmdSelect(int joint1based) {
  if (joint1based < 1 || joint1based > NUM_SERVOS) {
    Serial.printf("! joint must be 1..%u\n", NUM_SERVOS);
    return;
  }
  activeIdx = (uint8_t)(joint1based - 1);
  Serial.printf("Selected %s (joint %d, PCA ch %u).\n",
                SERVO_NAME[activeIdx], joint1based, servos[activeIdx].channel);
  printStatus();
}

void cmdCapturAngle(float deg) {
  ServoCal &s = servos[activeIdx];
  if (!s.calASet || !s.calBSet) {
    if (!s.calASet) {
      s.calPulseA = s.currentUs; s.calAngleA = deg; s.calASet = true;
      Serial.printf("Point A captured: %d us = %.1f deg.\n", s.calPulseA, deg);
    } else {
      s.calPulseB = s.currentUs; s.calAngleB = deg; s.calBSet = true;
      Serial.printf("Point B captured: %d us = %.1f deg.\n", s.calPulseB, deg);
    }
  } else {
    // both already set -> start a fresh pair from this capture
    s.calPulseA = s.currentUs; s.calAngleA = deg; s.calASet = true;
    s.calBSet = false;
    Serial.printf("Recalibrating. Point A captured: %d us = %.1f deg.\n", s.calPulseA, deg);
  }
  if (calReady(s)) {
    float slope = (s.calAngleB - s.calAngleA) / (float)(s.calPulseB - s.calPulseA);
    Serial.printf("  -> linear fit: %.4f deg/us  (%.2f us/deg)\n",
                  slope, (slope != 0.0f) ? 1.0f / slope : 0.0f);
  }
}

void reportJoint(int idx) {
  ServoCal &s = servos[idx];
  Serial.printf("\n--- %s (joint %d, PCA ch %u) ---\n", SERVO_NAME[idx], idx + 1, s.channel);
  Serial.printf("  min PWM   : %s%d %s\n", s.minSet ? "" : "(unset) ", reportPwm(s.minUs), pwmUnit());
  Serial.printf("  max PWM   : %s%d %s\n", s.maxSet ? "" : "(unset) ", reportPwm(s.maxUs), pwmUnit());
  if (s.minSet && s.maxSet)
    Serial.printf("  pulse span: %d us\n", abs(s.maxUs - s.minUs));
  if (calReady(s)) {
    float slope = (s.calAngleB - s.calAngleA) / (float)(s.calPulseB - s.calPulseA);
    Serial.printf("  cal pt A  : %d us = %.1f deg\n", s.calPulseA, s.calAngleA);
    Serial.printf("  cal pt B  : %d us = %.1f deg\n", s.calPulseB, s.calAngleB);
    Serial.printf("  mapping   : %.4f deg/us  (%.2f us/deg)\n",
                  slope, (slope != 0.0f) ? 1.0f / slope : 0.0f);
    float aMin, aMax;
    if (s.minSet && usToAngle(s, s.minUs, aMin) &&
        s.maxSet && usToAngle(s, s.maxUs, aMax)) {
      Serial.printf("  travel    : %.1f deg .. %.1f deg  (%.1f deg sweep)\n",
                    aMin, aMax, fabsf(aMax - aMin));
    }
  } else {
    Serial.println(F("  angle cal : incomplete (need 2 'g' captures)"));
  }
}

void printSummary() {
  Serial.println(F("\n========================= CALIBRATION SUMMARY ========================="));
  Serial.printf ("Feature        min PWM(%s)   min degree   max PWM(%s)   max degree\n",
                 pwmUnit(), pwmUnit());
  for (int i = 0; i < NUM_SERVOS; i++) {
    ServoCal &s = servos[i];
    float aMin = NAN, aMax = NAN;
    if (calReady(s)) {
      usToAngle(s, s.minUs, aMin);
      usToAngle(s, s.maxUs, aMax);
    }
    Serial.printf("%-13s  %9d    %9.1f    %9d    %9.1f\n",
                  SERVO_NAME[i], reportPwm(s.minUs), aMin, reportPwm(s.maxUs), aMax);
  }
  // Paste-ready array for your main arm firmware.
  Serial.println(F("\n// ---- paste into your arm firmware ----"));
  Serial.println(F("// {channel, minUs, maxUs, minDeg, maxDeg}"));
  Serial.println(F("struct ServoLimit { uint8_t ch; int minUs; int maxUs; float minDeg; float maxDeg; };"));
  Serial.printf("const ServoLimit SERVO_LIMITS[%u] = {\n", NUM_SERVOS);
  for (int i = 0; i < NUM_SERVOS; i++) {
    ServoCal &s = servos[i];
    float aMin = 0, aMax = 0;
    usToAngle(s, s.minUs, aMin);
    usToAngle(s, s.maxUs, aMax);
    Serial.printf("  {%u, %d, %d, %.1f, %.1f},  // %s\n",
                  s.channel, s.minUs, s.maxUs, aMin, aMax, SERVO_NAME[i]);
  }
  Serial.println(F("};"));
  Serial.println(F("==========================================================="));
}

// ---------- keypad input ----------

// Jog the active joint by a signed microsecond delta (clamped) and show it.
void jog(int deltaUs) {
  ServoCal &s = servos[activeIdx];
  writeUs(s, clampUs(s.currentUs + deltaUs));
  printStatus();
}

// Begin typing a number; it is applied to `what` when Enter is pressed.
void startNumEntry(PendingNum what, const char *prompt) {
  pendingNum = what;
  numBuf = "";
  inputMode = MODE_NUM;
  Serial.print(prompt);
}

void applyNumEntry() {
  ServoCal &s = servos[activeIdx];
  float val = numBuf.toFloat();
  Serial.println();
  switch (pendingNum) {
    case PEND_ANGLE_CAP:
      cmdCapturAngle(val);
      break;
    case PEND_ANGLE_GO: {
      int us;
      if (angleToUs(s, val, us)) {
        rampTo(s, clampUs(us));
        Serial.printf("Commanded %.1f deg -> %d us. Check it on the dial.\n", val, s.currentUs);
        printStatus();
      } else {
        Serial.println(F("! angle cal incomplete (need 2 'g' captures)"));
      }
      break;
    }
    case PEND_PULSE_GO:
      rampTo(s, (int)lroundf(val));
      printStatus();
      break;
    default:
      break;
  }
  pendingNum = PEND_NONE;
}

// While in MODE_NUM: collect digits, Enter applies, Esc cancels.
void handleNumChar(char c) {
  if (c == '\n' || c == '\r') {
    // Apply only once digits have been typed. A stray newline (e.g. the line
    // ending a buffered monitor sends right after the 'g' key) is ignored, so
    // number entry survives in both raw and line-buffered monitors.
    if (numBuf.length()) { applyNumEntry(); inputMode = MODE_KEYS; }
    return;
  }
  if (c == 27) {  // Esc cancels
    Serial.println(F(" (cancelled)"));
    inputMode = MODE_KEYS;
    return;
  }
  if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
    numBuf += c;
    Serial.print(c);  // echo the number as you type it
  }
}

// While in MODE_KEYS: one keypress = one action.
void handleKey(int b) {
  // Arrow keys arrive as a 3-byte escape sequence: ESC '[' A/B/C/D
  static uint8_t esc = 0;
  if (esc == 1) { esc = (b == '[') ? 2 : 0; return; }
  if (esc == 2) {
    esc = 0;
    switch (b) {
      case 'A': jog(+FINE_STEP_US);   break;  // Up
      case 'B': jog(-FINE_STEP_US);   break;  // Down
      case 'C': jog(+COARSE_STEP_US); break;  // Right
      case 'D': jog(-COARSE_STEP_US); break;  // Left
    }
    return;
  }
  if (b == 27) { esc = 1; return; }

  ServoCal &s = servos[activeIdx];
  char c = (char)b;

  if (c >= '1' && c <= '6') { cmdSelect(c - '0'); return; }

  switch (c) {
    case 'w': case '+': case '=': jog(+FINE_STEP_US);   break;
    case 's': case '-':           jog(-FINE_STEP_US);   break;
    case 'd':                     jog(+COARSE_STEP_US); break;
    case 'a':                     jog(-COARSE_STEP_US); break;

    case '[': s.minUs = s.currentUs; s.minSet = true;
              Serial.printf("\n%s MIN PWM = %d %s.\n", SERVO_NAME[activeIdx], reportPwm(s.minUs), pwmUnit()); break;
    case ']': s.maxUs = s.currentUs; s.maxSet = true;
              Serial.printf("\n%s MAX PWM = %d %s.\n", SERVO_NAME[activeIdx], reportPwm(s.maxUs), pwmUnit()); break;

    case ',': if (s.minSet) { rampTo(s, s.minUs); printStatus(); }
              else Serial.println(F("\n! MIN not marked yet (press '[')")); break;
    case '.': if (s.maxSet) { rampTo(s, s.maxUs); printStatus(); }
              else Serial.println(F("\n! MAX not marked yet (press ']')")); break;
    case 'c': rampTo(s, START_PULSE_US); printStatus(); break;

    case 'g': startNumEntry(PEND_ANGLE_CAP, "\nType the angle from your dial (deg) + Enter: "); break;
    case 'v': startNumEntry(PEND_ANGLE_GO,  "\nGo to angle (deg) + Enter: "); break;
    case 't': startNumEntry(PEND_PULSE_GO,  "\nGo to pulse (us) + Enter: "); break;

    case 'o': pwm.setPWM(s.channel, 0, 0);
              Serial.printf("\n%s released (limp). Any move re-engages it.\n", SERVO_NAME[activeIdx]); break;

    case 'p': {  // jump joint 5 (Wrist Roll) to 1050 us
      activeIdx = 4;
      ServoCal &j5 = servos[activeIdx];
      rampTo(j5, 1050);
      Serial.printf("\nJoint 5 (%s) -> %d %s.\n",
                    SERVO_NAME[activeIdx], reportPwm(j5.currentUs), pwmUnit());
      printStatus();
      break;
    }

    case 'r': reportJoint(activeIdx); break;
    case 'P': printSummary(); break;
    case 'h': case '?': printHelp(); break;

    case '\n': case '\r': case ' ': break;  // ignore stray whitespace
    default: Serial.printf("\n? key '%c' - press h for the key map\n", c); break;
  }
}

// ---------- arduino entry points ----------

void setup() {
  Serial.begin(115200);
  delay(300);

  if (I2C_SDA_PIN >= 0 && I2C_SCL_PIN >= 0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else                                      Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(20);

  // Initialise records and park every joint at center.
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i] = ServoCal{};
    servos[i].channel = SERVO_CHANNEL[i];
    servos[i].minUs   = START_PULSE_US;
    servos[i].maxUs   = START_PULSE_US;
    writeUs(servos[i], START_PULSE_US);
    delay(60);
  }

  Serial.println(F("\n6-DOF Servo Calibration Tool ready."));
  Serial.printf("PCA9685 @ 0x%02X, %.0f Hz, %u joints parked at %d us.\n",
                PCA9685_ADDR, SERVO_FREQ_HZ, NUM_SERVOS, START_PULSE_US);
  printHelp();
  cmdSelect(1);
}

void loop() {
  while (Serial.available()) {
    int b = Serial.read();
    if (inputMode == MODE_NUM) handleNumChar((char)b);
    else                       handleKey(b);
  }
}


