#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== CHANNELS =====================
const int GRIPPER_CH  = 0;
const int WRIST_R_CH  = 1;
const int SHOULDER_CH = 2;
const int ELBOW_CH    = 3;
const int WRIST_P_CH  = 4;
const int BASE_CH     = 5;

// ===================== LIMITS =====================
struct JointLimits { int min; int max; };

const JointLimits LIMIT_G  = {0, 180};
const JointLimits LIMIT_B  = {10, 260};
const JointLimits LIMIT_S  = {20, 160};
const JointLimits LIMIT_E  = {20, 160};
const JointLimits LIMIT_WP = {10, 170};
const JointLimits LIMIT_WR = {10, 170};

// ===================== POSE =====================
struct Pose {
  int g, b, s, e, wp, wr;
};

struct NamedPose {
  const char* name;
  Pose pose;
};

// ===================== HARD-CODED SEQUENCE =====================
NamedPose sequence[] = {

  {"POSE_1", {30, 145, 100, 80, 140, 100}},
  {"POSE_2", {30, 285, 100, 80, 30, 100}},
  {"POSE_3", {160, 285, 100, 80, 30, 100}},
  {"POSE_4", {160, 285, 100, 80, 130, 100}},
  {"POSE_5", {160, 75, 100, 80, 130, 100}},
  {"POSE_6", {160, 75, 100, 80, 30, 100}},
  {"POSE_7", {-120, 75, 100, 80, 30, 100}},
  {"POSE_8", {-120, 75, 100, 80, 90, 100}},
};

const int seqCount = sizeof(sequence) / sizeof(sequence[0]);

// ===================== STATE =====================
int curG=30, curB=135, curS=90, curE=90, curWP=90, curWR=90;
bool isRunning = false;

// ===================== HOME POSITION =====================
const Pose HOME_POSE = {30, 145, 100, 80, 140, 100};

// ===================== PWM =====================
void setServoSafe(uint8_t ch, int angle, JointLimits limit, int range) {
  int safeAngle = constrain(angle, limit.min, limit.max);
  pwm.writeMicroseconds(ch, map(safeAngle, 0, range, 500, 2500));
}

void updateRobot() {
  setServoSafe(GRIPPER_CH,  curG,  LIMIT_G,  180);
  setServoSafe(BASE_CH,     curB,  LIMIT_B,  270);
  setServoSafe(SHOULDER_CH, curS,  LIMIT_S,  180);
  setServoSafe(ELBOW_CH,    curE,  LIMIT_E,  180);
  setServoSafe(WRIST_P_CH,  curWP, LIMIT_WP, 180);
  setServoSafe(WRIST_R_CH,  curWR, LIMIT_WR, 180);
}

// ===================== MOTION ENGINE =====================
void moveToPose(Pose a, Pose b, int steps = 60, int delayMs = 25) {

  for (int i = 0; i <= steps; i++) {

    float t = (float)i / steps;

    curG  = a.g  + (b.g  - a.g)  * t;
    curB  = a.b  + (b.b  - a.b)  * t;
    curS  = a.s  + (b.s  - a.s)  * t;
    curE  = a.e  + (b.e  - a.e)  * t;
    curWP = a.wp + (b.wp - a.wp) * t;
    curWR = a.wr + (b.wr - a.wr) * t;

    updateRobot();
    delay(delayMs);
  }
}

// ===================== SAFE HOMING =====================
void goHome() {

  Serial.println("\n>>> SAFE HOMING <<<");

  Pose start = {curG, curB, curS, curE, curWP, curWR};

  // STEP 1: RETRACT WRIST
  Pose retract = {
    HOME_POSE.g,
    curB,
    curS,
    curE,
    90,
    90
  };

  moveToPose(start, retract);

  // STEP 2: ELEVATE ARM
  Pose elevate = {
    HOME_POSE.g,
    curB,
    90,
    90,
    90,
    90
  };

  moveToPose(retract, elevate);

  // STEP 3: ALIGN BASE
  Pose align = {
    HOME_POSE.g,
    HOME_POSE.b,
    90,
    90,
    90,
    90
  };

  moveToPose(elevate, align);

  // STEP 4: FINAL HOME POSITION
  moveToPose(align, HOME_POSE);

  curG  = HOME_POSE.g;
  curB  = HOME_POSE.b;
  curS  = HOME_POSE.s;
  curE  = HOME_POSE.e;
  curWP = HOME_POSE.wp;
  curWR = HOME_POSE.wr;

  updateRobot();

  Serial.println(">>> HOME REACHED <<<");
}

// ===================== RUN SEQUENCE =====================
void runScript() {

  if (isRunning) return;
  isRunning = true;

  Serial.println("\n>>> RUNNING SEQUENCE <<<");

  for (int i = 0; i < seqCount - 1; i++) {

    Serial.print("Step: ");
    Serial.println(sequence[i].name);

    moveToPose(sequence[i].pose, sequence[i + 1].pose);
  }

  Serial.println(">>> SEQUENCE COMPLETE <<<");

  // ===================== AUTO RETURN HOME =====================
  Serial.println("Returning to HOME position...");
  delay(300);
  goHome();

  isRunning = false;
}

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(50);

  updateRobot();

  delay(1000);

  // AUTO HOME ON STARTUP
  goHome();

  Serial.println("\nSYSTEM READY");
  Serial.println("ENTER = Run Sequence | H = Home");
}

// ===================== LOOP =====================
void loop() {

  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == '\n') {
    runScript();
  }

  if (tolower(c) == 'h') {
    goHome();
  }
}