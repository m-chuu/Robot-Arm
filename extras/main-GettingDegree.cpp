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
struct JointLimits {
  int min;
  int max;
};

const JointLimits LIMIT_G  = {0, 180};
const JointLimits LIMIT_B  = {10, 260};
const JointLimits LIMIT_S  = {20, 160};
const JointLimits LIMIT_E  = {20, 160};
const JointLimits LIMIT_WP = {10, 170};
const JointLimits LIMIT_WR = {10, 170};

// ===================== POSE =====================
struct Pose {
  int g;
  int b;
  int s;
  int e;
  int wp;
  int wr;
};

const int MAX_SEQ = 20;

Pose sequence[MAX_SEQ];
int seqCount = 0;

// ===================== STATE =====================
int curG  = 30;
int curB  = 135;
int curS  = 90;
int curE  = 90;
int curWP = 90;
int curWR = 90;

const int STEP = 10;

bool showDashboard = true;

// ===================== PWM =====================
void setServoSafe(
  uint8_t ch,
  int angle,
  JointLimits limit,
  int range
) {
  int safeAngle = constrain(angle, limit.min, limit.max);

  pwm.writeMicroseconds(
    ch,
    map(safeAngle, 0, range, 500, 2500)
  );
}

void updateRobot() {

  setServoSafe(GRIPPER_CH,  curG,  LIMIT_G,  180);
  setServoSafe(BASE_CH,     curB,  LIMIT_B,  270);
  setServoSafe(SHOULDER_CH, curS,  LIMIT_S,  180);
  setServoSafe(ELBOW_CH,    curE,  LIMIT_E,  180);
  setServoSafe(WRIST_P_CH,  curWP, LIMIT_WP, 180);
  setServoSafe(WRIST_R_CH,  curWR, LIMIT_WR, 180);
}

// ===================== INTERPOLATION =====================
void moveToPose(
  Pose a,
  Pose b,
  int steps = 50,
  int delayMs = 30
) {

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

// ===================== REVIEW SEQUENCE =====================
void printSequence() {

  Serial.println();
  Serial.println("===== RECORDED SEQUENCE =====");

  if (seqCount == 0) {
    Serial.println("NO POSES RECORDED");
    return;
  }

  for (int i = 0; i < seqCount; i++) {

    Serial.printf(
      "{\"POSE_%d\", {%d, %d, %d, %d, %d, %d}},\n",
      i + 1,
      sequence[i].g,
      sequence[i].b,
      sequence[i].s,
      sequence[i].e,
      sequence[i].wp,
      sequence[i].wr
    );
  }

  Serial.println();
  Serial.println("Order:");
  Serial.println("{ GRIPPER, BASE, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL }");
  Serial.println("=============================");
}

// ===================== DASHBOARD =====================
void printDashboard() {

  Serial.println();
  Serial.println("==============================");
  Serial.println("      ROBOT TEACH SYSTEM      ");
  Serial.println("==============================");

  Serial.printf(
    "Pose: G:%d B:%d S:%d E:%d WP:%d WR:%d\n",
    curG,
    curB,
    curS,
    curE,
    curWP,
    curWR
  );

  Serial.printf(
    "Sequence: %d / %d\n",
    seqCount,
    MAX_SEQ
  );

  Serial.println("------------------------------");
  Serial.println("CONTROLS");
  Serial.println("------------------------------");

  Serial.println("[Q/E] Grip");
  Serial.println("[A/D] Base");
  Serial.println("[W/S] Shoulder");
  Serial.println("[Z/X] Elbow");
  Serial.println("[Y/F] Wrist Pitch");
  Serial.println("[T/G] Wrist Roll");

  Serial.println("------------------------------");

  Serial.println("[SPACE] Record Pose");
  Serial.println("[R] Review Sequence");
  Serial.println("[C] Clear Sequence");
  Serial.println("[P] Play Sequence");
  Serial.println("[V] Toggle Dashboard");
}

// ===================== PLAYBACK =====================
void playSequence() {

  Serial.println();
  Serial.println(">>> PLAYING SEQUENCE <<<");

  if (seqCount == 0) {
    Serial.println("EMPTY SEQUENCE");
    return;
  }

  for (int i = 0; i < seqCount - 1; i++) {
    moveToPose(sequence[i], sequence[i + 1]);
  }

  Serial.println(">>> DONE <<<");
}

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(50);

  updateRobot();

  Serial.println("SYSTEM READY");

  printDashboard();
}

// ===================== LOOP =====================
void loop() {

  if (!Serial.available()) {
    return;
  }

  char k = tolower(Serial.read());

  switch (k) {

    case 'q':
      curG += STEP;
      break;

    case 'e':
      curG -= STEP;
      break;

    case 'a':
      curB -= STEP;
      break;

    case 'd':
      curB += STEP;
      break;

    case 'w':
      curS += STEP;
      break;

    case 's':
      curS -= STEP;
      break;

    case 'z':
      curE += STEP;
      break;

    case 'x':
      curE -= STEP;
      break;

    case 'y':
      curWP += STEP;
      break;

    case 'f':
      curWP -= STEP;
      break;

    case 't':
      curWR += STEP;
      break;

    case 'g':
      curWR -= STEP;
      break;

    // ================= RECORD POSE =================
    case ' ':

      if (seqCount < MAX_SEQ) {

        sequence[seqCount++] = {
          curG,
          curB,
          curS,
          curE,
          curWP,
          curWR
        };

        Serial.printf(
          "{\"POSE_%d\", {%d, %d, %d, %d, %d, %d}},\n",
          seqCount,
          curG,
          curB,
          curS,
          curE,
          curWP,
          curWR
        );
      }
      else {

        Serial.println("MAX SEQUENCE REACHED");
      }

      break;

    // ================= REVIEW =================
    case 'r':

      printSequence();

      break;

    // ================= CLEAR =================
    case 'c':

      seqCount = 0;
      Serial.println("SEQUENCE CLEARED");

      break;

    // ================= PLAY =================
    case 'p':

      playSequence();

      break;

    // ================= DASHBOARD =================
    case 'v':

      showDashboard = !showDashboard;

      Serial.println(
        showDashboard
        ? "DASH ON"
        : "DASH OFF"
      );

      break;

    default:
      return;
  }

  updateRobot();

  if (showDashboard) {
    printDashboard();
  }
}