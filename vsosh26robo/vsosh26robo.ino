#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>

// =================== PINS ===================
// ULN2003 IN1..IN4
const uint8_t PIN_M1 = 2;
const uint8_t PIN_M2 = 3;
const uint8_t PIN_M3 = 4;
const uint8_t PIN_M4 = 5;

// UI
const uint8_t PIN_START_BTN = 7;
const uint8_t PIN_LED_RUN   = 13;

// Датчик черной линии
const uint8_t PIN_BLACK_SENSOR = A0;
int  BLACK_THRESHOLD = 500;
bool BLACK_IS_LOW    = true;

// Сервы
const uint8_t PIN_SERVO_SHOULDER = 9;
const uint8_t PIN_SERVO_ELBOW    = 10;
const uint8_t PIN_SERVO_PEN      = 11;

// =================== SERVO SETTINGS ===================
// Use microseconds mapping for stable motion
int SERVO_US_MIN = 900;
int SERVO_US_MAX = 2100;

// Neutral positions (tune by calibration)
int SHOULDER_0 = 90;
int ELBOW_0    = 90;

// Pen up/down (tune so lift >= 5mm, and draws reliably)
int PEN_UP_DEG   = 120;
int PEN_DOWN_DEG = 80;

// настройка плавности серво
const int SERVO_STEP_DEG       = 1;   // step in degrees
const int SERVO_STEP_DELAY_MS  = 10;  // delay per step (bigger -> smoother but slower)

// =================== X AXIS (28BYJ-48 + rack M2 + gear 32T) ===================
const float X_MAX_MM = 297.0f;

// Calculated from rack module m=2 and gear z=32, 28BYJ-48 ~4096 half-steps/rev:
// steps/mm = 4096 / (z*pi*m) = 20.36
const float STEPS_PER_MM = 20.36f;

// Realistic speed/accel for 28BYJ-48 (avoid missed steps)
float X_SPEED_MM_S   = 40.0f;
float X_ACCEL_MM_S2  = 150.0f;
float HOME_SPEED_MM_S = 20.0f;

// How far to drive into the "field" after button press BEFORE homing/drawing.
// Tune 40..120mm depending on where robot starts relative to the sheet.
const float FIELD_ENTRY_MM = 80.0f;

// Stepper test travel
const float STEPPER_TEST_MM = 30.0f;

// =================== ENGINE ===================
AccelStepper xStepper(AccelStepper::HALF4WIRE, PIN_M1, PIN_M3, PIN_M2, PIN_M4);
Servo sShoulder, sElbow, sPen;

// Current servo positions (for smooth moves)
int curShoulder = 90;
int curElbow    = 90;
int curPen      = 120;

// =================== COMMAND ARRAY ===================
enum CmdType : uint8_t { CMD_POINT, CMD_SEGMENT, CMD_WAIT_BTN };

struct Command {
  CmdType type;
  float a; // x or x1
  float b; // unused or x2
};

const int MAX_CMDS = 80;
Command cmds[MAX_CMDS];
int cmdCount = 0;

// =================== HELPERS ===================
static inline long mmToSteps(float mm) {
  return (long)lround(mm * STEPS_PER_MM);
}
static inline float stepsToMm(long st) {
  return (float)st / STEPS_PER_MM;
}

bool startPressed() {
  return digitalRead(PIN_START_BTN) == LOW;
}

int readBlack() {
  return analogRead(PIN_BLACK_SENSOR);
}

bool isBlack(int v) {
  return BLACK_IS_LOW ? (v < BLACK_THRESHOLD) : (v > BLACK_THRESHOLD);
}

int degToUs(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
}

void servoGotoSmooth(Servo &s, int &curDeg, int targetDeg) {
  targetDeg = constrain(targetDeg, 0, 180);
  if (curDeg == targetDeg) return;

  int step = (targetDeg > curDeg) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;

  while (curDeg != targetDeg) {
    curDeg += step;
    if ((step > 0 && curDeg > targetDeg) || (step < 0 && curDeg < targetDeg)) {
      curDeg = targetDeg;
    }
    s.writeMicroseconds(degToUs(curDeg));
    delay(SERVO_STEP_DELAY_MS);
  }
}

// =================== PEN ===================
void penUpSmooth() {
  servoGotoSmooth(sPen, curPen, PEN_UP_DEG);
  delay(80);
}
void penDownSmooth() {
  servoGotoSmooth(sPen, curPen, PEN_DOWN_DEG);
  delay(100);
}

// =================== X MOVEMENT ===================
void xMoveToMm(float x_mm) {
  x_mm = constrain(x_mm, 0.0f, X_MAX_MM);
  xStepper.moveTo(mmToSteps(x_mm));
  while (xStepper.distanceToGo() != 0) xStepper.run();
}

void homeByBlackLine() {
  xStepper.setMaxSpeed(mmToSteps(HOME_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // small initial move
  xStepper.move(mmToSteps(10));
  while (xStepper.distanceToGo() != 0) xStepper.run();

  // move right until black stripe found
  xStepper.moveTo(mmToSteps(X_MAX_MM));
  while (xStepper.distanceToGo() != 0) {
    xStepper.run();
    if (isBlack(readBlack())) {
      xStepper.stop();
      while (xStepper.isRunning()) xStepper.run();
      break;
    }
  }

  // black stripe edge = X0
  xStepper.setCurrentPosition(0);

  // move into white zone
  xMoveToMm(5);
}

// =================== DRAW PRIMITIVES ===================
void drawPoint(float x_mm) {
  xMoveToMm(x_mm);
  penDownSmooth();
  delay(120);
  penUpSmooth();
}

void drawSegment(float x1_mm, float x2_mm) {
  if (x2_mm < x1_mm) { float t = x1_mm; x1_mm = x2_mm; x2_mm = t; }
  xMoveToMm(x1_mm);
  penDownSmooth();
  xMoveToMm(x2_mm);
  penUpSmooth();
}

// =================== COMMANDS: add/list/clear ===================
bool addCmd(CmdType t, float a = 0, float b = 0) {
  if (cmdCount >= MAX_CMDS) return false;
  cmds[cmdCount++] = { t, a, b };
  return true;
}

void clearCmds() {
  cmdCount = 0;
  Serial.println("CMDs cleared.");
}

void listCmds() {
  Serial.print("CMD count = "); Serial.println(cmdCount);
  for (int i = 0; i < cmdCount; i++) {
    Serial.print(i); Serial.print(": ");
    if (cmds[i].type == CMD_POINT) {
      Serial.print("P "); Serial.println(cmds[i].a, 2);
    } else if (cmds[i].type == CMD_SEGMENT) {
      Serial.print("S ");
      Serial.print(cmds[i].a, 2);
      Serial.print(" ");
      Serial.println(cmds[i].b, 2);
    } else if (cmds[i].type == CMD_WAIT_BTN) {
      Serial.println("W");
    }
  }
}

// =================== EXECUTION ===================
void executeCmds_NoHome() {
  for (int i = 0; i < cmdCount; i++) {
    Command &c = cmds[i];
    if (c.type == CMD_POINT) {
      drawPoint(c.a);
    } else if (c.type == CMD_SEGMENT) {
      drawSegment(c.a, c.b);
    } else if (c.type == CMD_WAIT_BTN) {
      Serial.println("[WAIT] Press button...");
      while (!startPressed()) delay(10);
      while (startPressed()) delay(10);
    }
  }
}

void goToFieldThenDraw() {
  digitalWrite(PIN_LED_RUN, HIGH);

  // Normal motion params
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // 1) Drive into the field
  Serial.println("[MOVE] Entering field...");
  // If it goes the wrong way on your build: change FIELD_ENTRY_MM to negative here.
  xStepper.move(mmToSteps(FIELD_ENTRY_MM));
  while (xStepper.distanceToGo() != 0) xStepper.run();
  delay(200);

  // 2) Home on black stripe
  Serial.println("[HOME] Black stripe...");
  homeByBlackLine();

  // 3) Optional: move to safe start
  xMoveToMm(5);

  // 4) Draw
  Serial.println("[DRAW] Executing commands...");
  executeCmds_NoHome();

  digitalWrite(PIN_LED_RUN, LOW);
  Serial.println("DONE.");
}

// =================== TESTS ===================
void testServoSmooth(const char* name, Servo &s, int &cur, int center, int delta) {
  Serial.print("[TEST] "); Serial.print(name); Serial.println("...");
  servoGotoSmooth(s, cur, center);
  delay(250);
  servoGotoSmooth(s, cur, constrain(center + delta, 0, 180));
  delay(500);
  servoGotoSmooth(s, cur, constrain(center - delta, 0, 180));
  delay(500);
  servoGotoSmooth(s, cur, center);
  delay(500);
}

void runServoTests() {
  testServoSmooth("Shoulder", sShoulder, curShoulder, SHOULDER_0, 25);
  testServoSmooth("Elbow",    sElbow,    curElbow,    ELBOW_0,    25);

  Serial.println("[TEST] Pen...");
  penUpSmooth();   delay(500);
  penDownSmooth(); delay(500);
  penUpSmooth();   delay(500);
}

void runStepperTest() {
  Serial.println("[TEST] Stepper X...");
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  float x0 = constrain(stepsToMm(xStepper.currentPosition()), 0.0f, X_MAX_MM);
  float x1 = constrain(x0 + STEPPER_TEST_MM, 0.0f, X_MAX_MM);

  xMoveToMm(x1);
  delay(500);
  xMoveToMm(x0);
  delay(500);
}

// =================== SERIAL PARSER ===================
String lineBuf;

void printHelp() {
  Serial.println("Serial commands:");
  Serial.println("  P <x_mm>           - add point");
  Serial.println("  S <x1_mm> <x2_mm>  - add segment");
  Serial.println("  W                  - add wait button");
  Serial.println("  CLEAR              - clear command list");
  Serial.println("  LIST               - list commands");
  Serial.println("  RUN                - draw now (no button), includes field entry + homing");
  Serial.println("  THR <value>        - set BLACK_THRESHOLD");
  Serial.println("  BLACKLOW 0/1       - 1 if black<thr, 0 if black>thr");
  Serial.println("  HELP               - show help");
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  String u = s; u.toUpperCase();

  if (u == "HELP")  { printHelp(); return; }
  if (u == "CLEAR") { clearCmds(); return; }
  if (u == "LIST")  { listCmds();  return; }
  if (u == "RUN")   { goToFieldThenDraw(); return; }

  if (u.startsWith("THR ")) {
    int thr = u.substring(4).toInt();
    BLACK_THRESHOLD = thr;
    Serial.print("BLACK_THRESHOLD = "); Serial.println(BLACK_THRESHOLD);
    return;
  }

  if (u.startsWith("BLACKLOW ")) {
    int v = u.substring(9).toInt();
    BLACK_IS_LOW = (v != 0);
    Serial.print("BLACK_IS_LOW = "); Serial.println(BLACK_IS_LOW ? 1 : 0);
    return;
  }

  char c;
  float a, b;

  // P x
  if (sscanf(s.c_str(), " %c %f", &c, &a) == 2 && (c == 'P' || c == 'p')) {
    if (addCmd(CMD_POINT, a, 0)) Serial.println("OK: added POINT");
    else Serial.println("ERR: cmd list full");
    return;
  }

  // S x1 x2
  if (sscanf(s.c_str(), " %c %f %f", &c, &a, &b) == 3 && (c == 'S' || c == 's')) {
    if (addCmd(CMD_SEGMENT, a, b)) Serial.println("OK: added SEGMENT");
    else Serial.println("ERR: cmd list full");
    return;
  }

  // W
  if (u == "W") {
    if (addCmd(CMD_WAIT_BTN, 0, 0)) Serial.println("OK: added WAIT");
    else Serial.println("ERR: cmd list full");
    return;
  }

  Serial.println("Unknown command. Type HELP.");
}

void serialPoll() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineBuf.length()) {
        handleLine(lineBuf);
        lineBuf = "";
      }
    } else {
      lineBuf += ch;
      if (lineBuf.length() > 140) lineBuf = ""; // safety
    }
  }
}

// =================== MAIN ===================
void setup() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_RUN, OUTPUT);
  digitalWrite(PIN_LED_RUN, LOW);

  Serial.begin(115200);
  delay(300);

  // Stepper init
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // Servos init
  sShoulder.attach(PIN_SERVO_SHOULDER);
  sElbow.attach(PIN_SERVO_ELBOW);
  sPen.attach(PIN_SERVO_PEN);

  // Safe pose
  curShoulder = SHOULDER_0;
  curElbow    = ELBOW_0;
  curPen      = PEN_UP_DEG;

  sShoulder.writeMicroseconds(degToUs(curShoulder));
  sElbow.writeMicroseconds(degToUs(curElbow));
  sPen.writeMicroseconds(degToUs(curPen));
  delay(300);

  Serial.println("BOOT OK.");
  Serial.print("STEPS_PER_MM="); Serial.println(STEPS_PER_MM, 4);
  printHelp();

  // Default demo set (you can CLEAR and input via Serial)
  addCmd(CMD_POINT, 12, 0);
  addCmd(CMD_SEGMENT, 10, 23);
  addCmd(CMD_WAIT_BTN, 0, 0);
  addCmd(CMD_SEGMENT, 10, 15);
  addCmd(CMD_SEGMENT, 20, 25);
  addCmd(CMD_SEGMENT, 30, 35);

  // Auto-tests at startup
  runServoTests();
  runStepperTest();

  Serial.println("[READY] Load commands via Serial (P/S/W), then press button ONCE to start.");
}

void loop() {
  serialPoll();

  // Debug values (optional)
  static uint32_t t = 0;
  if (millis() - t > 500) {
    t = millis();
    Serial.print("Black="); Serial.print(readBlack());
    Serial.print(" X(mm)="); Serial.print(stepsToMm(xStepper.currentPosition()), 1);
    Serial.print(" CMDs="); Serial.println(cmdCount);
  }

  // Start button used ONCE to run drawing sequence
  static bool started = false;
  if (!started && startPressed()) {
    delay(30);
    if (startPressed()) {
      started = true;
      while (startPressed()) delay(10); // wait release
      Serial.println("[START] Button pressed. Enter field -> home -> draw.");
      goToFieldThenDraw();
      Serial.println("[FINISH] Reset Arduino to run again.");
    }
  }
}
