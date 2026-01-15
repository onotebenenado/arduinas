#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>

// =================== ПИНЫ ===================
// ULN2003 IN1..IN4
const uint8_t PIN_M1 = 38;
const uint8_t PIN_M2 = 39;
const uint8_t PIN_M3 = 40;
const uint8_t PIN_M4 = 41;

// UI
const uint8_t PIN_START_BTN = 7;  // кнопка на GND, INPUT_PULLUP
const uint8_t PIN_LED_RUN = 13;   // индикатор выполнения

// Black line sensor (если используете хоуминг по полосе)
const uint8_t PIN_BLACK_SENSOR = A0;
int BLACK_THRESHOLD = 500;  // подберите по Serial
bool BLACK_IS_LOW = true;   // если чёрное даёт МЕНЬШЕ -> true, если БОЛЬШЕ -> false

// Servos
const uint8_t PIN_SERVO_SHOULDER = 33;
const uint8_t PIN_SERVO_ELBOW = 31;
const uint8_t PIN_SERVO_PEN = 30;

// =================== НАСТРОЙКИ СЕРВО ===================
// Управляем через microseconds (стабильнее)
int SERVO_US_MIN = 900;
int SERVO_US_MAX = 2100;

// Нули
int SHOULDER_0 = 90;
int ELBOW_0 = 90;

// Перо
int PEN_UP_DEG = 120;
int PEN_DOWN_DEG = 80;

// =================== ОСЬ X ===================
// Рабочая зона
const float X_MAX_MM = 297.0f;

// 28BYJ-48 half-step
const float STEPS_PER_MM = 20.36f;  // посчитано для рейки M2 и шестерни 32T

// скорости (реалистично для 28BYJ-48)
float X_SPEED_MM_S = 40.0f;
float X_ACCEL_MM_S2 = 150.0f;
float HOME_SPEED_MM_S = 20.0f;

// тестовый ход для шаговика
const float STEPPER_TEST_MM = 30.0f;

// =================== ДВИЖОК ===================
AccelStepper xStepper(AccelStepper::HALF4WIRE, PIN_M1, PIN_M3, PIN_M2, PIN_M4);
Servo sShoulder, sElbow, sPen;

// =================== КОМАНДЫ (МАССИВ) ===================
enum CmdType : uint8_t { CMD_POINT,
                         CMD_SEGMENT,
                         CMD_WAIT_BTN };

struct Command {
  CmdType type;
  float a;  // x or x1
  float b;  // unused or x2
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

int degToUs(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
}
void servoWriteDeg(Servo &s, int deg) {
  s.writeMicroseconds(degToUs(deg));
}

int readBlack() {
  return analogRead(PIN_BLACK_SENSOR);
}

bool isBlack(int v) {
  // BLACK_IS_LOW=true -> чёрное даёт меньше порога
  return BLACK_IS_LOW ? (v < BLACK_THRESHOLD) : (v > BLACK_THRESHOLD);
}

// =================== ПЕРО ===================
void penUp() {
  servoWriteDeg(sPen, PEN_UP_DEG);
  delay(150);
}
void penDown() {
  servoWriteDeg(sPen, PEN_DOWN_DEG);
  delay(200);
}

// =================== ОСЬ X ===================
void xMoveToMm(float x_mm) {
  x_mm = constrain(x_mm, 0.0f, X_MAX_MM);
  xStepper.moveTo(mmToSteps(x_mm));
  while (xStepper.distanceToGo() != 0) xStepper.run();
}

void homeByBlackLine() {
  xStepper.setMaxSpeed(mmToSteps(HOME_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // стартовый сдвиг
  xStepper.move(mmToSteps(10));
  while (xStepper.distanceToGo() != 0) xStepper.run();

  // едем вправо до чёрной полосы
  xStepper.moveTo(mmToSteps(X_MAX_MM));
  while (xStepper.distanceToGo() != 0) {
    xStepper.run();
    if (isBlack(readBlack())) {
      xStepper.stop();
      while (xStepper.isRunning()) xStepper.run();
      break;
    }
  }
  xStepper.setCurrentPosition(0);  // правая граница полосы = X0
  xMoveToMm(5);                    // отъезд в белую зону
}

// =================== РИСОВАНИЕ (примитивы) ===================
void drawPoint(float x_mm) {
  xMoveToMm(x_mm);
  penDown();
  delay(120);
  penUp();
}

void drawSegment(float x1_mm, float x2_mm) {
  if (x2_mm < x1_mm) {
    float t = x1_mm;
    x1_mm = x2_mm;
    x2_mm = t;
  }
  xMoveToMm(x1_mm);
  penDown();
  xMoveToMm(x2_mm);
  penUp();
}

// =================== ТЕСТЫ ===================
void testOneServo(const char *name, Servo &s, int centerDeg, int deltaDeg) {
  Serial.print("[TEST] ");
  Serial.print(name);
  Serial.println("...");
  servoWriteDeg(s, centerDeg);
  delay(300);

  servoWriteDeg(s, constrain(centerDeg + deltaDeg, 0, 180));
  delay(500);

  servoWriteDeg(s, constrain(centerDeg - deltaDeg, 0, 180));
  delay(500);

  servoWriteDeg(s, centerDeg);
  delay(500);
}

void runServoTests() {
  // плечо/локоть: небольшой ход, чтобы не упереться в механику
  testOneServo("Shoulder", sShoulder, SHOULDER_0, 25);
  testOneServo("Elbow", sElbow, ELBOW_0, 25);

  // перо: вверх-вниз-вверх (как реальная работа)
  Serial.println("[TEST] Pen...");
  penUp();
  delay(500);
  penDown();
  delay(500);
  penUp();
  delay(500);
}

void runStepperTest() {
  Serial.println("[TEST] Stepper X...");
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  float x0 = constrain(stepsToMm(xStepper.currentPosition()), 0.0f, X_MAX_MM);
  float x1 = constrain(x0 + STEPPER_TEST_MM, 0.0f, X_MAX_MM);
  float x2 = constrain(x0, 0.0f, X_MAX_MM);

  xMoveToMm(x1);
  delay(500);
  xMoveToMm(x2);
  delay(500);
}

// =================== КОМАНДЫ: добавление/листинг/выполнение ===================
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
  Serial.print("CMD count = ");
  Serial.println(cmdCount);
  for (int i = 0; i < cmdCount; i++) {
    Serial.print(i);
    Serial.print(": ");
    if (cmds[i].type == CMD_POINT) {
      Serial.print("P ");
      Serial.println(cmds[i].a, 2);
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

void executeCmds() {
  digitalWrite(PIN_LED_RUN, HIGH);

  // Хоуминг перед рисованием (по ТЗ)
  homeByBlackLine();

  // выполнение
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

  digitalWrite(PIN_LED_RUN, LOW);
  Serial.println("DONE.");
}

// =================== SERIAL PARSER ===================
String lineBuf;

void printHelp() {
  Serial.println("Serial commands:");
  Serial.println("  P <x_mm>         - add point");
  Serial.println("  S <x1_mm> <x2_mm> - add segment");
  Serial.println("  W                - add wait button");
  Serial.println("  CLEAR            - clear command list");
  Serial.println("  LIST             - list commands");
  Serial.println("  RUN              - execute now");
  Serial.println("  THR <value>      - set BLACK_THRESHOLD");
  Serial.println("  BLACKLOW 0/1     - 1 if black<thr, 0 if black>thr");
  Serial.println("  HELP             - show help");
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  // uppercase for keywords
  String u = s;
  u.toUpperCase();

  if (u == "HELP") {
    printHelp();
    return;
  }
  if (u == "CLEAR") {
    clearCmds();
    return;
  }
  if (u == "LIST") {
    listCmds();
    return;
  }
  if (u == "RUN") {
    executeCmds();
    return;
  }

  if (u.startsWith("THR ")) {
    int thr = u.substring(4).toInt();
    BLACK_THRESHOLD = thr;
    Serial.print("BLACK_THRESHOLD = ");
    Serial.println(BLACK_THRESHOLD);
    return;
  }
  if (u.startsWith("BLACKLOW ")) {
    int v = u.substring(9).toInt();
    BLACK_IS_LOW = (v != 0);
    Serial.print("BLACK_IS_LOW = ");
    Serial.println(BLACK_IS_LOW ? 1 : 0);
    return;
  }

  // parse drawing commands
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
      if (lineBuf.length() > 120) lineBuf = "";  // защита
    }
  }
}

// =================== SETUP/LOOP ===================
void setup() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_RUN, OUTPUT);
  digitalWrite(PIN_LED_RUN, LOW);

  Serial.begin(115200);
  delay(300);

  // init motors/servos
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  sShoulder.attach(PIN_SERVO_SHOULDER);
  sElbow.attach(PIN_SERVO_ELBOW);
  sPen.attach(PIN_SERVO_PEN);

  // safe pose
  servoWriteDeg(sShoulder, SHOULDER_0);
  servoWriteDeg(sElbow, ELBOW_0);
  penUp();

  Serial.println("BOOT OK.");
  Serial.print("STEPS_PER_MM=");
  Serial.println(STEPS_PER_MM, 4);

  // По умолчанию можно загрузить пример из ТЗ (чтобы не было пусто)
  addCmd(CMD_POINT, 12, 0);
  addCmd(CMD_SEGMENT, 10, 23);
  addCmd(CMD_SEGMENT, 10, 15);
  addCmd(CMD_SEGMENT, 20, 25);
  addCmd(CMD_SEGMENT, 30, 35);

  printHelp();

  // Автотесты при старте
  runServoTests();
  runStepperTest();

  Serial.println("[READY] Send commands via Serial, then press the button ONCE to start drawing.");
}

void loop() {
  // всегда слушаем Serial (можно настраивать/вводить команды)
  serialPoll();

  // вывод отладочных значений раз в 400 мс
  static uint32_t t = 0;
  if (millis() - t > 400) {
    t = millis();
    Serial.print("Black=");
    Serial.print(readBlack());
    Serial.print(" X(mm)=");
    Serial.print(stepsToMm(xStepper.currentPosition()), 1);
    Serial.print(" CMDs=");
    Serial.println(cmdCount);
  }

  // КНОПКА: используется ОДИН РАЗ — запускает рисование
  static bool started = false;
  if (!started && startPressed()) {
    delay(30);
    if (startPressed()) {
      started = true;
      while (startPressed()) delay(10);  // дождаться отпускания
      Serial.println("[START] Drawing...");
      executeCmds();
      Serial.println("[FINISH] (To run again, reset Arduino)");
    }
  }
}
