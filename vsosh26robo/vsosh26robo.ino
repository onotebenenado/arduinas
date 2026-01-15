#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>

// =================== НАСТРОЙКИ ПОД ВАШЕ ЖЕЛЕЗО ===================

// --- ULN2003 pins (IN1..IN4) ---
const uint8_t PIN_M1 = 38;
const uint8_t PIN_M2 = 39;
const uint8_t PIN_M3 = 40;
const uint8_t PIN_M4 = 41;

// --- UI ---
const uint8_t PIN_MODE_POT = A1;  // потенциометр выбора режима (0..1023)
const uint8_t PIN_START_BTN = 7;  // кнопка старт (на GND)
const uint8_t PIN_LED_RUN = 13;   // встроенный LED

// --- black line sensor ---
const uint8_t PIN_BLACK_SENSOR = A0;
int BLACK_THRESHOLD = 500;  // подберите по Serial (ниже объясню)

// --- Servos ---
const uint8_t PIN_SERVO_SHOULDER = 30;
const uint8_t PIN_SERVO_ELBOW = 32;
const uint8_t PIN_SERVO_PEN = 34;

// Для стабильности серво лучше microseconds диапазон
int SERVO_US_MIN = 900;
int SERVO_US_MAX = 2100;

// Нули и ограничения (подберите по калибровке)
int SHOULDER_0 = 90;
int ELBOW_0 = 90;

int SHOULDER_MIN = 15, SHOULDER_MAX = 165;
int ELBOW_MIN = 15, ELBOW_MAX = 165;

// Перо (подберите)
int PEN_UP_DEG = 120;
int PEN_DOWN_DEG = 80;

// --- X calibration ---
// 28BYJ-48: обычно 4096 half-steps на оборот (бывает 2048/4096 — зависит от версии).
// Мы используем HALFSTEP, поэтому стартуем с 4096.
const float STEPS_PER_REV = 4096.0f;

// !!! ВАЖНО: STEPS_PER_MM зависит от вашей механики (шестерня на рейке / ремень / винт).
// Поставил заглушку. Вам нужно заменить на своё значение.
float STEPS_PER_MM = 20.0f;  // <- подберите!

// рабочая зона по X (по вашему полигону)
const float X_MAX_MM = 297.0f;

// Скорости 28BYJ-48 (реалистичные):
// шаги/сек = мм/с * STEPS_PER_MM.
// Сильно большие значения приводят к пропускам.
float X_SPEED_MM_S = 40.0f;  // начните с 30–50 мм/с
float X_ACCEL_MM_S2 = 150.0f;

// поиск чёрной полосы
float HOME_SPEED_MM_S = 20.0f;

// =================================================================

// AccelStepper in HALFSTEP mode for 4-wire stepper
AccelStepper xStepper(AccelStepper::HALF4WIRE, PIN_M1, PIN_M3, PIN_M2, PIN_M4);

Servo sShoulder, sElbow, sPen;

// ---------- helpers ----------
int degToUs(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
}

void servoWriteDeg(Servo &s, int deg) {
  s.writeMicroseconds(degToUs(deg));
}

bool startPressed() {
  return digitalRead(PIN_START_BTN) == LOW;
}

int readMode6() {
  int v = analogRead(PIN_MODE_POT);  // 0..1023
  int mode = (v * 6) / 1024;         // 0..5
  return constrain(mode, 0, 5);
}

int readBlack() {
  return analogRead(PIN_BLACK_SENSOR);
}

long mmToSteps(float mm) {
  return (long)lround(mm * STEPS_PER_MM);
}

float stepsToMm(long st) {
  return (float)st / STEPS_PER_MM;
}

// ---------- pen ----------
void penUp() {
  servoWriteDeg(sPen, PEN_UP_DEG);
  delay(150);
}
void penDown() {
  servoWriteDeg(sPen, PEN_DOWN_DEG);
  delay(200);
}

// ---------- X movement ----------
void xMoveToMm(float x_mm) {
  x_mm = constrain(x_mm, 0.0f, X_MAX_MM);
  xStepper.moveTo(mmToSteps(x_mm));
  while (xStepper.distanceToGo() != 0) {
    xStepper.run();
  }
}

// ---------- homing by black line ----------
void homeByBlackLine() {
  // Настраиваем скорости под хоуминг
  xStepper.setMaxSpeed(mmToSteps(HOME_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // Сдвиг вправо чуть-чуть, чтобы “не стоять на месте”
  xStepper.move(mmToSteps(10));
  while (xStepper.distanceToGo() != 0) xStepper.run();

  // Едем вправо до тех пор, пока не увидим чёрную полосу
  xStepper.moveTo(mmToSteps(X_MAX_MM));

  while (xStepper.distanceToGo() != 0) {
    xStepper.run();
    int v = readBlack();

    // ВАЖНО: у некоторых датчиков чёрное = БОЛЬШЕ.
    // Если у вас наоборот — поменяйте знак (<) на (>).
    if (v < BLACK_THRESHOLD) {
      xStepper.stop();
      while (xStepper.isRunning()) xStepper.run();
      break;
    }
  }

  // Текущую позицию принимаем за 0
  xStepper.setCurrentPosition(0);

  // Отъезжаем от полосы чуть вправо, чтобы начать рисовать в белой зоне
  xMoveToMm(5);
}

// ---------- primitives (1D drawing along X) ----------
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

// ---------- 6 modes ----------
void runMode(int mode) {
  digitalWrite(PIN_LED_RUN, HIGH);

  // скорости для обычной работы
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // Всегда начинаем с хоуминга
  homeByBlackLine();

  switch (mode) {
    case 0:
      // Режим 1: точка на 12 мм
      drawPoint(12);
      break;

    case 1:
      // Режим 2: отрезок длиной 13 мм начиная с 10 мм
      drawSegment(10, 23);
      break;

    case 2:
      // Режим 3: по нажатию кнопки — 3 отрезка по 5 мм с промежутками
      while (!startPressed()) delay(10);
      while (startPressed()) delay(10);  // отпускание
      drawSegment(10, 15);
      drawSegment(20, 25);
      drawSegment(30, 35);
      break;

    case 3:
      // Режим 4: тест — линия 0..50
      drawSegment(0, 50);
      break;

    case 4:
      // Режим 5: 10 точек каждые 10 мм
      for (int i = 0; i < 10; i++) drawPoint(i * 10.0f);
      break;

    case 5:
      // Режим 6: сервисный — просто хоуминг и стоп
      break;
  }

  digitalWrite(PIN_LED_RUN, LOW);
}

void setup() {
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_RUN, OUTPUT);

  Serial.begin(115200);
  delay(300);

  // Stepper init
  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // Servos init
  sShoulder.attach(PIN_SERVO_SHOULDER);
  sElbow.attach(PIN_SERVO_ELBOW);
  sPen.attach(PIN_SERVO_PEN);

  // безопасная поза
  servoWriteDeg(sShoulder, SHOULDER_0);
  servoWriteDeg(sElbow, ELBOW_0);
  penUp();

  Serial.println("READY. Check black sensor values, tune BLACK_THRESHOLD and STEPS_PER_MM.");
}

void loop() {
  int mode = readMode6();

  static uint32_t t = 0;
  if (millis() - t > 300) {
    t = millis();
    Serial.print("Mode: ");
    Serial.print(mode);
    Serial.print(" | Black: ");
    Serial.print(readBlack());
    Serial.print(" | X(mm): ");
    Serial.println(stepsToMm(xStepper.currentPosition()));
  }

  if (startPressed()) {
    delay(30);
    if (startPressed()) {
      runMode(mode);
      while (startPressed()) delay(10);
    }
  }
}
