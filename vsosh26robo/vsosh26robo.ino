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
const uint8_t PIN_MODE_POT = A1;  // выбор 1 из 6 режимов потенциометром :contentReference[oaicite:6]{index=6}
const uint8_t PIN_START_BTN = 7;  // кнопка старт на GND (INPUT_PULLUP)
const uint8_t PIN_LED_RUN = 13;   // индикатор выполнения (горит постоянно во время работы) :contentReference[oaicite:7]{index=7}

// Датчик чёрной полосы
const uint8_t PIN_BLACK_SENSOR = A0;
int BLACK_THRESHOLD = 500;  // подберите по Serial (см. ниже)

// Серво (MG90S/SG90)
const uint8_t PIN_SERVO_SHOULDER = 33;
const uint8_t PIN_SERVO_ELBOW = 31;
const uint8_t PIN_SERVO_PEN = 30;

// =================== СЕРВО НАСТРОЙКИ ===================
// Стабильнее управлять через microseconds
int SERVO_US_MIN = 900;
int SERVO_US_MAX = 2100;

// Нули (подберите по калибровке)
int SHOULDER_0 = 90;
int ELBOW_0 = 90;

// Ограничения, чтобы не ломать механику
int SHOULDER_MIN = 15, SHOULDER_MAX = 165;
int ELBOW_MIN = 15, ELBOW_MAX = 165;

// Перо вверх/вниз (подберите так, чтобы поднималось >= 5 мм) :contentReference[oaicite:8]{index=8}
int PEN_UP_DEG = 120;
int PEN_DOWN_DEG = 80;

// =================== ОСЬ X (28BYJ-48 + рейка M2) ===================
// Рабочая область по X для региона: 297 мм :contentReference[oaicite:9]{index=9}
const float X_MAX_MM = 297.0f;

// 28BYJ-48 half-step ~4096 шаг/оборот
const float STEPS_PER_REV = 4096.0f;

// Рейка модуль M2 (из ЕТЗ) :contentReference[oaicite:10]{index=10}
// Шестерня 32 зуба:
// mm_per_rev = z * pi * m = 32 * pi * 2 = 201.06 мм/оборот
// steps/mm = 4096 / 201.06 = 20.36
const float STEPS_PER_MM = 20.36f;

// Реалистичные скорости для 28BYJ-48 (чтобы не пропускал шаги)
float X_SPEED_MM_S = 40.0f;  // попробуйте 30..60
float X_ACCEL_MM_S2 = 150.0f;

float HOME_SPEED_MM_S = 20.0f;

// =================== ДВИЖОК ===================
AccelStepper xStepper(AccelStepper::HALF4WIRE, PIN_M1, PIN_M3, PIN_M2, PIN_M4);
Servo sShoulder, sElbow, sPen;

// ---------- helpers ----------

// =================== СЦЕНАРИЙ РИСОВАНИЯ ===================
// 👉 МЕНЯЕТСЯ ТОЛЬКО ЭТА ФУНКЦИЯ 👈

void drawTask() {

  // // ПРИМЕР 1 — точка в 12 мм
  // POINT(12);

  // // ПРИМЕР 2 — отрезок длиной 13 мм
  // SEGMENT(10, 23);

  // // ПРИМЕР 3 — ждать кнопку, затем 3 отрезка
  // WAIT_BUTTON();
  // SEGMENT(10, 15);
  // SEGMENT(20, 25);
  // SEGMENT(30, 35);
  POINT(5);
  POINT(15);
  POINT(25);
  SEGMENT(40, 60);
}

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
  int v = analogRead(PIN_MODE_POT);
  int mode = (v * 6) / 1024;  // 0..5
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

// ---------- X move ----------
void xMoveToMm(float x_mm) {
  x_mm = constrain(x_mm, 0.0f, X_MAX_MM);
  xStepper.moveTo(mmToSteps(x_mm));
  while (xStepper.distanceToGo() != 0) xStepper.run();
}

// ---------- homing by black stripe ----------
void homeByBlackLine() {
  xStepper.setMaxSpeed(mmToSteps(HOME_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  // небольшой стартовый сдвиг
  xStepper.move(mmToSteps(10));
  while (xStepper.distanceToGo() != 0) xStepper.run();

  // едем вправо, пока не найдём чёрную полосу
  xStepper.moveTo(mmToSteps(X_MAX_MM));
  while (xStepper.distanceToGo() != 0) {
    xStepper.run();
    int v = readBlack();

    // Если у вашего датчика чёрное = БОЛЬШЕ, замените "<" на ">"
    if (v < BLACK_THRESHOLD) {
      xStepper.stop();
      while (xStepper.isRunning()) xStepper.run();
      break;
    }
  }

  // чёрная полоса = X=0
  xStepper.setCurrentPosition(0);

  // отъезжаем на белую зону
  xMoveToMm(5);
}

// ---------- primitives (координатная прямая по X) ----------
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

// ---------- 6 modes (по ТЗ/полигону региона) ----------
void runMode(int mode) {
  digitalWrite(PIN_LED_RUN, HIGH);

  xStepper.setMaxSpeed(mmToSteps(X_SPEED_MM_S));
  xStepper.setAcceleration(mmToSteps(X_ACCEL_MM_S2));

  homeByBlackLine();

  switch (mode) {
    case 0:
      // Режим 0: сервис — только хоуминг и стоп
      break;

    case 1:
      // Режим 1: точка в координатах 12 мм :contentReference[oaicite:11]{index=11}
      drawPoint(12);
      break;

    case 2:
      // Режим 2: отрезок длиной 13 мм :contentReference[oaicite:12]{index=12}
      // Начинаем с 10 мм, рисуем до 23 мм
      drawSegment(10, 23);
      break;

    case 3:
      // // Режим 3: после нажатия кнопки — 3 отрезка 5 мм, расстояние 5 мм :contentReference[oaicite:13]{index=13}
      // while (!startPressed()) delay(10);
      // while (startPressed()) delay(10);

      // drawSegment(10, 15);
      // drawSegment(20, 25);
      // drawSegment(30, 35);
      // break;
      drawTask();
      break;

    case 4:
      // Режим 4: тестовая линия 0..50 мм
      drawSegment(0, 50);
      break;

    case 5:
      // Режим 5: 10 точек через 10 мм
      for (int i = 0; i < 10; i++) drawPoint(i * 10.0f);
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

  // neutral pose
  servoWriteDeg(sShoulder, SHOULDER_0);
  servoWriteDeg(sElbow, ELBOW_0);
  penUp();

  Serial.println("READY. Tune BLACK_THRESHOLD, PEN_UP/PEN_DOWN if needed.");
  Serial.print("STEPS_PER_MM = ");
  Serial.println(STEPS_PER_MM, 4);
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
