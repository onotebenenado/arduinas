#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

// Определение размеров дисплея
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Объекты для устройств
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// Переменные для фильтрации (простой фильтр)
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
const float alpha = 0.3; // Коэффициент фильтрации

void setup() {
  Serial.begin(115200);
  
  // Инициализация I2C
  Wire.begin(6, 7); // SDA=GPIO6, SCL=GPIO7 для ESP32-C3
  
  // Инициализация OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  delay(1000);
  
  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MPU6050 error!");
    display.display();
    while (1) {
      delay(10);
    }
  }
  
  // Настройка MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("System Ready!");
  display.display();
  delay(500);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Применяем простой фильтр низких частот
  accX = alpha * a.acceleration.x + (1 - alpha) * accX;
  accY = alpha * a.acceleration.y + (1 - alpha) * accY;
  accZ = alpha * a.acceleration.z + (1 - alpha) * accZ;
  
  gyroX = alpha * g.gyro.x + (1 - alpha) * gyroX;
  gyroY = alpha * g.gyro.y + (1 - alpha) * gyroY;
  gyroZ = alpha * g.gyro.z + (1 - alpha) * gyroZ;
  
  // Вывод на OLED
  display.clearDisplay();
  
  // Заголовок
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("MPU6050 Sensor Data");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Акселерометр
  display.setCursor(0, 15);
  display.println("Accelerometer:");
  
  display.setCursor(0, 25);
  display.print("X: ");
  display.print(accX);
  display.println(" m/s^2");
  
  display.setCursor(0, 35);
  display.print("Y: ");
  display.print(accY);
  display.println(" m/s^2");
  
  display.setCursor(0, 45);
  display.print("Z: ");
  display.print(accZ);
  display.println(" m/s^2");
  
  // Гироскоп (если нужно, можно раскомментировать)
  
  display.setCursor(65, 15);
  display.println("Gyro:");
  
  display.setCursor(65, 25);
  display.print("X: ");
  display.print(gyroX);
  display.println(" r/s");
  
  display.setCursor(65, 35);
  display.print("Y: ");
  display.print(gyroY);
  display.println(" r/s");
  
  display.setCursor(65, 45);
  display.print("Z: ");
  display.print(gyroZ);
  display.println(" r/s");
  
  
  // Температура
  display.setCursor(0, 55);
  display.print("Temp: ");
  display.print(temp.temperature);
  display.println(" C");
  
  display.display();
  
  // Также выводим в Serial Monitor
  Serial.print("Acc X: "); Serial.print(accX);
  Serial.print(" Y: "); Serial.print(accY);
  Serial.print(" Z: "); Serial.print(accZ);
  Serial.print(" | Gyro X: "); Serial.print(gyroX);
  Serial.print(" Y: "); Serial.print(gyroY);
  Serial.print(" Z: "); Serial.println(gyroZ);
  
  delay(100); // Задержка между обновлениями
}