int BLS = A0;  // Black line sensor

float setVoltBLS3 = 3.0;  // устанавливаем напряжение BLS
float setVoltBLS5 = 5.0;


void setup() {
  Serial.begin(9600);
  pinMode(BLS, INPUT);
}

void loop() {
  // int valBLS = analogRead(BLS);  // читаем и записываем данные с датчика
  float voltBLS = analogRead(BLS) * setVoltBLS3 / 1024.0;

  Serial.print("BLS val: ");
  Serial.print(voltBLS);
  Serial.println("V");
}