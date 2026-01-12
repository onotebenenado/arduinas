byte ena = 3;
byte in2 = 14;
byte in4 = 15;

void setup() {
  pinMode(ena, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  analogWrite(ena, 255);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH);
}
