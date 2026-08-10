const int IN1 = 25;
const int IN2 = 26;

const unsigned long MOVE_TIME_MS = 1500;
const unsigned long PAUSE_TIME_MS = 1000;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  stopMotor();
  delay(1000);
}

void loop() {
  extendMotor();
  delay(MOVE_TIME_MS);

  stopMotor();
  delay(PAUSE_TIME_MS);

  retractMotor();
  delay(MOVE_TIME_MS);

  stopMotor();
  delay(PAUSE_TIME_MS);
}

void extendMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retractMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); // coast/stop
}
