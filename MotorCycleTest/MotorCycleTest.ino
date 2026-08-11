// One-cycle actuator test for an ESP32 DevKit V1 and DRV8871.
//
// Wiring used by this sketch:
//   ESP32 GPIO 25 -> DRV8871 IN1
//   ESP32 GPIO 26 -> DRV8871 IN2
//   ESP32 GND     -> DRV8871 GND
//
// Power the actuator from a suitable external supply through the DRV8871.
// Do not power the actuator from the ESP32.

const uint8_t MOTOR_IN1_PIN = 25;
const uint8_t MOTOR_IN2_PIN = 26;

// Begin with a short movement. Increase this only after confirming that the
// actuator does not bind or remain stalled at the end of its travel.
const unsigned long MOVE_TIME_MS = 1500;
const unsigned long PAUSE_TIME_MS = 1000;
const unsigned long DIRECTION_CHANGE_DELAY_MS = 100;

void stopMotor() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void extendMotor() {
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void retractMotor() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

void setup() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  // Keep the actuator stopped while the ESP32 finishes starting.
  stopMotor();
  delay(1000);

  extendMotor();
  delay(MOVE_TIME_MS);

  stopMotor();
  delay(PAUSE_TIME_MS);

  // Brief dead time prevents reversing the driver instantaneously.
  stopMotor();
  delay(DIRECTION_CHANGE_DELAY_MS);

  retractMotor();
  delay(MOVE_TIME_MS);

  stopMotor();
}

void loop() {
  // Intentionally empty: the extend/retract sequence runs only once per reset.
}
