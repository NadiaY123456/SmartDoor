// Dual TOF400C / VL53L1X test for an ESP32 DevKit V1.
//
// Wiring:
//   Both sensor VIN  -> ESP32 3V3
//   Both sensor GND  -> ESP32 GND
//   Both sensor SDA  -> ESP32 GPIO 21
//   Both sensor SCL  -> ESP32 GPIO 22
//   Outside SHUT     -> ESP32 GPIO 32
//   Inside SHUT      -> ESP32 GPIO 33
//   INT pins are not used.
//
// Required Arduino library: "VL53L1X" by Pololu.
// Open Serial Monitor at 115200 baud after uploading.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;

const uint8_t OUTSIDE_SHUT_PIN = 32;
const uint8_t INSIDE_SHUT_PIN = 33;

// These are 7-bit I2C addresses. Each must be unique.
const uint8_t OUTSIDE_I2C_ADDRESS = 0x2A;
const uint8_t INSIDE_I2C_ADDRESS = 0x2B;

const uint32_t SERIAL_BAUD = 115200;
const uint32_t SENSOR_TIMEOUT_MS = 500;
const uint32_t MEASUREMENT_PERIOD_MS = 100;
const uint32_t PRINT_PERIOD_MS = 250;

// Only used to make the Serial Monitor output easier to interpret.
// Change this later to match the final door geometry.
const uint16_t NEAR_THRESHOLD_MM = 500;

VL53L1X outsideSensor;
VL53L1X insideSensor;

bool outsideSensorReady = false;
bool insideSensorReady = false;

void disableSensor(uint8_t shutPin) {
  pinMode(shutPin, OUTPUT);
  digitalWrite(shutPin, LOW);
}

void enableSensor(uint8_t shutPin) {
  digitalWrite(shutPin, HIGH);
  delay(20);
}

bool initializeSensor(VL53L1X &sensor, uint8_t shutPin,
                      uint8_t newAddress, const char *name) {
  enableSensor(shutPin);
  sensor.setTimeout(SENSOR_TIMEOUT_MS);

  // Only this sensor is at the factory address (0x29) while init() runs.
  if (!sensor.init()) {
    Serial.printf("ERROR: %s sensor did not respond at address 0x29.\n", name);
    Serial.println("Check its VIN, GND, SDA, SCL, and SHUT connections.");

    // Keep a failed sensor off so it cannot interfere with the other unit.
    disableSensor(shutPin);
    return false;
  }

  sensor.setAddress(newAddress);
  sensor.setDistanceMode(VL53L1X::Long);

  if (!sensor.setMeasurementTimingBudget(50000)) {
    Serial.printf("ERROR: Could not set the %s sensor timing budget.\n", name);
    disableSensor(shutPin);
    return false;
  }

  sensor.startContinuous(MEASUREMENT_PERIOD_MS);
  Serial.printf("%s sensor ready at I2C address 0x%02X.\n", name, newAddress);
  return true;
}

void printMeasurement(const char *name, VL53L1X &sensor, bool ready) {
  Serial.printf("%s: ", name);

  if (!ready) {
    Serial.print("NOT INITIALIZED");
    return;
  }

  const uint16_t distanceMm = sensor.read();

  if (sensor.timeoutOccurred()) {
    Serial.print("TIMEOUT");
    return;
  }

  Serial.printf("%u mm (%s)", distanceMm,
                distanceMm <= NEAR_THRESHOLD_MM ? "NEAR" : "CLEAR");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("Dual TOF400C / VL53L1X test starting");

  // Both sensors start with the same address, so hold both in shutdown first.
  disableSensor(OUTSIDE_SHUT_PIN);
  disableSensor(INSIDE_SHUT_PIN);
  delay(20);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // Bring up and re-address one sensor at a time.
  outsideSensorReady = initializeSensor(
    outsideSensor, OUTSIDE_SHUT_PIN, OUTSIDE_I2C_ADDRESS, "OUTSIDE");

  insideSensorReady = initializeSensor(
    insideSensor, INSIDE_SHUT_PIN, INSIDE_I2C_ADDRESS, "INSIDE");

  if (!outsideSensorReady && !insideSensorReady) {
    Serial.println("ERROR: Neither TOF sensor initialized.");
  } else {
    Serial.println("Continuous distance readings follow:");
  }
}

void loop() {
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs < PRINT_PERIOD_MS) {
    delay(1);
    return;
  }
  lastPrintMs = millis();

  Serial.printf("[%lu ms] ", millis());
  printMeasurement("OUTSIDE", outsideSensor, outsideSensorReady);
  Serial.print(" | ");
  printMeasurement("INSIDE", insideSensor, insideSensorReady);
  Serial.println();
}
