// Single TOF400C / VL53L1X test for an ESP32 DevKit V1.
//
// Wiring:
//   Sensor VIN  -> ESP32 3V3
//   Sensor GND  -> ESP32 GND
//   Sensor SDA  -> ESP32 GPIO 21
//   Sensor SCL  -> ESP32 GPIO 22
//   Sensor INT and SHUT are not used and can remain disconnected.
//
// Required Arduino library: "VL53L1X" by Pololu.
// Open Serial Monitor at 115200 baud after uploading.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;

const uint32_t SERIAL_BAUD = 115200;
const uint32_t SENSOR_TIMEOUT_MS = 500;
const uint32_t MEASUREMENT_PERIOD_MS = 100;
const uint32_t PRINT_PERIOD_MS = 250;

// Only used to make the Serial Monitor output easier to interpret.
// Change this later to match the final door geometry.
const uint16_t NEAR_THRESHOLD_MM = 500;

VL53L1X tofSensor;
bool sensorReady = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("Single TOF400C / VL53L1X test starting");
  Serial.printf("I2C wiring: SDA=GPIO %u, SCL=GPIO %u\n",
                I2C_SDA_PIN, I2C_SCL_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  tofSensor.setTimeout(SENSOR_TIMEOUT_MS);

  // A single VL53L1X can use its factory 7-bit I2C address, 0x29.
  if (!tofSensor.init()) {
    Serial.println("ERROR: TOF sensor did not respond at I2C address 0x29.");
    Serial.println("Check VIN, GND, SDA, and SCL wiring.");
    return;
  }

  tofSensor.setDistanceMode(VL53L1X::Long);

  if (!tofSensor.setMeasurementTimingBudget(50000)) {
    Serial.println("ERROR: Could not set the TOF sensor timing budget.");
    return;
  }

  tofSensor.startContinuous(MEASUREMENT_PERIOD_MS);
  sensorReady = true;

  Serial.println("TOF sensor ready at I2C address 0x29.");
  Serial.println("Continuous distance readings follow:");
}

void loop() {
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs < PRINT_PERIOD_MS) {
    delay(1);
    return;
  }
  lastPrintMs = millis();

  Serial.printf("[%lu ms] TOF: ", millis());

  if (!sensorReady) {
    Serial.println("NOT INITIALIZED");
    return;
  }

  const uint16_t distanceMm = tofSensor.read();

  if (tofSensor.timeoutOccurred()) {
    Serial.println("TIMEOUT");
    return;
  }

  Serial.printf("%u mm (%s)\n", distanceMm,
                distanceMm <= NEAR_THRESHOLD_MM ? "NEAR" : "CLEAR");
}
