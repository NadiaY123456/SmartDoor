// Integrated smart cat-door controller for an ESP32 DevKit V1.
//
// TOF400C / VL53L1X wiring:
//   Both SDA       -> GPIO 21
//   Both SCL       -> GPIO 22
//   Outside SHUT   -> GPIO 32
//   Inside SHUT    -> GPIO 33
//   Both VIN       -> 3V3
//   Both GND       -> common GND
//
// JRD-4035 UART wiring:
//   Reader TX      -> GPIO 16 (ESP32 RX)
//   Reader RX      -> GPIO 17 (ESP32 TX)
//   Reader GND     -> common GND
//   Power the reader with the voltage required by its particular board.
//
// DRV8871 control wiring (one driver per actuator):
//   Outside driver 1 / left:  IN1 -> GPIO 25, IN2 -> GPIO 26
//   Outside driver 2 / right: IN1 -> GPIO 27, IN2 -> GPIO 14
//   Inside driver 1 / left:   IN1 -> GPIO 13, IN2 -> GPIO 18
//   Inside driver 2 / right:  IN1 -> GPIO 19, IN2 -> GPIO 23
//
// Use a suitable external actuator supply. Connect its ground, all DRV8871
// grounds, the sensor grounds, the RFID ground, and ESP32 ground together.
// Do not power an actuator from the ESP32.
//
// Required Arduino library: "VL53L1X" by Pololu.
// Open Serial Monitor at 115200 baud.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// --------------------------- Adjustable settings ---------------------------

const uint16_t TOF_TRIGGER_DISTANCE_MM = 100;  // 100 mm = 10 cm.
const uint32_t CONDITIONS_CLEAR_DELAY_MS = 10000;
const uint32_t ACTUATOR_TRAVEL_TIME_MS = 1500;

// A valid tag remains "present" briefly between reader polls. Increase this
// if the reader occasionally misses one poll while the tag is stationary.
const uint32_t RFID_PRESENCE_HOLD_MS = 1000;

const uint32_t SERIAL_REPORT_PERIOD_MS = 500;
const uint32_t TOF_MEASUREMENT_PERIOD_MS = 100;
const uint32_t TOF_STALE_AFTER_MS = 600;
const uint32_t RFID_POLL_PERIOD_MS = 250;
const uint32_t MOTOR_DIRECTION_CHANGE_DELAY_MS = 100;

// Set a motor's value to false if that actuator moves opposite to the label
// printed by the program. Alternatively, swap that driver's OUT1/OUT2 wires.
const bool OUTSIDE_LEFT_EXTEND_IS_IN1_HIGH = true;
const bool OUTSIDE_RIGHT_EXTEND_IS_IN1_HIGH = true;
const bool INSIDE_LEFT_EXTEND_IS_IN1_HIGH = true;
const bool INSIDE_RIGHT_EXTEND_IS_IN1_HIGH = true;

// ------------------------------- Pin layout --------------------------------

const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;
const uint8_t OUTSIDE_SHUT_PIN = 32;
const uint8_t INSIDE_SHUT_PIN = 33;
const uint8_t OUTSIDE_I2C_ADDRESS = 0x2A;
const uint8_t INSIDE_I2C_ADDRESS = 0x2B;

const uint8_t RFID_RX_PIN = 16;
const uint8_t RFID_TX_PIN = 17;

const uint32_t USB_SERIAL_BAUD = 115200;
const uint32_t RFID_UART_BAUD = 115200;
const uint32_t TOF_TIMEOUT_MS = 500;

// ------------------------------- TOF sensors --------------------------------

VL53L1X outsideSensor;
VL53L1X insideSensor;

struct TofReading {
  bool initialized;
  bool valid;
  bool timedOut;
  uint16_t distanceMm;
  uint32_t lastReadingMs;
};

TofReading outsideTof = {false, false, false, 0, 0};
TofReading insideTof = {false, false, false, 0, 0};

void holdSensorInShutdown(uint8_t shutPin) {
  pinMode(shutPin, OUTPUT);
  digitalWrite(shutPin, LOW);
}

void releaseSensorFromShutdown(uint8_t shutPin) {
  digitalWrite(shutPin, HIGH);
  delay(20);
}

bool initializeTofSensor(VL53L1X &sensor, TofReading &reading,
                         uint8_t shutPin, uint8_t newAddress,
                         const char *name) {
  releaseSensorFromShutdown(shutPin);
  sensor.setTimeout(TOF_TIMEOUT_MS);

  if (!sensor.init()) {
    Serial.printf("ERROR: %s TOF did not initialize. Check VIN, GND, SDA, SCL, and SHUT.\n",
                  name);
    holdSensorInShutdown(shutPin);
    return false;
  }

  sensor.setAddress(newAddress);
  sensor.setDistanceMode(VL53L1X::Long);
  if (!sensor.setMeasurementTimingBudget(50000)) {
    Serial.printf("ERROR: Could not set the %s TOF timing budget.\n", name);
    holdSensorInShutdown(shutPin);
    return false;
  }

  sensor.startContinuous(TOF_MEASUREMENT_PERIOD_MS);
  reading.initialized = true;
  Serial.printf("%s TOF ready at I2C address 0x%02X.\n", name, newAddress);
  return true;
}

void updateTofSensor(VL53L1X &sensor, TofReading &reading, uint32_t nowMs) {
  if (!reading.initialized) {
    return;
  }

  if (sensor.dataReady()) {
    const uint16_t distanceMm = sensor.read(false);
    reading.timedOut = sensor.timeoutOccurred();
    if (!reading.timedOut) {
      reading.distanceMm = distanceMm;
      reading.lastReadingMs = nowMs;
      reading.valid = true;
    } else {
      reading.valid = false;
    }
  }

  if (reading.valid && nowMs - reading.lastReadingMs > TOF_STALE_AFTER_MS) {
    reading.valid = false;
  }
}

bool tofIsNear(const TofReading &reading) {
  return reading.initialized && reading.valid &&
         reading.distanceMm <= TOF_TRIGGER_DISTANCE_MM;
}

// ------------------------------- RFID reader --------------------------------

HardwareSerial rfidUart(2);

const uint8_t SINGLE_POLL_COMMAND[] = {
  0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E
};

const size_t MAX_RFID_FRAME_LENGTH = 135;
const size_t MAX_EPC_LENGTH = 128;

uint8_t rfidFrame[MAX_RFID_FRAME_LENGTH];
size_t rfidFrameLength = 0;
size_t expectedRfidFrameLength = 0;

bool rfidHasResponded = false;
bool rfidHasSeenTag = false;
uint32_t lastRfidResponseMs = 0;
uint32_t lastTagSeenMs = 0;
uint32_t lastRfidPollMs = 0;
int8_t lastTagRssiDbm = 0;
uint8_t lastTagEpc[MAX_EPC_LENGTH];
size_t lastTagEpcLength = 0;

void resetRfidParser() {
  rfidFrameLength = 0;
  expectedRfidFrameLength = 0;
}

void processCompleteRfidFrame(uint32_t nowMs) {
  if (rfidFrameLength < 7 || rfidFrame[rfidFrameLength - 1] != 0x7E) {
    return;
  }

  uint8_t checksum = 0;
  for (size_t index = 1; index < rfidFrameLength - 2; ++index) {
    checksum += rfidFrame[index];
  }
  if (checksum != rfidFrame[rfidFrameLength - 2]) {
    return;
  }

  rfidHasResponded = true;
  lastRfidResponseMs = nowMs;

  const uint8_t type = rfidFrame[1];
  const uint8_t command = rfidFrame[2];
  const uint16_t payloadLength =
    ((uint16_t)rfidFrame[3] << 8) | rfidFrame[4];

  // Tag payload: RSSI (1), PC (2), EPC (variable), CRC (2).
  if (type == 0x02 && command == 0x22 && payloadLength >= 5) {
    size_t epcLength = payloadLength - 5;
    if (epcLength > MAX_EPC_LENGTH) {
      epcLength = MAX_EPC_LENGTH;
    }

    lastTagRssiDbm = (int8_t)rfidFrame[5];
    lastTagEpcLength = epcLength;
    for (size_t index = 0; index < epcLength; ++index) {
      lastTagEpc[index] = rfidFrame[8 + index];
    }
    rfidHasSeenTag = true;
    lastTagSeenMs = nowMs;
  }
}

void consumeRfidByte(uint8_t value, uint32_t nowMs) {
  if (rfidFrameLength == 0) {
    if (value != 0xBB) {
      return;
    }
    rfidFrame[rfidFrameLength++] = value;
    return;
  }

  if (rfidFrameLength >= MAX_RFID_FRAME_LENGTH) {
    resetRfidParser();
    return;
  }

  rfidFrame[rfidFrameLength++] = value;

  if (rfidFrameLength == 5) {
    const uint16_t payloadLength =
      ((uint16_t)rfidFrame[3] << 8) | rfidFrame[4];
    expectedRfidFrameLength = (size_t)payloadLength + 7;
    if (expectedRfidFrameLength > MAX_RFID_FRAME_LENGTH) {
      resetRfidParser();
      return;
    }
  }

  if (expectedRfidFrameLength > 0 &&
      rfidFrameLength == expectedRfidFrameLength) {
    processCompleteRfidFrame(nowMs);
    resetRfidParser();
  }
}

void updateRfid(uint32_t nowMs) {
  while (rfidUart.available() > 0) {
    consumeRfidByte((uint8_t)rfidUart.read(), nowMs);
  }

  if (nowMs - lastRfidPollMs >= RFID_POLL_PERIOD_MS) {
    lastRfidPollMs = nowMs;
    rfidUart.write(SINGLE_POLL_COMMAND, sizeof(SINGLE_POLL_COMMAND));
  }
}

bool rfidTagIsPresent(uint32_t nowMs) {
  return rfidHasSeenTag && nowMs - lastTagSeenMs <= RFID_PRESENCE_HOLD_MS;
}

// --------------------------------- Motors -----------------------------------

struct Motor {
  uint8_t in1Pin;
  uint8_t in2Pin;
  bool extendIsIn1High;
};

Motor outsideLeftMotor = {25, 26, OUTSIDE_LEFT_EXTEND_IS_IN1_HIGH};
Motor outsideRightMotor = {27, 14, OUTSIDE_RIGHT_EXTEND_IS_IN1_HIGH};
Motor insideLeftMotor = {13, 18, INSIDE_LEFT_EXTEND_IS_IN1_HIGH};
Motor insideRightMotor = {19, 23, INSIDE_RIGHT_EXTEND_IS_IN1_HIGH};

enum PairState {
  PAIR_EXTENDING,
  PAIR_EXTENDED,
  PAIR_RETRACTING,
  PAIR_RETRACTED_ACTIVE,
  PAIR_RETRACTED_CLEAR_WAIT,
  PAIR_REVERSAL_PAUSE
};

struct MotorPair {
  Motor *left;
  Motor *right;
  PairState state;
  uint32_t stateStartedMs;
  uint32_t clearStartedMs;
};

// Explicit declarations keep Arduino's automatic prototype generator from
// placing these functions before the custom motor types above.
void initializeMotor(const Motor &motor);
void stopMotor(const Motor &motor);
void driveMotor(const Motor &motor, bool extend);
void stopPair(MotorPair &pair);
void startExtending(MotorPair &pair, uint32_t nowMs);
void startRetracting(MotorPair &pair, uint32_t nowMs);
void updateMotorPair(MotorPair &pair, bool retractCondition, uint32_t nowMs);
const char *pairStateName(PairState state);
uint32_t clearWaitRemainingMs(const MotorPair &pair, uint32_t nowMs);
void printMotorPair(const char *location, const MotorPair &pair,
                    uint32_t nowMs);

MotorPair outsidePair = {
  &outsideLeftMotor, &outsideRightMotor, PAIR_EXTENDING, 0, 0
};
MotorPair insidePair = {
  &insideLeftMotor, &insideRightMotor, PAIR_EXTENDING, 0, 0
};

void initializeMotor(const Motor &motor) {
  pinMode(motor.in1Pin, OUTPUT);
  pinMode(motor.in2Pin, OUTPUT);
  digitalWrite(motor.in1Pin, LOW);
  digitalWrite(motor.in2Pin, LOW);
}

void stopMotor(const Motor &motor) {
  digitalWrite(motor.in1Pin, LOW);
  digitalWrite(motor.in2Pin, LOW);
}

void driveMotor(const Motor &motor, bool extend) {
  const bool in1High = extend == motor.extendIsIn1High;
  digitalWrite(motor.in1Pin, in1High ? HIGH : LOW);
  digitalWrite(motor.in2Pin, in1High ? LOW : HIGH);
}

void stopPair(MotorPair &pair) {
  stopMotor(*pair.left);
  stopMotor(*pair.right);
}

void startExtending(MotorPair &pair, uint32_t nowMs) {
  driveMotor(*pair.left, true);
  driveMotor(*pair.right, true);
  pair.state = PAIR_EXTENDING;
  pair.stateStartedMs = nowMs;
}

void startRetracting(MotorPair &pair, uint32_t nowMs) {
  driveMotor(*pair.left, false);
  driveMotor(*pair.right, false);
  pair.state = PAIR_RETRACTING;
  pair.stateStartedMs = nowMs;
}

void updateMotorPair(MotorPair &pair, bool retractCondition,
                     uint32_t nowMs) {
  switch (pair.state) {
    case PAIR_EXTENDING:
      if (retractCondition) {
        stopPair(pair);
        pair.state = PAIR_REVERSAL_PAUSE;
        pair.stateStartedMs = nowMs;
      } else if (nowMs - pair.stateStartedMs >= ACTUATOR_TRAVEL_TIME_MS) {
        stopPair(pair);
        pair.state = PAIR_EXTENDED;
        pair.stateStartedMs = nowMs;
      }
      break;

    case PAIR_EXTENDED:
      if (retractCondition) {
        startRetracting(pair, nowMs);
      }
      break;

    case PAIR_RETRACTING:
      if (nowMs - pair.stateStartedMs >= ACTUATOR_TRAVEL_TIME_MS) {
        stopPair(pair);
        pair.stateStartedMs = nowMs;
        if (retractCondition) {
          pair.state = PAIR_RETRACTED_ACTIVE;
        } else {
          pair.state = PAIR_RETRACTED_CLEAR_WAIT;
          pair.clearStartedMs = nowMs;
        }
      }
      break;

    case PAIR_RETRACTED_ACTIVE:
      if (!retractCondition) {
        pair.state = PAIR_RETRACTED_CLEAR_WAIT;
        pair.clearStartedMs = nowMs;
      }
      break;

    case PAIR_RETRACTED_CLEAR_WAIT:
      if (retractCondition) {
        pair.state = PAIR_RETRACTED_ACTIVE;
      } else if (nowMs - pair.clearStartedMs >= CONDITIONS_CLEAR_DELAY_MS) {
        startExtending(pair, nowMs);
      }
      break;

    case PAIR_REVERSAL_PAUSE:
      if (nowMs - pair.stateStartedMs >= MOTOR_DIRECTION_CHANGE_DELAY_MS) {
        if (retractCondition) {
          startRetracting(pair, nowMs);
        } else {
          // The condition vanished during the reversal pause. Finish locking.
          startExtending(pair, nowMs);
        }
      }
      break;
  }
}

const char *pairStateName(PairState state) {
  switch (state) {
    case PAIR_EXTENDING: return "EXTENDING (locking)";
    case PAIR_EXTENDED: return "EXTENDED (locked)";
    case PAIR_RETRACTING: return "RETRACTING (opening)";
    case PAIR_RETRACTED_ACTIVE: return "RETRACTED (condition active)";
    case PAIR_RETRACTED_CLEAR_WAIT: return "RETRACTED (10 s clear-wait)";
    case PAIR_REVERSAL_PAUSE: return "STOPPED (direction-change pause)";
  }
  return "UNKNOWN";
}

uint32_t clearWaitRemainingMs(const MotorPair &pair, uint32_t nowMs) {
  if (pair.state != PAIR_RETRACTED_CLEAR_WAIT) {
    return 0;
  }
  const uint32_t elapsedMs = nowMs - pair.clearStartedMs;
  return elapsedMs >= CONDITIONS_CLEAR_DELAY_MS
           ? 0
           : CONDITIONS_CLEAR_DELAY_MS - elapsedMs;
}

// ------------------------------ Serial report -------------------------------

void printTofReading(const char *name, const TofReading &reading) {
  Serial.printf("TOF %-7s: ", name);
  if (!reading.initialized) {
    Serial.println("NOT INITIALIZED");
  } else if (reading.timedOut) {
    Serial.println("TIMEOUT");
  } else if (!reading.valid) {
    Serial.println("NO CURRENT READING / STALE");
  } else {
    Serial.printf("%u mm (%.1f cm) | %s\n", reading.distanceMm,
                  reading.distanceMm / 10.0,
                  tofIsNear(reading) ? "WITHIN THRESHOLD" : "CLEAR");
  }
}

void printRfidReading(uint32_t nowMs, bool tagPresent) {
  Serial.print("RFID        : ");
  if (tagPresent) {
    Serial.print("TAG PRESENT | EPC=");
    for (size_t index = 0; index < lastTagEpcLength; ++index) {
      Serial.printf("%02X", lastTagEpc[index]);
    }
    Serial.printf(" | RSSI=%d dBm\n", lastTagRssiDbm);
  } else if (rfidHasResponded) {
    Serial.printf("NO TAG | last reader response %lu ms ago\n",
                  nowMs - lastRfidResponseMs);
  } else {
    Serial.println("NO UART RESPONSE YET");
  }
}

void printMotorPair(const char *location, const MotorPair &pair,
                    uint32_t nowMs) {
  Serial.printf("%s driver 1 (LEFT) : %s [commanded/assumed]\n",
                location, pairStateName(pair.state));
  Serial.printf("%s driver 2 (RIGHT): %s [commanded/assumed]\n",
                location, pairStateName(pair.state));
  if (pair.state == PAIR_RETRACTED_CLEAR_WAIT) {
    Serial.printf("%s clear-wait remaining: %.1f s\n", location,
                  clearWaitRemainingMs(pair, nowMs) / 1000.0);
  }
}

const char *overallDoorStage(bool outboundCondition, bool inboundCondition) {
  if (outboundCondition || inboundCondition) {
    return "CONDITIONS ACTIVE";
  }
  if (outsidePair.state == PAIR_RETRACTED_CLEAR_WAIT ||
      insidePair.state == PAIR_RETRACTED_CLEAR_WAIT) {
    return "CONDITIONS FALSE - 10 s CLEAR-WAIT ACTIVE";
  }
  if (outsidePair.state == PAIR_RETRACTING ||
      insidePair.state == PAIR_RETRACTING) {
    return "OPENING / RETRACTING";
  }
  if (outsidePair.state == PAIR_RETRACTED_ACTIVE ||
      insidePair.state == PAIR_RETRACTED_ACTIVE) {
    return "MOTORS RETRACTED - CONDITIONS NOW FALSE";
  }
  if (outsidePair.state == PAIR_EXTENDING ||
      insidePair.state == PAIR_EXTENDING) {
    return "CLOSING / EXTENDING";
  }
  if (outsidePair.state == PAIR_REVERSAL_PAUSE ||
      insidePair.state == PAIR_REVERSAL_PAUSE) {
    return "MOTOR DIRECTION-CHANGE PAUSE";
  }
  return "LOCKED / IDLE - CONDITIONS FALSE";
}

void printStatus(uint32_t nowMs, bool tagPresent, bool insideNear,
                 bool outsideNear, bool outboundCondition,
                 bool inboundCondition) {
  Serial.println();
  Serial.printf("================ SMART DOOR @ %lu ms ================\n", nowMs);
  printRfidReading(nowMs, tagPresent);
  printTofReading("INSIDE", insideTof);
  printTofReading("OUTSIDE", outsideTof);
  Serial.printf("Inputs      : RFID=%s | INSIDE_NEAR=%s | OUTSIDE_NEAR=%s\n",
                tagPresent ? "YES" : "NO", insideNear ? "YES" : "NO",
                outsideNear ? "YES" : "NO");
  Serial.printf("OUTBOUND condition (opens OUTSIDE motors): %s\n",
                outboundCondition ? "TRUE" : "FALSE");
  Serial.printf("INBOUND condition  (opens INSIDE motors) : %s\n",
                inboundCondition ? "TRUE" : "FALSE");
  Serial.printf("DOOR STAGE: %s\n",
                overallDoorStage(outboundCondition, inboundCondition));
  printMotorPair("OUTSIDE", outsidePair, nowMs);
  printMotorPair("INSIDE ", insidePair, nowMs);
}

// -------------------------------- Arduino -----------------------------------

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("Integrated smart cat-door controller starting");

  initializeMotor(outsideLeftMotor);
  initializeMotor(outsideRightMotor);
  initializeMotor(insideLeftMotor);
  initializeMotor(insideRightMotor);

  holdSensorInShutdown(OUTSIDE_SHUT_PIN);
  holdSensorInShutdown(INSIDE_SHUT_PIN);
  delay(20);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // Stable setting for the current TOF wiring.

  initializeTofSensor(outsideSensor, outsideTof, OUTSIDE_SHUT_PIN,
                      OUTSIDE_I2C_ADDRESS, "OUTSIDE");
  initializeTofSensor(insideSensor, insideTof, INSIDE_SHUT_PIN,
                      INSIDE_I2C_ADDRESS, "INSIDE");

  rfidUart.begin(RFID_UART_BAUD, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  Serial.printf("RFID UART ready: RX GPIO %u, TX GPIO %u, %lu baud.\n",
                RFID_RX_PIN, RFID_TX_PIN, RFID_UART_BAUD);

  // Establish a known locked state at startup. The actuators stop after the
  // configured travel time; there are no position switches in this design.
  const uint32_t nowMs = millis();
  startExtending(outsidePair, nowMs);
  startExtending(insidePair, nowMs);
  Serial.printf("Startup: extending all four actuators for %lu ms.\n",
                ACTUATOR_TRAVEL_TIME_MS);
}

void loop() {
  const uint32_t nowMs = millis();

  updateRfid(nowMs);
  updateTofSensor(outsideSensor, outsideTof, nowMs);
  updateTofSensor(insideSensor, insideTof, nowMs);

  const bool tagPresent = rfidTagIsPresent(nowMs);
  const bool insideNear = tofIsNear(insideTof);
  const bool outsideNear = tofIsNear(outsideTof);

  // A cat on the inside opens the outside locks to permit outward travel.
  const bool outboundCondition = tagPresent && insideNear;
  // A cat on the outside opens the inside locks to permit inward travel.
  const bool inboundCondition = tagPresent && outsideNear;

  updateMotorPair(outsidePair, outboundCondition, nowMs);
  updateMotorPair(insidePair, inboundCondition, nowMs);

  static uint32_t lastReportMs = 0;
  if (nowMs - lastReportMs >= SERIAL_REPORT_PERIOD_MS) {
    lastReportMs = nowMs;
    printStatus(nowMs, tagPresent, insideNear, outsideNear,
                outboundCondition, inboundCondition);
  }

  delay(1);
}
