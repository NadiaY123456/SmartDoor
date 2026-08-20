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
// Open Serial Monitor at 115200 baud. After entering the Wi-Fi settings below,
// open http://smartdoor.local/ (or the printed IP address) on the same network.

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <time.h>

// --------------------------- Adjustable settings ---------------------------

const uint16_t TOF_TRIGGER_DISTANCE_MM = 250;  // 250 mm = 25 cm.
const uint32_t CONDITIONS_CLEAR_DELAY_MS = 20000;
const uint32_t ACTUATOR_TRAVEL_TIME_MS = 1500;

// Copy arduino_secrets.h.example to arduino_secrets.h and enter the 2.4 GHz
// Wi-Fi credentials there. That local file is ignored by Git.
#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
const char *WIFI_SSID = SMARTDOOR_WIFI_SSID;
const char *WIFI_PASSWORD = SMARTDOOR_WIFI_PASSWORD;
#else
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
#endif

// POSIX time-zone rule. This default is US Central Time with daylight saving.
// Change it if the door is installed in another time zone.
const char *TIME_ZONE = "CST6CDT,M3.2.0,M11.1.0";
const char *NTP_SERVER_1 = "pool.ntp.org";
const char *NTP_SERVER_2 = "time.nist.gov";

// A valid tag remains "present" briefly between reader polls. Increase this
// if the reader occasionally misses one poll while the tag is stationary.
const uint32_t RFID_PRESENCE_HOLD_MS = 1000;

const uint32_t SERIAL_REPORT_PERIOD_MS = 500;
const uint32_t TOF_MEASUREMENT_PERIOD_MS = 100;
const uint32_t TOF_STALE_AFTER_MS = 600;
const uint32_t RFID_POLL_PERIOD_MS = 250;
const uint32_t MOTOR_DIRECTION_CHANGE_DELAY_MS = 100;
const uint32_t WIFI_RETRY_PERIOD_MS = 10000;
const uint32_t SCHEDULE_CHECK_PERIOD_MS = 1000;

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

enum OperationMode {
  MODE_AUTOMATIC,
  MODE_FORCE_OPEN,
  MODE_FORCE_LOCKED
};

OperationMode activeMode = MODE_AUTOMATIC;
OperationMode pendingMode = MODE_AUTOMATIC;
bool modeChangePending = false;

bool scheduleEnabled = false;
uint16_t scheduledOpenMinute = 8 * 60;
uint16_t scheduledLockMinute = 22 * 60;
int8_t lastScheduleTarget = -1;

WebServer webServer(80);
Preferences preferences;

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
void updateMotorPair(MotorPair &pair, bool retractCondition,
                     bool immediateExtend, uint32_t nowMs);
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
                     bool immediateExtend, uint32_t nowMs) {
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
      if (immediateExtend) {
        stopPair(pair);
        pair.state = PAIR_REVERSAL_PAUSE;
        pair.stateStartedMs = nowMs;
      } else if (nowMs - pair.stateStartedMs >= ACTUATOR_TRAVEL_TIME_MS) {
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
      if (immediateExtend) {
        startExtending(pair, nowMs);
      } else if (!retractCondition) {
        pair.state = PAIR_RETRACTED_CLEAR_WAIT;
        pair.clearStartedMs = nowMs;
      }
      break;

    case PAIR_RETRACTED_CLEAR_WAIT:
      if (immediateExtend) {
        startExtending(pair, nowMs);
      } else if (retractCondition) {
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
    case PAIR_RETRACTED_CLEAR_WAIT: return "RETRACTED (20 s clear-wait)";
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
    return "CONDITIONS FALSE - 20 s CLEAR-WAIT ACTIVE";
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
                 bool outsideNear, bool automaticOutboundCondition,
                 bool automaticInboundCondition, bool outsideRetractDemand,
                 bool insideRetractDemand) {
  Serial.println();
  Serial.printf("================ SMART DOOR @ %lu ms ================\n", nowMs);
  printRfidReading(nowMs, tagPresent);
  printTofReading("INSIDE", insideTof);
  printTofReading("OUTSIDE", outsideTof);
  Serial.printf("Inputs      : RFID=%s | INSIDE_NEAR=%s | OUTSIDE_NEAR=%s\n",
                tagPresent ? "YES" : "NO", insideNear ? "YES" : "NO",
                outsideNear ? "YES" : "NO");
  Serial.printf("OUTBOUND condition (opens OUTSIDE motors): %s\n",
                automaticOutboundCondition ? "TRUE" : "FALSE");
  Serial.printf("INBOUND condition  (opens INSIDE motors) : %s\n",
                automaticInboundCondition ? "TRUE" : "FALSE");
  Serial.printf("CONTROL MODE: %s", operationModeName(activeMode));
  if (modeChangePending) {
    Serial.printf(" | PENDING: %s", operationModeName(pendingMode));
  }
  Serial.printf(" | forced-change safety=%s\n",
                forcedChangeIsSafe(nowMs) ? "CLEAR" : "BLOCKED");
  Serial.printf("DOOR STAGE: %s\n",
                overallDoorStage(outsideRetractDemand, insideRetractDemand));
  printMotorPair("OUTSIDE", outsidePair, nowMs);
  printMotorPair("INSIDE ", insidePair, nowMs);
}

// -------------------------- Mode and safety control -------------------------

const char *operationModeName(OperationMode mode) {
  switch (mode) {
    case MODE_AUTOMATIC: return "AUTOMATIC";
    case MODE_FORCE_OPEN: return "FORCE OPEN";
    case MODE_FORCE_LOCKED: return "FORCE LOCKED";
  }
  return "UNKNOWN";
}

bool forcedChangeIsSafe(uint32_t nowMs) {
  // Invalid or stale TOF data is deliberately treated as unsafe.
  return !rfidTagIsPresent(nowMs) && insideTof.initialized && insideTof.valid &&
         outsideTof.initialized && outsideTof.valid && !tofIsNear(insideTof) &&
         !tofIsNear(outsideTof);
}

void requestOperationMode(OperationMode requestedMode, const char *source,
                          bool safetyClear) {
  preferences.putUChar("savedMode", (uint8_t)requestedMode);

  if (requestedMode == MODE_AUTOMATIC) {
    activeMode = MODE_AUTOMATIC;
    pendingMode = MODE_AUTOMATIC;
    modeChangePending = false;
    Serial.printf("MODE: AUTOMATIC activated by %s.\n", source);
    return;
  }

  if (requestedMode == activeMode) {
    pendingMode = requestedMode;
    modeChangePending = false;
    Serial.printf("MODE: %s was already active when requested by %s.\n",
                  operationModeName(requestedMode), source);
    return;
  }

  if (safetyClear) {
    activeMode = requestedMode;
    pendingMode = requestedMode;
    modeChangePending = false;
    Serial.printf("MODE: %s activated by %s; safety area is clear.\n",
                  operationModeName(requestedMode), source);
  } else {
    pendingMode = requestedMode;
    modeChangePending = true;
    Serial.printf("MODE: %s requested by %s but is PENDING until RFID is clear "
                  "and both valid TOF readings exceed %u mm.\n",
                  operationModeName(requestedMode), source,
                  TOF_TRIGGER_DISTANCE_MM);
  }
}

void applyPendingModeWhenSafe(uint32_t nowMs) {
  if (modeChangePending && forcedChangeIsSafe(nowMs)) {
    activeMode = pendingMode;
    modeChangePending = false;
    Serial.printf("MODE: pending %s is now active; safety area became clear.\n",
                  operationModeName(activeMode));
  }
}

// --------------------------- Daily schedule logic ---------------------------

bool parseClockTime(const String &text, uint16_t &minuteOfDay) {
  if (text.length() != 5 || text.charAt(2) != ':') {
    return false;
  }
  const int hour = text.substring(0, 2).toInt();
  const int minute = text.substring(3, 5).toInt();
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59) {
    return false;
  }
  minuteOfDay = hour * 60 + minute;
  return true;
}

String formatClockTime(uint16_t minuteOfDay) {
  char value[6];
  snprintf(value, sizeof(value), "%02u:%02u", minuteOfDay / 60,
           minuteOfDay % 60);
  return String(value);
}

OperationMode scheduledModeAt(uint16_t minuteOfDay) {
  if (scheduledOpenMinute < scheduledLockMinute) {
    return minuteOfDay >= scheduledOpenMinute &&
                   minuteOfDay < scheduledLockMinute
             ? MODE_FORCE_OPEN
             : MODE_FORCE_LOCKED;
  }

  // An opening time later than the locking time represents an overnight
  // open interval, such as 20:00 through 06:00.
  return minuteOfDay >= scheduledOpenMinute ||
                 minuteOfDay < scheduledLockMinute
           ? MODE_FORCE_OPEN
           : MODE_FORCE_LOCKED;
}

bool getDoorLocalTime(struct tm &timeInfo) {
  return getLocalTime(&timeInfo, 10);
}

void updateDailySchedule(uint32_t nowMs) {
  static uint32_t lastCheckMs = 0;
  if (!scheduleEnabled || nowMs - lastCheckMs < SCHEDULE_CHECK_PERIOD_MS) {
    return;
  }
  lastCheckMs = nowMs;

  struct tm timeInfo;
  if (!getDoorLocalTime(timeInfo)) {
    return;
  }

  const uint16_t minuteOfDay = timeInfo.tm_hour * 60 + timeInfo.tm_min;
  const OperationMode target = scheduledModeAt(minuteOfDay);
  if ((int8_t)target != lastScheduleTarget) {
    lastScheduleTarget = (int8_t)target;
    requestOperationMode(target, "daily schedule", forcedChangeIsSafe(nowMs));
  }
}

// ------------------------------- Web control --------------------------------

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Smart Cat Door</title>
  <style>
    :root{color-scheme:dark;--bg:#10151b;--card:#1a222c;--line:#314050;--text:#edf3f7;--muted:#9cb0c0;--green:#37c978;--red:#ff6670;--amber:#f4bd50;--blue:#55a8ff}
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:16px system-ui,sans-serif}main{width:min(980px,94vw);margin:28px auto 60px}h1{font-size:clamp(1.8rem,5vw,2.7rem);margin:0 0 5px}h2{font-size:1.05rem;margin:0 0 14px}.subtitle,.note{color:var(--muted)}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px;margin-top:18px}.card{background:var(--card);border:1px solid var(--line);border-radius:15px;padding:18px}.wide{grid-column:1/-1}.value{font-size:1.25rem;font-weight:750}.row{display:flex;justify-content:space-between;gap:14px;padding:7px 0;border-bottom:1px solid #26313d}.row:last-child{border:0}.ok{color:var(--green)}.warn{color:var(--amber)}.bad{color:var(--red)}button{border:0;border-radius:10px;padding:12px 15px;margin:4px 5px 4px 0;font-weight:750;cursor:pointer;color:#071017}.auto{background:var(--blue)}.open{background:var(--green)}.lock{background:var(--red)}.save{background:var(--amber)}input[type=time]{background:#111820;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:9px;font:inherit;margin:5px 12px 5px 5px}.pending{border-color:var(--amber)}#message{min-height:24px;color:var(--amber);margin-top:10px}.small{font-size:.88rem}.pill{display:inline-block;border:1px solid var(--line);border-radius:999px;padding:5px 10px;margin:3px 4px 3px 0}
  </style>
</head>
<body><main>
  <h1>Smart Cat Door</h1>
  <div class="subtitle">Local ESP32 control panel</div>
  <div class="grid">
    <section class="card wide" id="modeCard">
      <h2>Door control</h2>
      <div class="row"><span>Active mode</span><span class="value" id="activeMode">—</span></div>
      <div class="row"><span>Requested mode</span><span id="pendingMode">None</span></div>
      <div class="row"><span>Door stage</span><span id="stage">—</span></div>
      <div style="margin-top:14px">
        <button class="auto" onclick="setMode('auto')">Automatic</button>
        <button class="open" onclick="setMode('open')">Force Open</button>
        <button class="lock" onclick="setMode('locked')">Force Locked</button>
      </div>
      <div id="message"></div>
      <div class="note small">Force Open and Force Locked remain pending until no RFID tag is present and both valid TOF readings are beyond 25 cm.</div>
    </section>
    <section class="card">
      <h2>Sensors</h2>
      <div class="row"><span>RFID</span><span id="rfid">—</span></div>
      <div class="row"><span>Inside TOF</span><span id="insideTof">—</span></div>
      <div class="row"><span>Outside TOF</span><span id="outsideTof">—</span></div>
      <div class="row"><span>Forced-mode safety</span><span id="safe">—</span></div>
    </section>
    <section class="card">
      <h2>Motor pairs</h2>
      <div class="row"><span>Outside driver 1 · left</span><span id="outsideLeft">—</span></div>
      <div class="row"><span>Outside driver 2 · right</span><span id="outsideRight">—</span></div>
      <div class="row"><span>Inside driver 1 · left</span><span id="insideLeft">—</span></div>
      <div class="row"><span>Inside driver 2 · right</span><span id="insideRight">—</span></div>
      <div class="note small" style="margin-top:10px">Positions are commanded/assumed because there are no limit switches.</div>
    </section>
    <section class="card wide">
      <h2>Daily forced-mode schedule</h2>
      <label><input id="scheduleEnabled" type="checkbox"> Enable every day</label><br><br>
      <label>Force open at <input id="openTime" type="time" value="08:00"></label>
      <label>Force locked at <input id="lockTime" type="time" value="22:00"></label>
      <button class="save" onclick="saveSchedule()">Save schedule</button>
      <div class="note small" style="margin-top:10px">The scheduled change is also held pending by the safety interlock. An overnight interval is supported.</div>
      <div class="row" style="margin-top:12px"><span>ESP32 local time</span><span id="localTime">Waiting for NTP</span></div>
    </section>
  </div>
</main>
<script>
const $=id=>document.getElementById(id);
function showMessage(text){$('message').textContent=text;setTimeout(()=>{$('message').textContent=''},5000)}
async function setMode(value){
  try{const r=await fetch('/api/mode',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'value='+encodeURIComponent(value)});const j=await r.json();showMessage(j.message);refresh()}catch(e){showMessage('Could not contact the door controller.')}
}
async function saveSchedule(){
  const body=new URLSearchParams({enabled:$('scheduleEnabled').checked?'1':'0',open:$('openTime').value,locked:$('lockTime').value});
  try{const r=await fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();showMessage(j.message);refresh()}catch(e){showMessage('Could not save the schedule.')}
}
function tofText(sensor){return sensor.valid?`${sensor.mm} mm (${(sensor.mm/10).toFixed(1)} cm)${sensor.near?' · DETECTED':' · clear'}`:sensor.initialized?'Unavailable / stale':'Not initialized'}
async function refresh(){
  try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();
    $('activeMode').textContent=s.activeMode;$('pendingMode').textContent=s.pending?s.pendingMode+' (waiting for clear area)':'None';$('modeCard').classList.toggle('pending',s.pending);$('stage').textContent=s.stage;
    $('rfid').textContent=s.rfid.present?'TAG PRESENT'+(s.rfid.epc?' · '+s.rfid.epc:''):'No tag';$('insideTof').textContent=tofText(s.insideTof);$('outsideTof').textContent=tofText(s.outsideTof);$('safe').textContent=s.safeForForcedChange?'CLEAR':'BLOCKED';$('safe').className=s.safeForForcedChange?'ok':'bad';
    $('outsideLeft').textContent=s.outsideMotors;$('outsideRight').textContent=s.outsideMotors;$('insideLeft').textContent=s.insideMotors;$('insideRight').textContent=s.insideMotors;
    if(document.activeElement.tagName!=='INPUT'){$('scheduleEnabled').checked=s.schedule.enabled;$('openTime').value=s.schedule.open;$('lockTime').value=s.schedule.locked}$('localTime').textContent=s.localTime;
  }catch(e){$('stage').textContent='Controller offline'}
}
refresh();setInterval(refresh,1000);
</script></body></html>
)HTML";

String jsonBool(bool value) {
  return value ? "true" : "false";
}

String currentEpcText() {
  String epc;
  epc.reserve(lastTagEpcLength * 2);
  char byteText[3];
  for (size_t index = 0; index < lastTagEpcLength; ++index) {
    snprintf(byteText, sizeof(byteText), "%02X", lastTagEpc[index]);
    epc += byteText;
  }
  return epc;
}

String currentLocalTimeText() {
  struct tm timeInfo;
  if (!getDoorLocalTime(timeInfo)) {
    return "Waiting for network time";
  }
  char text[32];
  strftime(text, sizeof(text), "%a %Y-%m-%d %H:%M:%S", &timeInfo);
  return String(text);
}

String tofJson(const TofReading &reading) {
  String json = "{\"initialized\":" + jsonBool(reading.initialized);
  json += ",\"valid\":" + jsonBool(reading.valid);
  json += ",\"near\":" + jsonBool(tofIsNear(reading));
  json += ",\"mm\":" + String(reading.distanceMm) + "}";
  return json;
}

void handleStatusRequest() {
  const uint32_t nowMs = millis();
  const bool tagPresent = rfidTagIsPresent(nowMs);
  const bool insideNear = tofIsNear(insideTof);
  const bool outsideNear = tofIsNear(outsideTof);
  const bool automaticOutbound = tagPresent && insideNear;
  const bool automaticInbound = tagPresent && outsideNear;
  const bool outsideRetract = activeMode == MODE_FORCE_OPEN ||
                              (activeMode == MODE_AUTOMATIC && automaticOutbound);
  const bool insideRetract = activeMode == MODE_FORCE_OPEN ||
                             (activeMode == MODE_AUTOMATIC && automaticInbound);

  String json;
  json.reserve(1100);
  json = "{\"activeMode\":\"" + String(operationModeName(activeMode)) + "\"";
  json += ",\"pending\":" + jsonBool(modeChangePending);
  json += ",\"pendingMode\":\"" + String(operationModeName(pendingMode)) + "\"";
  json += ",\"stage\":\"" +
          String(overallDoorStage(outsideRetract, insideRetract)) + "\"";
  json += ",\"safeForForcedChange\":" + jsonBool(forcedChangeIsSafe(nowMs));
  json += ",\"rfid\":{\"present\":" + jsonBool(tagPresent) +
          ",\"epc\":\"" + currentEpcText() + "\"}";
  json += ",\"insideTof\":" + tofJson(insideTof);
  json += ",\"outsideTof\":" + tofJson(outsideTof);
  json += ",\"outsideMotors\":\"" + String(pairStateName(outsidePair.state)) + "\"";
  json += ",\"insideMotors\":\"" + String(pairStateName(insidePair.state)) + "\"";
  json += ",\"schedule\":{\"enabled\":" + jsonBool(scheduleEnabled) +
          ",\"open\":\"" + formatClockTime(scheduledOpenMinute) +
          "\",\"locked\":\"" + formatClockTime(scheduledLockMinute) + "\"}";
  json += ",\"localTime\":\"" + currentLocalTimeText() + "\"}";
  webServer.send(200, "application/json", json);
}

void handleModeRequest() {
  if (!webServer.hasArg("value")) {
    webServer.send(400, "application/json", "{\"message\":\"Missing mode.\"}");
    return;
  }

  const String value = webServer.arg("value");
  OperationMode requestedMode;
  if (value == "auto") {
    requestedMode = MODE_AUTOMATIC;
  } else if (value == "open") {
    requestedMode = MODE_FORCE_OPEN;
  } else if (value == "locked") {
    requestedMode = MODE_FORCE_LOCKED;
  } else {
    webServer.send(400, "application/json", "{\"message\":\"Invalid mode.\"}");
    return;
  }

  const bool safetyClear = forcedChangeIsSafe(millis());
  requestOperationMode(requestedMode, "web page", safetyClear);
  const String message = modeChangePending
                           ? String(operationModeName(requestedMode)) +
                               " is pending until RFID and both TOFs are clear."
                           : String(operationModeName(requestedMode)) + " is active.";
  webServer.send(200, "application/json",
                 "{\"message\":\"" + message + "\"}");
}

void handleScheduleRequest() {
  if (!webServer.hasArg("enabled") || !webServer.hasArg("open") ||
      !webServer.hasArg("locked")) {
    webServer.send(400, "application/json",
                   "{\"message\":\"Schedule fields are missing.\"}");
    return;
  }

  uint16_t openMinute;
  uint16_t lockMinute;
  if (!parseClockTime(webServer.arg("open"), openMinute) ||
      !parseClockTime(webServer.arg("locked"), lockMinute) ||
      openMinute == lockMinute) {
    webServer.send(400, "application/json",
                   "{\"message\":\"Choose two different valid times.\"}");
    return;
  }

  scheduleEnabled = webServer.arg("enabled") == "1";
  scheduledOpenMinute = openMinute;
  scheduledLockMinute = lockMinute;
  lastScheduleTarget = -1;
  preferences.putBool("schedOn", scheduleEnabled);
  preferences.putUShort("openMin", scheduledOpenMinute);
  preferences.putUShort("lockMin", scheduledLockMinute);

  webServer.send(200, "application/json",
                 scheduleEnabled
                   ? "{\"message\":\"Daily schedule saved and enabled.\"}"
                   : "{\"message\":\"Schedule saved but disabled.\"}");
}

void startWebServer() {
  webServer.on("/", HTTP_GET, []() {
    webServer.send_P(200, "text/html", CONTROL_PAGE);
  });
  webServer.on("/api/status", HTTP_GET, handleStatusRequest);
  webServer.on("/api/mode", HTTP_POST, handleModeRequest);
  webServer.on("/api/schedule", HTTP_POST, handleScheduleRequest);
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "Not found");
  });
  webServer.begin();
}

void updateWifi(uint32_t nowMs) {
  static uint32_t lastAttemptMs = 0;
  static bool servicesStarted = false;

  if (WiFi.status() == WL_CONNECTED) {
    if (!servicesStarted) {
      servicesStarted = true;
      if (MDNS.begin("smartdoor")) {
        MDNS.addService("http", "tcp", 80);
      }
      configTzTime(TIME_ZONE, NTP_SERVER_1, NTP_SERVER_2);
      Serial.printf("WEB: connected to %s. Open http://smartdoor.local/ or http://%s/\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
    webServer.handleClient();
    return;
  }

  servicesStarted = false;
  if (nowMs - lastAttemptMs >= WIFI_RETRY_PERIOD_MS) {
    lastAttemptMs = nowMs;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("WEB: attempting Wi-Fi connection...");
  }
}

// -------------------------------- Arduino -----------------------------------

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("Integrated smart cat-door controller starting");

  preferences.begin("smartdoor", false);
  scheduleEnabled = preferences.getBool("schedOn", false);
  scheduledOpenMinute = preferences.getUShort("openMin", 8 * 60);
  scheduledLockMinute = preferences.getUShort("lockMin", 22 * 60);
  const uint8_t savedMode = preferences.getUChar("savedMode", MODE_AUTOMATIC);
  if (savedMode == MODE_FORCE_OPEN || savedMode == MODE_FORCE_LOCKED) {
    pendingMode = (OperationMode)savedMode;
    modeChangePending = true;
    Serial.printf("MODE: restored %s as pending until the safety area is clear.\n",
                  operationModeName(pendingMode));
  }

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

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  startWebServer();
  if (String(WIFI_SSID) == "YOUR_WIFI_NAME") {
    Serial.println("WEB: copy arduino_secrets.h.example to arduino_secrets.h, then enter Wi-Fi credentials.");
  } else {
    Serial.printf("WEB: connecting to Wi-Fi network %s...\n", WIFI_SSID);
  }

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
  updateWifi(nowMs);
  updateDailySchedule(nowMs);
  applyPendingModeWhenSafe(nowMs);

  const bool tagPresent = rfidTagIsPresent(nowMs);
  const bool insideNear = tofIsNear(insideTof);
  const bool outsideNear = tofIsNear(outsideTof);

  // A cat on the inside opens the outside locks to permit outward travel.
  const bool automaticOutboundCondition = tagPresent && insideNear;
  // A cat on the outside opens the inside locks to permit inward travel.
  const bool automaticInboundCondition = tagPresent && outsideNear;

  bool outsideRetractDemand = false;
  bool insideRetractDemand = false;
  bool immediateExtend = false;

  if (activeMode == MODE_FORCE_OPEN) {
    outsideRetractDemand = true;
    insideRetractDemand = true;
  } else if (activeMode == MODE_FORCE_LOCKED) {
    immediateExtend = true;
  } else {
    outsideRetractDemand = automaticOutboundCondition;
    insideRetractDemand = automaticInboundCondition;
  }

  updateMotorPair(outsidePair, outsideRetractDemand, immediateExtend, nowMs);
  updateMotorPair(insidePair, insideRetractDemand, immediateExtend, nowMs);

  static uint32_t lastReportMs = 0;
  if (nowMs - lastReportMs >= SERIAL_REPORT_PERIOD_MS) {
    lastReportMs = nowMs;
    printStatus(nowMs, tagPresent, insideNear, outsideNear,
                automaticOutboundCondition, automaticInboundCondition,
                outsideRetractDemand, insideRetractDemand);
  }

  delay(1);
}
