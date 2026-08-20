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
const uint32_t ACTUATOR_TRAVEL_TIME_MS = 3500;

// Copy arduino_secrets.h.example to arduino_secrets.h and enter the 2.4 GHz
// Wi-Fi credentials there. That local file is ignored by Git.
#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
const char *WIFI_SSID = SMARTDOOR_WIFI_SSID;
const char *WIFI_PASSWORD = SMARTDOOR_WIFI_PASSWORD;
#elif __has_include("arduino_secrets.h/arduino_secrets.h.ino")
// Support the credentials file currently stored in its Arduino-created folder.
#include "arduino_secrets.h/arduino_secrets.h.ino"
const char *WIFI_SSID = SMARTDOOR_WIFI_SSID;
const char *WIFI_PASSWORD = SMARTDOOR_WIFI_PASSWORD;
#else
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
#endif

// POSIX time-zone rule. Operating in PST / PDT, automatically changes
const char *TIME_ZONE = "PST8PDT,M3.2.0,M11.1.0";
const char *NTP_SERVER_1 = "pool.ntp.org";
const char *NTP_SERVER_2 = "time.nist.gov";

// A valid tag remains "present" briefly between reader polls. Increase this
// if the reader occasionally misses one poll while the tag is stationary.
const uint32_t RFID_PRESENCE_HOLD_MS = 1000;

const uint32_t SERIAL_REPORT_PERIOD_MS = 500;
const uint32_t TOF_MEASUREMENT_PERIOD_MS = 100;
const uint32_t TOF_STALE_AFTER_MS = 600;
const uint32_t RFID_RESPONSE_WINDOW_MS = 350;
const uint32_t RFID_POLL_INTERVAL_MS = 500;
const uint32_t MOTOR_DIRECTION_CHANGE_DELAY_MS = 100;
const uint32_t WIFI_RETRY_PERIOD_MS = 10000;
const uint32_t SCHEDULE_CHECK_PERIOD_MS = 1000;

// PWM slows the unloaded actuators without adding an audible low-frequency
// whine. Raise MOTOR_RUN_DUTY if a door later needs more torque.
const uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
const uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;
const uint8_t MOTOR_START_DUTY = 90;
const uint8_t MOTOR_RUN_DUTY = 140;
const uint32_t MOTOR_RAMP_TIME_MS = 300;

// Set a motor's value to false if that actuator moves opposite to the label
// printed by the program. Alternatively, swap that driver's OUT1/OUT2 wires.
const bool OUTSIDE_LEFT_EXTEND_IS_IN1_HIGH = false;
const bool OUTSIDE_RIGHT_EXTEND_IS_IN1_HIGH = false;
const bool INSIDE_LEFT_EXTEND_IS_IN1_HIGH = false;
const bool INSIDE_RIGHT_EXTEND_IS_IN1_HIGH = false;

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

enum DoorOpeningDirection : uint8_t {
  OPENING_OUTBOUND,
  OPENING_INBOUND,
  OPENING_BOTH_AUTOMATIC,
  OPENING_FORCED_BOTH
};

const uint32_t DOOR_LOG_MAGIC = 0x53444C47;  // "SDLG"
const size_t MAX_DOOR_LOG_EVENTS = 50;

struct DoorLogEvent {
  int64_t epochSeconds;
  uint32_t uptimeSeconds;
  uint8_t direction;
  uint8_t reserved[3];
};

struct DoorLogStorage {
  uint32_t magic;
  uint16_t count;
  uint16_t nextIndex;
  DoorLogEvent events[MAX_DOOR_LOG_EVENTS];
};

OperationMode activeMode = MODE_AUTOMATIC;
OperationMode pendingMode = MODE_AUTOMATIC;
bool modeChangePending = false;

bool lockedScheduleEnabled = false;
uint16_t lockedScheduleStartMinute = 22 * 60;
uint16_t lockedScheduleEndMinute = 8 * 60;
bool openScheduleEnabled = false;
uint16_t openScheduleStartMinute = 8 * 60;
uint16_t openScheduleEndMinute = 22 * 60;
int8_t lastScheduleTarget = -1;

DoorLogStorage doorLog = {DOOR_LOG_MAGIC, 0, 0, {}};

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

const size_t MAX_EPC_LENGTH = 128;

struct RfidFrame {
  uint8_t type;
  uint8_t command;
  uint16_t payloadLength;
  uint8_t payload[MAX_EPC_LENGTH];
  uint8_t checksum;
};

struct TagReading {
  bool valid;
  int8_t rssiDbm;
  size_t epcLength;
  uint8_t epc[MAX_EPC_LENGTH];
};

bool rfidHasResponded = false;
bool rfidHasSeenTag = false;
uint32_t lastRfidResponseMs = 0;
uint32_t lastTagSeenMs = 0;
uint32_t lastRfidPollMs = 0;
int8_t lastTagRssiDbm = 0;
uint8_t lastTagEpc[MAX_EPC_LENGTH];
size_t lastTagEpcLength = 0;

// Prevent Arduino's prototype generator from placing these declarations before
// the RFID frame types above.
bool readRfidByteBefore(uint8_t &value, uint32_t deadlineMs);
bool readRfidFrame(RfidFrame &frame, uint32_t timeoutMs);
bool frameContainsTag(const RfidFrame &frame, TagReading &tag);
void pollRfidOnce();
void updateRfid(uint32_t nowMs);

bool readRfidByteBefore(uint8_t &value, uint32_t deadlineMs) {
  while ((int32_t)(deadlineMs - millis()) > 0) {
    if (rfidUart.available() > 0) {
      value = (uint8_t)rfidUart.read();
      return true;
    }
    delay(1);
  }
  return false;
}

bool readRfidFrame(RfidFrame &frame, uint32_t timeoutMs) {
  const uint32_t deadlineMs = millis() + timeoutMs;
  uint8_t value = 0;

  do {
    if (!readRfidByteBefore(value, deadlineMs)) {
      return false;
    }
  } while (value != 0xBB);

  uint8_t lengthMsb = 0;
  uint8_t lengthLsb = 0;
  if (!readRfidByteBefore(frame.type, deadlineMs) ||
      !readRfidByteBefore(frame.command, deadlineMs) ||
      !readRfidByteBefore(lengthMsb, deadlineMs) ||
      !readRfidByteBefore(lengthLsb, deadlineMs)) {
    return false;
  }

  frame.payloadLength = ((uint16_t)lengthMsb << 8) | lengthLsb;
  if (frame.payloadLength > MAX_EPC_LENGTH) {
    return false;
  }

  uint8_t calculatedChecksum = frame.type + frame.command + lengthMsb + lengthLsb;
  for (size_t index = 0; index < frame.payloadLength; ++index) {
    if (!readRfidByteBefore(frame.payload[index], deadlineMs)) {
      return false;
    }
    calculatedChecksum += frame.payload[index];
  }

  uint8_t frameEnd = 0;
  if (!readRfidByteBefore(frame.checksum, deadlineMs) ||
      !readRfidByteBefore(frameEnd, deadlineMs)) {
    return false;
  }

  return frameEnd == 0x7E && frame.checksum == calculatedChecksum;
}

bool frameContainsTag(const RfidFrame &frame, TagReading &tag) {
  // Tag payload: RSSI (1), PC (2), EPC (variable), CRC (2).
  if (frame.type != 0x02 || frame.command != 0x22 ||
      frame.payloadLength < 5) {
    return false;
  }

  tag.valid = true;
  tag.rssiDbm = (int8_t)frame.payload[0];
  tag.epcLength = frame.payloadLength - 5;
  for (size_t index = 0; index < tag.epcLength; ++index) {
    tag.epc[index] = frame.payload[index + 3];
  }
  return true;
}

void pollRfidOnce() {
  // Match the standalone test: discard stale bytes before each new command.
  while (rfidUart.available() > 0) {
    rfidUart.read();
  }

  rfidUart.write(SINGLE_POLL_COMMAND, sizeof(SINGLE_POLL_COMMAND));
  rfidUart.flush();

  const uint32_t responseDeadlineMs = millis() + RFID_RESPONSE_WINDOW_MS;
  bool receivedValidFrame = false;
  TagReading firstTag = {false, 0, 0, {0}};

  while ((int32_t)(responseDeadlineMs - millis()) > 0) {
    RfidFrame frame;
    const uint32_t timeRemainingMs = responseDeadlineMs - millis();
    if (!readRfidFrame(frame, timeRemainingMs)) {
      break;
    }

    receivedValidFrame = true;
    if (!firstTag.valid) {
      frameContainsTag(frame, firstTag);
    }
  }

  const uint32_t nowMs = millis();
  if (receivedValidFrame) {
    rfidHasResponded = true;
    lastRfidResponseMs = nowMs;
  }
  if (firstTag.valid) {
    lastTagRssiDbm = firstTag.rssiDbm;
    lastTagEpcLength = firstTag.epcLength;
    for (size_t index = 0; index < firstTag.epcLength; ++index) {
      lastTagEpc[index] = firstTag.epc[index];
    }
    rfidHasSeenTag = true;
    lastTagSeenMs = nowMs;
  }
}

void updateRfid(uint32_t nowMs) {
  if (nowMs - lastRfidPollMs < RFID_POLL_INTERVAL_MS) {
    return;
  }
  pollRfidOnce();
  // Wait the full interval after the response window, like RfidUartTest.ino.
  lastRfidPollMs = millis();
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
  bool commandedExtend;
  bool openingClaimed;
};

// Explicit declarations keep Arduino's automatic prototype generator from
// placing these functions before the custom motor types above.
void initializeMotor(const Motor &motor);
void stopMotor(const Motor &motor);
void driveMotor(const Motor &motor, bool extend, uint8_t duty);
void drivePair(MotorPair &pair, uint8_t duty);
uint8_t motorDutyAt(const MotorPair &pair, uint32_t nowMs);
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
  &outsideRightMotor, &insideLeftMotor, PAIR_EXTENDING, 0, 0, true, false
};
MotorPair insidePair = {
  &outsideLeftMotor, &insideRightMotor, PAIR_EXTENDING, 0, 0, true, false
};

void initializeMotor(const Motor &motor) {
  ledcAttach(motor.in1Pin, MOTOR_PWM_FREQUENCY_HZ,
             MOTOR_PWM_RESOLUTION_BITS);
  ledcAttach(motor.in2Pin, MOTOR_PWM_FREQUENCY_HZ,
             MOTOR_PWM_RESOLUTION_BITS);
  ledcWrite(motor.in1Pin, 0);
  ledcWrite(motor.in2Pin, 0);
}

void stopMotor(const Motor &motor) {
  ledcWrite(motor.in1Pin, 0);
  ledcWrite(motor.in2Pin, 0);
}

void driveMotor(const Motor &motor, bool extend, uint8_t duty) {
  const bool in1High = extend == motor.extendIsIn1High;
  ledcWrite(motor.in1Pin, in1High ? duty : 0);
  ledcWrite(motor.in2Pin, in1High ? 0 : duty);
}

void drivePair(MotorPair &pair, uint8_t duty) {
  driveMotor(*pair.left, pair.commandedExtend, duty);
  driveMotor(*pair.right, pair.commandedExtend, duty);
}

uint8_t motorDutyAt(const MotorPair &pair, uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - pair.stateStartedMs;
  if (elapsedMs >= MOTOR_RAMP_TIME_MS) {
    return MOTOR_RUN_DUTY;
  }
  return MOTOR_START_DUTY +
         ((uint32_t)(MOTOR_RUN_DUTY - MOTOR_START_DUTY) * elapsedMs) /
             MOTOR_RAMP_TIME_MS;
}

void stopPair(MotorPair &pair) {
  stopMotor(*pair.left);
  stopMotor(*pair.right);
}

void startExtending(MotorPair &pair, uint32_t nowMs) {
  pair.commandedExtend = true;
  pair.openingClaimed = false;
  pair.state = PAIR_EXTENDING;
  pair.stateStartedMs = nowMs;
  drivePair(pair, MOTOR_START_DUTY);
}

void startRetracting(MotorPair &pair, uint32_t nowMs) {
  pair.commandedExtend = false;
  pair.openingClaimed = true;
  pair.state = PAIR_RETRACTING;
  pair.stateStartedMs = nowMs;
  drivePair(pair, MOTOR_START_DUTY);
}

void updateMotorPair(MotorPair &pair, bool retractCondition,
                     bool immediateExtend, uint32_t nowMs) {
  switch (pair.state) {
    case PAIR_EXTENDING:
      drivePair(pair, motorDutyAt(pair, nowMs));
      if (retractCondition) {
        pair.openingClaimed = true;
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
      drivePair(pair, motorDutyAt(pair, nowMs));
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

bool minuteIsInWindow(uint16_t minuteOfDay, uint16_t startMinute,
                      uint16_t endMinute) {
  if (startMinute < endMinute) {
    return minuteOfDay >= startMinute && minuteOfDay < endMinute;
  }
  // A start later than the end represents an overnight interval.
  return minuteOfDay >= startMinute || minuteOfDay < endMinute;
}

bool schedulesAreEnabled() {
  return lockedScheduleEnabled || openScheduleEnabled;
}

OperationMode scheduledModeAt(uint16_t minuteOfDay) {
  // Locked wins when two enabled windows overlap.
  if (lockedScheduleEnabled &&
      minuteIsInWindow(minuteOfDay, lockedScheduleStartMinute,
                       lockedScheduleEndMinute)) {
    return MODE_FORCE_LOCKED;
  }
  if (openScheduleEnabled &&
      minuteIsInWindow(minuteOfDay, openScheduleStartMinute,
                       openScheduleEndMinute)) {
    return MODE_FORCE_OPEN;
  }
  return MODE_AUTOMATIC;
}

bool getDoorLocalTime(struct tm &timeInfo) {
  return getLocalTime(&timeInfo, 10);
}

void updateDailySchedule(uint32_t nowMs) {
  static uint32_t lastCheckMs = 0;
  if (!schedulesAreEnabled()) {
    lastScheduleTarget = -1;
    return;
  }
  if (nowMs - lastCheckMs < SCHEDULE_CHECK_PERIOD_MS) {
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

// ----------------------------- Opening history ------------------------------

const char *doorDirectionName(DoorOpeningDirection direction) {
  switch (direction) {
    case OPENING_OUTBOUND: return "Cat going out";
    case OPENING_INBOUND: return "Cat coming in";
    case OPENING_BOTH_AUTOMATIC: return "Both directions detected";
    case OPENING_FORCED_BOTH: return "Forced open - both directions";
  }
  return "Unknown direction";
}

void loadDoorHistory() {
  if (preferences.getBytesLength("doorLog") != sizeof(doorLog)) {
    return;
  }

  DoorLogStorage storedLog;
  preferences.getBytes("doorLog", &storedLog, sizeof(storedLog));
  if (storedLog.magic != DOOR_LOG_MAGIC ||
      storedLog.count > MAX_DOOR_LOG_EVENTS ||
      storedLog.nextIndex >= MAX_DOOR_LOG_EVENTS) {
    Serial.println("LOG: stored opening history was invalid and was ignored.");
    return;
  }
  doorLog = storedLog;
  Serial.printf("LOG: restored %u door-opening event(s).\n", doorLog.count);
}

void recordDoorOpening(DoorOpeningDirection direction, uint32_t nowMs) {
  DoorLogEvent &event = doorLog.events[doorLog.nextIndex];
  const time_t currentTime = time(nullptr);
  event.epochSeconds = currentTime >= 1609459200 ? (int64_t)currentTime : 0;
  event.uptimeSeconds = nowMs / 1000;
  event.direction = (uint8_t)direction;
  memset(event.reserved, 0, sizeof(event.reserved));

  doorLog.nextIndex = (doorLog.nextIndex + 1) % MAX_DOOR_LOG_EVENTS;
  if (doorLog.count < MAX_DOOR_LOG_EVENTS) {
    ++doorLog.count;
  }
  preferences.putBytes("doorLog", &doorLog, sizeof(doorLog));

  Serial.printf("DOOR LOG: opened - %s", doorDirectionName(direction));
  if (event.epochSeconds == 0) {
    Serial.printf(" | clock not synchronized | uptime %lu s\n",
                  event.uptimeSeconds);
  } else {
    time_t eventTime = (time_t)event.epochSeconds;
    struct tm timeInfo;
    localtime_r(&eventTime, &timeInfo);
    char timeText[32];
    strftime(timeText, sizeof(timeText), "%Y-%m-%d %H:%M:%S", &timeInfo);
    Serial.printf(" | %s\n", timeText);
  }
}

String doorEventTimeText(const DoorLogEvent &event) {
  if (event.epochSeconds == 0) {
    return "Clock not synced (uptime " + String(event.uptimeSeconds) + " s)";
  }

  time_t eventTime = (time_t)event.epochSeconds;
  struct tm timeInfo;
  localtime_r(&eventTime, &timeInfo);
  char timeText[40];
  strftime(timeText, sizeof(timeText), "%a %Y-%m-%d %H:%M:%S", &timeInfo);
  return String(timeText);
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
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:16px system-ui,sans-serif}main{width:min(980px,94vw);margin:28px auto 60px}h1{font-size:clamp(1.8rem,5vw,2.7rem);margin:0 0 5px}h2{font-size:1.05rem;margin:0 0 14px}.subtitle,.note{color:var(--muted)}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px;margin-top:18px}.card{background:var(--card);border:1px solid var(--line);border-radius:15px;padding:18px}.wide{grid-column:1/-1}.value{font-size:1.25rem;font-weight:750}.row{display:flex;justify-content:space-between;gap:14px;padding:7px 0;border-bottom:1px solid #26313d}.row:last-child{border:0}.ok{color:var(--green)}.warn{color:var(--amber)}.bad{color:var(--red)}button{border:0;border-radius:10px;padding:12px 15px;margin:4px 5px 4px 0;font-weight:750;cursor:pointer;color:#071017}.auto{background:var(--blue)}.open{background:var(--green)}.lock{background:var(--red)}.save{background:var(--amber)}input[type=time]{background:#111820;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:9px;font:inherit;margin:5px 12px 5px 5px}.pending{border-color:var(--amber)}#message{min-height:24px;color:var(--amber);margin-top:10px}.small{font-size:.88rem}.event{display:grid;grid-template-columns:minmax(145px,1fr) minmax(130px,1fr);gap:16px;padding:10px 0;border-bottom:1px solid #26313d}.event:last-child{border:0}.eventTime{color:var(--muted)}
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
      <h2>Daily schedules</h2>
      <div class="row"><label><input id="lockScheduleEnabled" type="checkbox"> Force locked</label><span><input id="lockStart" type="time" value="22:00"> to <input id="lockEnd" type="time" value="08:00"></span></div>
      <div class="row"><label><input id="openScheduleEnabled" type="checkbox"> Force open</label><span><input id="openStart" type="time" value="08:00"> to <input id="openEnd" type="time" value="22:00"></span></div>
      <button class="save" onclick="saveSchedules()">Save schedules</button>
      <div class="note small" style="margin-top:10px">Outside enabled windows, the door runs in Automatic mode. Overnight windows are supported; locked takes priority if windows overlap.</div>
      <div class="row" style="margin-top:12px"><span>ESP32 local time</span><span id="localTime">Waiting for NTP</span></div>
    </section>
    <section class="card wide">
      <h2>Opening history</h2>
      <div class="note small" style="margin-bottom:8px">Most recent 50 completed openings, newest first.</div>
      <div id="history"><div class="note">No openings logged yet.</div></div>
    </section>
  </div>
</main>
<script>
const $=id=>document.getElementById(id);
function showMessage(text){$('message').textContent=text;setTimeout(()=>{$('message').textContent=''},5000)}
async function setMode(value){
  try{const r=await fetch('/api/mode',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'value='+encodeURIComponent(value)});const j=await r.json();showMessage(j.message);refresh()}catch(e){showMessage('Could not contact the door controller.')}
}
async function saveSchedules(){
  const body=new URLSearchParams({lockEnabled:$('lockScheduleEnabled').checked?'1':'0',lockStart:$('lockStart').value,lockEnd:$('lockEnd').value,openEnabled:$('openScheduleEnabled').checked?'1':'0',openStart:$('openStart').value,openEnd:$('openEnd').value});
  try{const r=await fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();showMessage(j.message);refresh()}catch(e){showMessage('Could not save the schedule.')}
}
function tofText(sensor){return sensor.valid?`${sensor.mm} mm (${(sensor.mm/10).toFixed(1)} cm)${sensor.near?' · DETECTED':' · clear'}`:sensor.initialized?'Unavailable / stale':'Not initialized'}
async function refresh(){
  try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();
    $('activeMode').textContent=s.activeMode;$('pendingMode').textContent=s.pending?s.pendingMode+' (waiting for clear area)':'None';$('modeCard').classList.toggle('pending',s.pending);$('stage').textContent=s.stage;
    $('rfid').textContent=s.rfid.present?'TAG PRESENT'+(s.rfid.epc?' · '+s.rfid.epc:''):'No tag';$('insideTof').textContent=tofText(s.insideTof);$('outsideTof').textContent=tofText(s.outsideTof);$('safe').textContent=s.safeForForcedChange?'CLEAR':'BLOCKED';$('safe').className=s.safeForForcedChange?'ok':'bad';
    $('outsideLeft').textContent=s.outsideMotors;$('outsideRight').textContent=s.outsideMotors;$('insideLeft').textContent=s.insideMotors;$('insideRight').textContent=s.insideMotors;
    if(document.activeElement.tagName!=='INPUT'){$('lockScheduleEnabled').checked=s.schedule.lockEnabled;$('lockStart').value=s.schedule.lockStart;$('lockEnd').value=s.schedule.lockEnd;$('openScheduleEnabled').checked=s.schedule.openEnabled;$('openStart').value=s.schedule.openStart;$('openEnd').value=s.schedule.openEnd}$('localTime').textContent=s.localTime;
  }catch(e){$('stage').textContent='Controller offline'}
}
async function refreshHistory(){
  try{const r=await fetch('/api/history',{cache:'no-store'});const data=await r.json();const box=$('history');box.replaceChildren();
    if(!data.events.length){const empty=document.createElement('div');empty.className='note';empty.textContent='No openings logged yet.';box.appendChild(empty);return}
    for(const event of data.events){const row=document.createElement('div');row.className='event';const time=document.createElement('span');time.className='eventTime';time.textContent=event.time;const direction=document.createElement('strong');direction.textContent=event.direction;row.append(time,direction);box.appendChild(row)}
  }catch(e){}
}
refresh();refreshHistory();setInterval(refresh,1000);setInterval(refreshHistory,5000);
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
  bool automaticOutbound = tagPresent && outsideNear;
  bool automaticInbound = tagPresent && insideNear;
  if (outsidePair.openingClaimed || automaticOutbound) {
    automaticInbound = false;
  } else if (insidePair.openingClaimed || automaticInbound) {
    automaticOutbound = false;
  }
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
  json += ",\"schedule\":{\"lockEnabled\":" +
          jsonBool(lockedScheduleEnabled) + ",\"lockStart\":\"" +
          formatClockTime(lockedScheduleStartMinute) + "\",\"lockEnd\":\"" +
          formatClockTime(lockedScheduleEndMinute) + "\",\"openEnabled\":" +
          jsonBool(openScheduleEnabled) + ",\"openStart\":\"" +
          formatClockTime(openScheduleStartMinute) + "\",\"openEnd\":\"" +
          formatClockTime(openScheduleEndMinute) + "\"}";
  json += ",\"localTime\":\"" + currentLocalTimeText() + "\"}";
  webServer.send(200, "application/json", json);
}

void handleHistoryRequest() {
  String json;
  json.reserve(6000);
  json = "{\"events\":[";
  for (uint16_t offset = 0; offset < doorLog.count; ++offset) {
    const size_t index =
      (doorLog.nextIndex + MAX_DOOR_LOG_EVENTS - 1 - offset) %
      MAX_DOOR_LOG_EVENTS;
    const DoorLogEvent &event = doorLog.events[index];
    if (offset > 0) {
      json += ',';
    }
    json += "{\"time\":\"" + doorEventTimeText(event) +
            "\",\"direction\":\"" +
            String(doorDirectionName((DoorOpeningDirection)event.direction)) +
            "\"}";
  }
  json += "]}";
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
  if (!webServer.hasArg("lockEnabled") || !webServer.hasArg("lockStart") ||
      !webServer.hasArg("lockEnd") || !webServer.hasArg("openEnabled") ||
      !webServer.hasArg("openStart") || !webServer.hasArg("openEnd")) {
    webServer.send(400, "application/json",
                   "{\"message\":\"Schedule fields are missing.\"}");
    return;
  }

  uint16_t lockStartMinute;
  uint16_t lockEndMinute;
  uint16_t openStartMinute;
  uint16_t openEndMinute;
  if (!parseClockTime(webServer.arg("lockStart"), lockStartMinute) ||
      !parseClockTime(webServer.arg("lockEnd"), lockEndMinute) ||
      !parseClockTime(webServer.arg("openStart"), openStartMinute) ||
      !parseClockTime(webServer.arg("openEnd"), openEndMinute)) {
    webServer.send(400, "application/json",
                   "{\"message\":\"Use valid start and end times.\"}");
    return;
  }

  const bool requestedLockEnabled = webServer.arg("lockEnabled") == "1";
  const bool requestedOpenEnabled = webServer.arg("openEnabled") == "1";
  if ((requestedLockEnabled && lockStartMinute == lockEndMinute) ||
      (requestedOpenEnabled && openStartMinute == openEndMinute)) {
    webServer.send(400, "application/json",
                   "{\"message\":\"An enabled window needs different start and end times.\"}");
    return;
  }

  lockedScheduleEnabled = requestedLockEnabled;
  lockedScheduleStartMinute = lockStartMinute;
  lockedScheduleEndMinute = lockEndMinute;
  openScheduleEnabled = requestedOpenEnabled;
  openScheduleStartMinute = openStartMinute;
  openScheduleEndMinute = openEndMinute;
  lastScheduleTarget = -1;
  preferences.putBool("lockOn", lockedScheduleEnabled);
  preferences.putUShort("lockStart", lockedScheduleStartMinute);
  preferences.putUShort("lockEnd", lockedScheduleEndMinute);
  preferences.putBool("openOn", openScheduleEnabled);
  preferences.putUShort("openStart", openScheduleStartMinute);
  preferences.putUShort("openEnd", openScheduleEndMinute);

  webServer.send(200, "application/json",
                 schedulesAreEnabled()
                   ? "{\"message\":\"Daily schedules saved.\"}"
                   : "{\"message\":\"Schedules saved; Automatic mode has no time rules.\"}");
}

void startWebServer() {
  webServer.on("/", HTTP_GET, []() {
    webServer.send_P(200, "text/html", CONTROL_PAGE);
  });
  webServer.on("/api/status", HTTP_GET, handleStatusRequest);
  webServer.on("/api/history", HTTP_GET, handleHistoryRequest);
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
        Serial.println("WEB: mDNS ready at http://smartdoor.local/");
      } else {
        Serial.println("WEB: mDNS failed; use the IP address below instead.");
      }
      configTzTime(TIME_ZONE, NTP_SERVER_1, NTP_SERVER_2);
      Serial.printf("WEB: connected to %s. Direct dashboard URL: http://%s/\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
    webServer.handleClient();
    return;
  }

  servicesStarted = false;
  if (nowMs - lastAttemptMs >= WIFI_RETRY_PERIOD_MS) {
    lastAttemptMs = nowMs;
    Serial.printf("WEB: Wi-Fi status %d; retrying connection...\n", WiFi.status());
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
  lockedScheduleEnabled = preferences.getBool("lockOn", false);
  lockedScheduleStartMinute = preferences.getUShort("lockStart", 22 * 60);
  lockedScheduleEndMinute = preferences.getUShort("lockEnd", 8 * 60);
  openScheduleEnabled = preferences.getBool("openOn", false);
  openScheduleStartMinute = preferences.getUShort("openStart", 8 * 60);
  openScheduleEndMinute = preferences.getUShort("openEnd", 22 * 60);
  const uint8_t savedMode = preferences.getUChar("savedMode", MODE_AUTOMATIC);
  loadDoorHistory();
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

  // The outside ToF sensor opens the outside motor pair. Only one side may
  // open automatically at a time; outside has priority if both detect first.
  bool automaticOutboundCondition = tagPresent && outsideNear;
  bool automaticInboundCondition = tagPresent && insideNear;
  if (outsidePair.openingClaimed || automaticOutboundCondition) {
    automaticInboundCondition = false;
  } else if (insidePair.openingClaimed || automaticInboundCondition) {
    automaticOutboundCondition = false;
  }

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

  const PairState previousOutsideState = outsidePair.state;
  const PairState previousInsideState = insidePair.state;
  updateMotorPair(outsidePair, outsideRetractDemand, immediateExtend, nowMs);
  updateMotorPair(insidePair, insideRetractDemand, immediateExtend, nowMs);

  const bool outsideFinishedOpening =
    previousOutsideState == PAIR_RETRACTING &&
    (outsidePair.state == PAIR_RETRACTED_ACTIVE ||
     outsidePair.state == PAIR_RETRACTED_CLEAR_WAIT);
  const bool insideFinishedOpening =
    previousInsideState == PAIR_RETRACTING &&
    (insidePair.state == PAIR_RETRACTED_ACTIVE ||
     insidePair.state == PAIR_RETRACTED_CLEAR_WAIT);

  static bool forcedOpeningLogged = false;
  if (activeMode == MODE_FORCE_OPEN) {
    if ((outsideFinishedOpening || insideFinishedOpening) &&
        !forcedOpeningLogged) {
      recordDoorOpening(OPENING_FORCED_BOTH, nowMs);
      forcedOpeningLogged = true;
    }
  } else {
    forcedOpeningLogged = false;
    if (activeMode == MODE_AUTOMATIC) {
      if (outsideFinishedOpening && insideFinishedOpening) {
        recordDoorOpening(OPENING_BOTH_AUTOMATIC, nowMs);
      } else if (outsideFinishedOpening) {
        recordDoorOpening(OPENING_OUTBOUND, nowMs);
      } else if (insideFinishedOpening) {
        recordDoorOpening(OPENING_INBOUND, nowMs);
      }
    }
  }

  static uint32_t lastReportMs = 0;
  if (nowMs - lastReportMs >= SERIAL_REPORT_PERIOD_MS) {
    lastReportMs = nowMs;
    printStatus(nowMs, tagPresent, insideNear, outsideNear,
                automaticOutboundCondition, automaticInboundCondition,
                outsideRetractDemand, insideRetractDemand);
  }

  delay(1);
}
