// UART tag-detection test for an ESP32 DevKit V1 and JRD-4035 UHF reader.
//
// Default wiring used by this sketch:
//   JRD-4035 TX -> ESP32 GPIO 16 (RFID_RX_PIN)
//   JRD-4035 RX -> ESP32 GPIO 17 (RFID_TX_PIN)
//   JRD-4035 GND -> ESP32 GND
//
// The reader is polled repeatedly. Open Arduino Serial Monitor at 115200 baud.

#include <Arduino.h>

const uint8_t RFID_RX_PIN = 16;  // ESP32 RX: connect to reader TX.
const uint8_t RFID_TX_PIN = 17;  // ESP32 TX: connect to reader RX.

const uint32_t USB_SERIAL_BAUD = 115200;
const uint32_t RFID_UART_BAUD = 115200;
const uint32_t RESPONSE_WINDOW_MS = 350;
const uint32_t POLL_INTERVAL_MS = 500;

// Set true if you want every valid UART frame printed in hexadecimal.
const bool PRINT_RAW_FRAMES = false;

const size_t MAX_PAYLOAD_LENGTH = 128;

// JRD-4035 single-polling command from its UART protocol documentation.
const uint8_t SINGLE_POLL_COMMAND[] = {
  0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E
};

HardwareSerial rfidUart(2);

struct RfidFrame {
  uint8_t type;
  uint8_t command;
  uint16_t payloadLength;
  uint8_t payload[MAX_PAYLOAD_LENGTH];
  uint8_t checksum;
};

struct TagReading {
  bool valid;
  int8_t rssiDbm;
  size_t epcLength;
  uint8_t epc[MAX_PAYLOAD_LENGTH];
};

bool readByteBefore(uint8_t &value, uint32_t deadlineMs) {
  while ((int32_t)(deadlineMs - millis()) > 0) {
    if (rfidUart.available() > 0) {
      value = (uint8_t)rfidUart.read();
      return true;
    }
    delay(1);
  }
  return false;
}

bool readFrame(RfidFrame &frame, uint32_t timeoutMs) {
  const uint32_t deadlineMs = millis() + timeoutMs;
  uint8_t value = 0;

  // Discard any noise until the frame header appears.
  do {
    if (!readByteBefore(value, deadlineMs)) {
      return false;
    }
  } while (value != 0xBB);

  uint8_t lengthMsb = 0;
  uint8_t lengthLsb = 0;
  if (!readByteBefore(frame.type, deadlineMs) ||
      !readByteBefore(frame.command, deadlineMs) ||
      !readByteBefore(lengthMsb, deadlineMs) ||
      !readByteBefore(lengthLsb, deadlineMs)) {
    return false;
  }

  frame.payloadLength = ((uint16_t)lengthMsb << 8) | lengthLsb;
  if (frame.payloadLength > MAX_PAYLOAD_LENGTH) {
    Serial.printf("RFID frame is too large (%u bytes); discarding it.\n",
                  frame.payloadLength);
    return false;
  }

  uint8_t calculatedChecksum = frame.type + frame.command + lengthMsb + lengthLsb;
  for (size_t index = 0; index < frame.payloadLength; ++index) {
    if (!readByteBefore(frame.payload[index], deadlineMs)) {
      return false;
    }
    calculatedChecksum += frame.payload[index];
  }

  uint8_t frameEnd = 0;
  if (!readByteBefore(frame.checksum, deadlineMs) ||
      !readByteBefore(frameEnd, deadlineMs)) {
    return false;
  }

  if (frameEnd != 0x7E) {
    Serial.println("Invalid RFID frame ending received.");
    return false;
  }

  if (frame.checksum != calculatedChecksum) {
    Serial.println("Invalid RFID frame checksum received.");
    return false;
  }

  return true;
}

void printRawFrame(const RfidFrame &frame) {
  Serial.printf("RX: BB %02X %02X %02X %02X", frame.type, frame.command,
                (uint8_t)(frame.payloadLength >> 8),
                (uint8_t)frame.payloadLength);

  for (size_t index = 0; index < frame.payloadLength; ++index) {
    Serial.printf(" %02X", frame.payload[index]);
  }

  Serial.printf(" %02X 7E\n", frame.checksum);
}

bool frameContainsTag(const RfidFrame &frame, TagReading &tag) {
  // A tag notification contains RSSI (1), PC (2), EPC (variable), and CRC (2).
  if (frame.type != 0x02 || frame.command != 0x22 ||
      frame.payloadLength < 5) {
    return false;
  }

  const size_t epcLength = frame.payloadLength - 5;
  tag.valid = true;
  tag.rssiDbm = (int8_t)frame.payload[0];
  tag.epcLength = epcLength;

  // EPC starts after the RSSI byte and the two-byte PC value.
  for (size_t index = 0; index < epcLength; ++index) {
    tag.epc[index] = frame.payload[index + 3];
  }

  return true;
}

void printTagResult(const TagReading &tag) {
  Serial.printf("[%lu ms] TAG SEEN | EPC=", millis());
  for (size_t index = 0; index < tag.epcLength; ++index) {
    Serial.printf("%02X", tag.epc[index]);
  }
  Serial.printf(" | RSSI=%d dBm\n", tag.rssiDbm);
}

void pollForTag() {
  // Remove stale bytes so this result belongs to the new poll.
  while (rfidUart.available() > 0) {
    rfidUart.read();
  }

  rfidUart.write(SINGLE_POLL_COMMAND, sizeof(SINGLE_POLL_COMMAND));
  rfidUart.flush();

  const uint32_t responseDeadlineMs = millis() + RESPONSE_WINDOW_MS;
  bool receivedValidFrame = false;
  TagReading firstTag = {false, 0, 0, {0}};

  while ((int32_t)(responseDeadlineMs - millis()) > 0) {
    RfidFrame frame;
    const uint32_t timeRemainingMs = responseDeadlineMs - millis();

    if (!readFrame(frame, timeRemainingMs)) {
      break;
    }

    receivedValidFrame = true;

    if (PRINT_RAW_FRAMES) {
      printRawFrame(frame);
    }

    if (!firstTag.valid) {
      frameContainsTag(frame, firstTag);
    }
  }

  if (firstTag.valid) {
    printTagResult(firstTag);
  } else if (receivedValidFrame) {
    Serial.printf("[%lu ms] NO TAG SEEN\n", millis());
  } else {
    Serial.printf(
      "[%lu ms] NO UART RESPONSE | Check power, common GND, RX/TX wiring, and baud rate.\n",
      millis());
  }
}

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  delay(1000);

  rfidUart.begin(RFID_UART_BAUD, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);

  Serial.println();
  Serial.println("JRD-4035 UART RFID tag test starting");
  Serial.printf("Reader UART: %lu baud, ESP32 RX=%u, TX=%u\n",
                RFID_UART_BAUD, RFID_RX_PIN, RFID_TX_PIN);
  Serial.println("Polling continuously...");
}

void loop() {
  pollForTag();
  delay(POLL_INTERVAL_MS);
}
