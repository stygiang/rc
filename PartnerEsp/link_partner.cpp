#include "link_partner.h"
#include "config/hardware_pins.h"
#include "hardware_partner.h"

HardwareSerial& linkSerial = Serial1;

namespace {
constexpr uint8_t FRAME_STX = 0xAA;
constexpr uint8_t FRAME_MAX_PAYLOAD = 32;
constexpr uint32_t LINK_BAUD = 115200;
constexpr unsigned long PING_INTERVAL_MS = 1000;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 1000;

constexpr int LINK_RX_PIN = 21; // UART_RX_MASTER from pin table
constexpr int LINK_TX_PIN = 20; // UART_TX_MASTER from pin table

struct Frame {
  uint8_t type;
  uint8_t length;
  uint8_t payload[FRAME_MAX_PAYLOAD];
};

enum RxState { WAIT_STX, READ_TYPE, READ_LENGTH, READ_PAYLOAD, READ_CHECKSUM };

RxState rxState = WAIT_STX;
Frame rxWorking{};
uint8_t rxChecksum = 0;
uint8_t rxIndex = 0;

unsigned long lastPingMs = 0;
unsigned long lastTelemetryMs = 0;
uint8_t lastMotorSequence = 0;
uint8_t lastLightTarget = 0xFF;

void sendFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (length > FRAME_MAX_PAYLOAD) {
    Serial.println("[Partner] frame too large, dropped");
    return;
  }

  uint8_t checksum = type + length;
  linkSerial.write(FRAME_STX);
  linkSerial.write(type);
  linkSerial.write(length);

  for (uint8_t i = 0; i < length; ++i) {
    const uint8_t b = payload ? payload[i] : 0;
    checksum += b;
    linkSerial.write(b);
  }

  linkSerial.write(checksum);
}

bool pollFrame(Stream& stream, Frame& out) {
  while (stream.available()) {
    const uint8_t b = stream.read();
    switch (rxState) {
      case WAIT_STX:
        if (b == FRAME_STX) rxState = READ_TYPE;
        break;
      case READ_TYPE:
        rxWorking.type = b;
        rxChecksum = b;
        rxState = READ_LENGTH;
        break;
      case READ_LENGTH:
        rxWorking.length = b;
        rxChecksum += b;
        rxIndex = 0;
        if (rxWorking.length == 0) {
          rxState = READ_CHECKSUM;
        } else if (rxWorking.length > FRAME_MAX_PAYLOAD) {
          rxState = WAIT_STX; // drop oversized frame
        } else {
          rxState = READ_PAYLOAD;
        }
        break;
      case READ_PAYLOAD:
        rxWorking.payload[rxIndex++] = b;
        rxChecksum += b;
        if (rxIndex >= rxWorking.length) {
          rxState = READ_CHECKSUM;
        }
        break;
      case READ_CHECKSUM:
        if (rxChecksum == b) {
          out = rxWorking;
          rxState = WAIT_STX;
          return true;
        }
        rxState = WAIT_STX; // checksum mismatch
        break;
    }
  }
  return false;
}

void handleFrame(const Frame& frame) {
  switch (frame.type) {
    case MSG_PING:
      sendFrame(MSG_PONG, nullptr, 0);
      break;
    case MSG_SET_MOTOR_PWM:
      applyMotorPwmCommand(frame.payload, frame.length);
      lastMotorSequence++;
      break;
    case MSG_SET_LIGHT_RGB:
      applyLightRgbCommand(frame.payload, frame.length);
      if (frame.length >= 1) lastLightTarget = frame.payload[0];
      break;
    case MSG_SET_IOEXP_STATE:
      applyIoExpanderStateCommand(frame.payload, frame.length);
      break;
    default:
      Serial.printf("[Partner] unknown frame type 0x%02X len=%u\n", frame.type, frame.length);
      break;
  }
}

void processIncoming() {
  Frame incoming{};
  while (pollFrame(linkSerial, incoming)) {
    handleFrame(incoming);
  }
}

void sendTelemetry() {
  const uint32_t ms = millis();
  const uint8_t payload[6] = {
    static_cast<uint8_t>(ms & 0xFF),
    static_cast<uint8_t>((ms >> 8) & 0xFF),
    static_cast<uint8_t>((ms >> 16) & 0xFF),
    static_cast<uint8_t>((ms >> 24) & 0xFF),
    lastMotorSequence,
    lastLightTarget
  };
  sendFrame(MSG_TELEMETRY, payload, sizeof(payload));
}

void heartbeat() {
  const unsigned long now = millis();
  if (now - lastPingMs >= PING_INTERVAL_MS) {
    sendFrame(MSG_PING, nullptr, 0);
    lastPingMs = now;
  }

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryMs = now;
  }
}

} // namespace

void linkSetup() {
  linkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.printf("[Partner] link started at %lu bps on RX=%d TX=%d\n", LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);
}

void linkLoop() {
  processIncoming();
  heartbeat();
}
