#include "link_master.h"
#include "config/hardware_pins.h"

// Basic framed UART link between Master and Partner ESP32 boards.

HardwareSerial& linkSerial = Serial1;

namespace {
constexpr uint8_t FRAME_STX = 0xAA;
constexpr uint8_t FRAME_MAX_PAYLOAD = 32;
constexpr uint32_t LINK_BAUD = 115200;
constexpr unsigned long PING_INTERVAL_MS = 1000;
constexpr unsigned long PONG_TIMEOUT_MS = 4000;

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
unsigned long lastPongMs = 0;
unsigned long lastTelemetryMs = 0;

void sendFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (length > FRAME_MAX_PAYLOAD) {
    Serial.println("[Master] frame too large, dropped");
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
    case MSG_PONG:
      lastPongMs = millis();
      break;
    case MSG_TELEMETRY:
      if (frame.length >= 6) {
        const uint32_t ms =
            (static_cast<uint32_t>(frame.payload[0])) |
            (static_cast<uint32_t>(frame.payload[1]) << 8) |
            (static_cast<uint32_t>(frame.payload[2]) << 16) |
            (static_cast<uint32_t>(frame.payload[3]) << 24);
        const uint8_t motorSeq = frame.payload[4];
        const uint8_t lightTarget = frame.payload[5];
        lastTelemetryMs = millis();
        Serial.printf("[Master] telemetry uptime=%lus motorSeq=%u lastLightTarget=%u\n",
                      ms / 1000UL, motorSeq, lightTarget);
      } else {
        Serial.printf("[Master] telemetry payload len=%u (expected >=6)\n", frame.length);
      }
      break;
    default:
      Serial.printf("[Master] unknown frame type 0x%02X len=%u\n", frame.type, frame.length);
      break;
  }
}

void processIncoming() {
  Frame incoming{};
  while (pollFrame(linkSerial, incoming)) {
    handleFrame(incoming);
  }
}

void heartbeat() {
  const unsigned long now = millis();
  if (now - lastPingMs >= PING_INTERVAL_MS) {
    sendFrame(MSG_PING, nullptr, 0);
    lastPingMs = now;
  }

  if (lastPongMs > 0 && (now - lastPongMs) > PONG_TIMEOUT_MS) {
    Serial.println("[Master] partner link timeout");
    lastPongMs = now; // avoid spamming while link is down
  }
}

} // namespace

void linkSetup() {
  linkSerial.begin(LINK_BAUD, SERIAL_8N1, ESP32_RX0_PIN, ESP32_TX0_PIN);
  Serial.printf("[Master] link started at %lu bps on RX=%d TX=%d\n", LINK_BAUD, ESP32_RX0_PIN, ESP32_TX0_PIN);
}

void linkLoop() {
  processIncoming();
  heartbeat();
}

void sendMotorPwm(uint8_t frontA, uint8_t frontB, uint8_t midA, uint8_t midB, uint8_t backA, uint8_t backB) {
  const uint8_t payload[6] = { frontA, frontB, midA, midB, backA, backB };
  sendFrame(MSG_SET_MOTOR_PWM, payload, sizeof(payload));
}

void sendLightRgb(LightTarget target, uint8_t r, uint8_t g, uint8_t b) {
  const uint8_t payload[4] = { static_cast<uint8_t>(target), r, g, b };
  sendFrame(MSG_SET_LIGHT_RGB, payload, sizeof(payload));
}

void sendIoExpanderState(uint16_t state) {
  const uint8_t payload[2] = { static_cast<uint8_t>(state & 0xFF), static_cast<uint8_t>((state >> 8) & 0xFF) };
  sendFrame(MSG_SET_IOEXP_STATE, payload, sizeof(payload));
}

void sendPingCommand() {
  sendFrame(MSG_PING, nullptr, 0);
}
