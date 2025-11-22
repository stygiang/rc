#include "console_master.h"

#include <Arduino.h>
#include "link_master.h"
#include "rc_control.h"
#include "hardware_master.h"
#include "config/hardware_pins.h"

namespace {
String lineBuf;
bool manualMode = false;

int16_t clampSigned(int value, int minv, int maxv) {
  if (value < minv) return minv;
  if (value > maxv) return maxv;
  return value;
}

void printHelp() {
  Serial.println("=== Master Console ===");
  Serial.println("mode rc              - enable RC control");
  Serial.println("mode manual          - disable RC control (manual commands only)");
  Serial.println("motor <L> <R>        - set left/right speed (-255..255)");
  Serial.println("stop                 - stop motors and clear directions");
  Serial.println("light <tgt> <r> <g> <b>  tgt 0=FL 1=FR 2=BL 3=BR");
  Serial.println("io <hex>             - set PCF8575 state (16-bit, e.g., 0x00FF)");
  Serial.println("ping                 - send ping");
  Serial.println("status               - show mode and last RC pulses");
  Serial.println("log on|off           - toggle hardware logging (RC/battery/ToF)");
  Serial.println("tof [fl|fr|bk]       - read one ToF measurement with status");
  Serial.println("help                 - show this help");
}

uint16_t hexToUint16(const String& token) {
  if (token.startsWith("0x") || token.startsWith("0X")) {
    return static_cast<uint16_t>(strtoul(token.c_str(), nullptr, 16));
  }
  return static_cast<uint16_t>(token.toInt());
}

void handleCommand(const String& cmdLine) {
  // Split by spaces
  String tokens[6];
  int count = 0;
  int start = 0;
  while (start < cmdLine.length() && count < 6) {
    int space = cmdLine.indexOf(' ', start);
    if (space == -1) space = cmdLine.length();
    if (space > start) {
      tokens[count++] = cmdLine.substring(start, space);
    }
    start = space + 1;
  }
  if (count == 0) return;

  const String& cmd = tokens[0];

  if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("h") || cmd.equals("?")) {
    printHelp();
    return;
  }

  if (cmd.equalsIgnoreCase("mode")) {
    if (count < 2) {
      Serial.println("usage: mode rc|manual");
      return;
    }
    if (tokens[1].equalsIgnoreCase("rc")) {
      manualMode = false;
      rcControlEnable();
      Serial.println("[Mode] RC control enabled");
    } else if (tokens[1].equalsIgnoreCase("manual")) {
      manualMode = true;
      rcControlDisable();
      Serial.println("[Mode] Manual control enabled");
    } else {
      Serial.println("usage: mode rc|manual");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("status")) {
    Serial.printf("[Status] mode=%s rcEnabled=%s\n",
                  manualMode ? "manual" : "rc",
                  rcControlIsEnabled() ? "yes" : "no");
    const uint16_t ch1 = rcReceiver.readUs(0);
    const uint16_t ch2 = rcReceiver.readUs(1);
    const float battery = batteryMonitor.readVolts();
    Serial.printf("[Status] RC ch1=%u ch2=%u battery=%.2fV\n", ch1, ch2, battery);
    return;
  }

  if (cmd.equalsIgnoreCase("ping")) {
    sendPingCommand();
    Serial.println("[Console] ping sent");
    return;
  }

  if (cmd.equalsIgnoreCase("log")) {
    if (count < 2) {
      Serial.println("usage: log on|off");
      return;
    }
    if (tokens[1].equalsIgnoreCase("on")) {
      hardwareLoggingEnable();
      Serial.println("[Console] hardware logging ON");
    } else if (tokens[1].equalsIgnoreCase("off")) {
      hardwareLoggingDisable();
      Serial.println("[Console] hardware logging OFF");
    } else {
      Serial.println("usage: log on|off");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("motor")) {
    if (count < 3) {
      Serial.println("usage: motor <left -255..255> <right -255..255>");
      return;
    }
    const int leftRaw = tokens[1].toInt();
    const int rightRaw = tokens[2].toInt();
    const int16_t left = clampSigned(leftRaw, -255, 255);
    const int16_t right = clampSigned(rightRaw, -255, 255);
    const uint8_t pwmLeft = static_cast<uint8_t>(abs(left));
    const uint8_t pwmRight = static_cast<uint8_t>(abs(right));
    sendMotorPwm(pwmLeft, pwmRight, pwmLeft, pwmRight, pwmLeft, pwmRight);

    // Direction bits
    auto setHBridge = [](uint16_t& state, int16_t value, uint8_t in1, uint8_t in2) {
      const bool forward = value >= 0;
      const bool enable = abs(value) > 0;
      if (!enable) {
        state &= ~(1u << in1);
        state &= ~(1u << in2);
        return;
      }
      if (forward) {
        state |= (1u << in1);
        state &= ~(1u << in2);
      } else {
        state &= ~(1u << in1);
        state |= (1u << in2);
      }
    };

    uint16_t state = 0;
    setHBridge(state, left, 0, 1);  // FRONT_A
    setHBridge(state, left, 4, 5);  // MID_A
    setHBridge(state, left, 8, 9);  // BACK_A
    setHBridge(state, right, 2, 3); // FRONT_B
    setHBridge(state, right, 6, 7); // MID_B
    setHBridge(state, right, 10, 11); // BACK_B
    sendIoExpanderState(state);

    Serial.printf("[Console] motor L=%d R=%d PWM=(%u,%u)\n", left, right, pwmLeft, pwmRight);
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    sendMotorPwm(0, 0, 0, 0, 0, 0);
    sendIoExpanderState(0x0000);
    Serial.println("[Console] motors stopped, directions cleared");
    return;
  }

  if (cmd.equalsIgnoreCase("light")) {
    if (count < 5) {
      Serial.println("usage: light <tgt 0-3> <r 0-255> <g 0-255> <b 0-255>");
      return;
    }
    const uint8_t tgt = static_cast<uint8_t>(tokens[1].toInt());
    const uint8_t r = static_cast<uint8_t>(tokens[2].toInt());
    const uint8_t g = static_cast<uint8_t>(tokens[3].toInt());
    const uint8_t b = static_cast<uint8_t>(tokens[4].toInt());
    sendLightRgb(static_cast<LightTarget>(tgt), r, g, b);
    Serial.printf("[Console] light tgt=%u rgb=(%u,%u,%u)\n", tgt, r, g, b);
    return;
  }

  if (cmd.equalsIgnoreCase("io")) {
    if (count < 2) {
      Serial.println("usage: io <hex|int>");
      return;
    }
    const uint16_t state = hexToUint16(tokens[1]);
    sendIoExpanderState(state);
    Serial.printf("[Console] IO state=0x%04X\n", state);
    return;
  }

  if (cmd.equalsIgnoreCase("tof")) {
    ToFSensor* sensor = &tofFL;
    const char* name = "FL";
    if (count >= 2) {
      if (tokens[1].equalsIgnoreCase("fr")) {
        sensor = &tofFR;
        name = "FR";
      } else if (tokens[1].equalsIgnoreCase("bk") || tokens[1].equalsIgnoreCase("back")) {
        sensor = &tofBK;
        name = "BK";
      } else if (!tokens[1].equalsIgnoreCase("fl")) {
        Serial.println("usage: tof [fl|fr|bk]");
        return;
      }
    }
    int mm = -1;
    uint8_t status = 0xFF;
    if (sensor->readMeasurement(mm, status)) {
      Serial.printf("[Console] ToF %s: %d mm (status=0x%02X)\n", name, mm, status);
    } else {
      Serial.printf("[Console] ToF %s: no data (status=0x%02X)\n", name, status);
    }
    return;
  }

  Serial.println("Unknown command. Type 'help' for commands.");
}
} // namespace

void consoleSetup() {
  printHelp();
}

void consoleLoop() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf.trim();
      if (lineBuf.length() > 0) {
        handleCommand(lineBuf);
      }
      lineBuf = "";
    } else {
      lineBuf += c;
      if (lineBuf.length() > 120) {
        lineBuf = "";
      }
    }
  }
}
