#include "rc_control.h"

#include <Arduino.h>
#include "config/hardware_pins.h"
#include "hardware_master.h"
#include "link_master.h"

namespace {
constexpr uint32_t RC_READ_TIMEOUT_US = 25000;
constexpr uint16_t RC_MIN_US = 1000;
constexpr uint16_t RC_MAX_US = 2000;
constexpr uint16_t RC_CENTER_US = 1500;
constexpr uint16_t RC_DEADBAND_US = 30;
constexpr unsigned long COMMAND_INTERVAL_MS = 50;
constexpr unsigned long FAILSAFE_MS = 500;

// PCF8575 bit positions for direction pins (see Partner pin map)
enum PcfBit : uint8_t {
  FRONT_AIN1 = 0,
  FRONT_AIN2 = 1,
  FRONT_BIN1 = 2,
  FRONT_BIN2 = 3,
  MID_AIN1   = 4,
  MID_AIN2   = 5,
  MID_BIN1   = 6,
  MID_BIN2   = 7,
  BACK_AIN1  = 8,
  BACK_AIN2  = 9,
  BACK_BIN1  = 10,
  BACK_BIN2  = 11
};

unsigned long lastCommandMs = 0;
unsigned long lastValidRxMs = 0;
bool rcEnabled = true;

int16_t pulseToSigned(uint16_t pulse) {
  if (pulse < RC_MIN_US || pulse > RC_MAX_US) return 0;
  int16_t delta = static_cast<int16_t>(pulse) - static_cast<int16_t>(RC_CENTER_US);
  if (abs(delta) < RC_DEADBAND_US) delta = 0;
  // Map approx +/-500us to +/-255
  const float scale = 255.0f / 500.0f;
  int16_t val = static_cast<int16_t>(delta * scale);
  return constrain(val, -255, 255);
}

void setHBridge(uint16_t& state, int16_t value, PcfBit in1, PcfBit in2) {
  const bool forward = value >= 0;
  const bool enable = abs(value) > 0;
  auto setBit = [&](PcfBit bit, bool high) {
    if (high) state |= (1u << bit);
    else state &= ~(1u << bit);
  };

  if (!enable) {
    setBit(in1, false);
    setBit(in2, false);
    return;
  }

  setBit(in1, forward);
  setBit(in2, !forward);
}
} // namespace

void rcControlSetup() {
  lastCommandMs = millis();
  lastValidRxMs = millis();
  rcEnabled = true;
}

void rcControlLoop() {
  if (!rcEnabled) return;

  const unsigned long now = millis();
  if (now - lastCommandMs < COMMAND_INTERVAL_MS) return;

  const uint16_t steerPulse = rcReceiver.readUs(0, RC_READ_TIMEOUT_US); // CH1
  const uint16_t throttlePulse = rcReceiver.readUs(1, RC_READ_TIMEOUT_US); // CH2

  if (steerPulse >= RC_MIN_US && steerPulse <= RC_MAX_US &&
      throttlePulse >= RC_MIN_US && throttlePulse <= RC_MAX_US) {
    lastValidRxMs = now;
  }

  const bool failsafe = (now - lastValidRxMs) > FAILSAFE_MS;
  if (failsafe) {
    sendMotorPwm(0, 0, 0, 0, 0, 0);
    sendIoExpanderState(0x0000);
    lastCommandMs = now;
    return;
  }

  const int16_t steer = pulseToSigned(steerPulse);
  const int16_t throttle = pulseToSigned(throttlePulse);

  int16_t left = throttle + steer;
  int16_t right = throttle - steer;
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  const uint8_t pwmLeft = static_cast<uint8_t>(abs(left));
  const uint8_t pwmRight = static_cast<uint8_t>(abs(right));
  sendMotorPwm(pwmLeft, pwmRight, pwmLeft, pwmRight, pwmLeft, pwmRight);

  uint16_t state = 0x0000;
  // Left side uses A channels on each driver
  setHBridge(state, left, FRONT_AIN1, FRONT_AIN2);
  setHBridge(state, left, MID_AIN1, MID_AIN2);
  setHBridge(state, left, BACK_AIN1, BACK_AIN2);
  // Right side uses B channels on each driver
  setHBridge(state, right, FRONT_BIN1, FRONT_BIN2);
  setHBridge(state, right, MID_BIN1, MID_BIN2);
  setHBridge(state, right, BACK_BIN1, BACK_BIN2);
  sendIoExpanderState(state);

  lastCommandMs = now;
}

void rcControlEnable() { rcEnabled = true; }
void rcControlDisable() { rcEnabled = false; }
bool rcControlIsEnabled() { return rcEnabled; }
