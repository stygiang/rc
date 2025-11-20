#include "hardware_partner.h"
#include "config/hardware_pins.h"

TwoWire partnerBus = TwoWire(0);
Pca9685Driver pca(partnerBus);
Pcf8575Driver pcf(partnerBus);
MotorPwmDriver motors;

Pcf8575Driver::Pcf8575Driver(TwoWire& busRef) : bus(busRef) {}

bool Pcf8575Driver::begin(uint8_t address) {
  addr = address;
  state = 0xFFFF; // inputs/high by default
  return writeState();
}

bool Pcf8575Driver::setState(uint16_t newState) {
  state = newState;
  return writeState();
}

bool Pcf8575Driver::setPin(uint8_t index, bool high) {
  if (index >= 16) return false;
  if (high) state |= (1u << index);
  else state &= ~(1u << index);
  return writeState();
}

uint16_t Pcf8575Driver::currentState() const { return state; }

bool Pcf8575Driver::writeState() {
  bus.beginTransmission(addr);
  bus.write(static_cast<uint8_t>(state & 0xFF));
  bus.write(static_cast<uint8_t>((state >> 8) & 0xFF));
  return bus.endTransmission() == 0;
}

Pca9685Driver::Pca9685Driver(TwoWire& busRef) : bus(busRef) {}

bool Pca9685Driver::begin(uint8_t address, int sdaPin, int sclPin, float freqHz) {
  addr = address;
  bus.begin(sdaPin, sclPin, 400000);
  reset();
  setPwmFreq(freqHz);
  return true;
}

void Pca9685Driver::setRgb(LightTarget target, uint8_t r, uint8_t g, uint8_t b) {
  if (target > LIGHT_BACK_RIGHT) return;
  static const uint8_t lightChannelMap[4][3] = {
    {0, 1, 2},   // front left
    {3, 4, 5},   // front right
    {6, 7, 8},   // back left
    {9, 10, 11}  // back right
  };
  const uint8_t* map = lightChannelMap[target];
  setPwm8(map[0], r);
  setPwm8(map[1], g);
  setPwm8(map[2], b);
}

void Pca9685Driver::setChannelDuty8(uint8_t channel, uint8_t duty) {
  setPwm(channel, 0, mapDuty(duty));
}

void Pca9685Driver::reset() {
  static constexpr uint8_t MODE1 = 0x00;
  write8(MODE1, 0x00);
  delay(10);
}

void Pca9685Driver::setPwmFreq(float freqHz) {
  static constexpr uint8_t MODE1 = 0x00;
  static constexpr uint8_t PRESCALE = 0xFE;
  float prescaleval = 25000000.0f;
  prescaleval /= 4096.0f;
  prescaleval /= freqHz;
  prescaleval -= 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(prescaleval + 0.5f);

  uint8_t oldmode = 0;
  read8(MODE1, oldmode);
  const uint8_t sleep = (oldmode & 0x7F) | 0x10; // sleep
  write8(MODE1, sleep);
  write8(PRESCALE, prescale);
  write8(MODE1, oldmode);
  delay(5);
  write8(MODE1, oldmode | 0xA1); // restart + auto-increment
}

uint16_t Pca9685Driver::mapDuty(uint8_t duty8) {
  return static_cast<uint16_t>((static_cast<uint32_t>(duty8) * 4095u + 127u) / 255u);
}

void Pca9685Driver::setPwm8(uint8_t channel, uint8_t duty8) {
  setPwm(channel, 0, mapDuty(duty8));
}

void Pca9685Driver::setPwm(uint8_t channel, uint16_t on, uint16_t off) {
  if (channel > 15) return;
  static constexpr uint8_t LED0_ON_L = 0x06;
  const uint8_t reg = LED0_ON_L + 4 * channel;
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(static_cast<uint8_t>(on & 0xFF));
  bus.write(static_cast<uint8_t>((on >> 8) & 0xFF));
  bus.write(static_cast<uint8_t>(off & 0xFF));
  bus.write(static_cast<uint8_t>((off >> 8) & 0xFF));
  bus.endTransmission();
}

void Pca9685Driver::write8(uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}

void Pca9685Driver::read8(uint8_t reg, uint8_t& val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.endTransmission();
  bus.requestFrom(addr, static_cast<uint8_t>(1));
  if (bus.available()) {
    val = bus.read();
  }
}

void MotorPwmDriver::begin() {
  const uint8_t pins[6] = { 6, 5, 1, 2, 3, 4 }; // from PARTNER_PINS
  const uint8_t chans[6] = { 0, 1, 2, 3, 4, 5 };
  for (uint8_t i = 0; i < 6; ++i) {
    ledcSetup(chans[i], pwmFreqHz, pwmResolutionBits);
    ledcAttachPin(pins[i], chans[i]);
    channels[i] = chans[i];
  }
}

void MotorPwmDriver::setAll(const uint8_t* duty) {
  if (!duty) return;
  for (uint8_t i = 0; i < 6; ++i) {
    ledcWrite(channels[i], duty[i]);
  }
}

void partnerHardwareSetup() {
  partnerBus.begin(PCA9685_PWM.sdaPin, PCA9685_PWM.sclPin, 400000);
  pca.begin(PCA9685_PWM.i2cAddress, PCA9685_PWM.sdaPin, PCA9685_PWM.sclPin, 600.0f);
  pcf.begin(IOEXP_PCF8575.i2cAddress);
  motors.begin();
}

void applyMotorPwmCommand(const uint8_t* payload, uint8_t length) {
  if (length != 6) {
    Serial.printf("[Partner] bad motor payload len=%u\n", length);
    return;
  }
  motors.setAll(payload);
  Serial.printf("[Partner] motor PWM front(%u,%u) mid(%u,%u) back(%u,%u)\n",
                payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
}

void applyLightRgbCommand(const uint8_t* payload, uint8_t length) {
  if (length != 4) {
    Serial.printf("[Partner] bad light payload len=%u\n", length);
    return;
  }

  const uint8_t rawTarget = payload[0];
  const uint8_t r = payload[1];
  const uint8_t g = payload[2];
  const uint8_t b = payload[3];
  if (rawTarget > LIGHT_BACK_RIGHT) {
    Serial.printf("[Partner] light target out of range: %u\n", rawTarget);
    return;
  }

  pca.setRgb(static_cast<LightTarget>(rawTarget), r, g, b);
  Serial.printf("[Partner] light target=%u rgb=(%u,%u,%u)\n", rawTarget, r, g, b);
}

void applyIoExpanderStateCommand(const uint8_t* payload, uint8_t length) {
  if (length != 2) {
    Serial.printf("[Partner] bad IO expander payload len=%u\n", length);
    return;
  }
  const uint16_t state = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
  if (pcf.setState(state)) {
    Serial.printf("[Partner] IO expander state=0x%04X\n", state);
  } else {
    Serial.println("[Partner] IO expander write failed");
  }
}
