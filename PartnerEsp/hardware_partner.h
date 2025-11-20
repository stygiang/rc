#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "link_partner.h"

class Pcf8575Driver {
public:
  explicit Pcf8575Driver(TwoWire& busRef);
  bool begin(uint8_t address);
  bool setState(uint16_t newState);
  bool setPin(uint8_t index, bool high);
  uint16_t currentState() const;

private:
  TwoWire& bus;
  uint8_t addr = 0x20;
  uint16_t state = 0xFFFF;
  bool writeState();
};

class Pca9685Driver {
public:
  explicit Pca9685Driver(TwoWire& busRef);
  bool begin(uint8_t address, int sdaPin, int sclPin, float freqHz = 1000.0f);
  void setRgb(LightTarget target, uint8_t r, uint8_t g, uint8_t b);
  void setChannelDuty8(uint8_t channel, uint8_t duty);

private:
  TwoWire& bus;
  uint8_t addr = 0x40;

  void reset();
  void setPwmFreq(float freqHz);
  static uint16_t mapDuty(uint8_t duty8);
  void setPwm8(uint8_t channel, uint8_t duty8);
  void setPwm(uint8_t channel, uint16_t on, uint16_t off);
  void write8(uint8_t reg, uint8_t val);
  void read8(uint8_t reg, uint8_t& val);
};

class MotorPwmDriver {
public:
  void begin();
  void setAll(const uint8_t* duty);

private:
  uint8_t channels[6] = {0, 1, 2, 3, 4, 5};
  const double pwmFreqHz = 20000.0;
  const uint8_t pwmResolutionBits = 8;
};

extern TwoWire partnerBus;
extern Pca9685Driver pca;
extern Pcf8575Driver pcf;
extern MotorPwmDriver motors;

void partnerHardwareSetup();

// Command hooks used by link handling
void applyMotorPwmCommand(const uint8_t* payload, uint8_t length);
void applyLightRgbCommand(const uint8_t* payload, uint8_t length);
void applyIoExpanderStateCommand(const uint8_t* payload, uint8_t length);
