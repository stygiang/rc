#pragma once

#include <Arduino.h>

// Uncomment to enable VL53L1X support when the library is installed.
// #define USE_VL53L1X

#ifdef USE_VL53L1X
#include <VL53L1X.h>
#ifndef WIRE_INTERFACES_COUNT
#define WIRE_INTERFACES_COUNT 2
#endif
#endif

struct PinDef;

struct RcReceiver {
  const PinDef* pins = nullptr;
  uint8_t count = 0;

  void begin(const PinDef* pinTable, uint8_t pinCount);
  uint16_t readUs(uint8_t idx, uint32_t timeoutUs = 25000) const;
};

struct BatteryMonitor {
  int pin = -1;
  float dividerRatio = 2.0f;
  float vref = 3.3f;

  void begin(int analogPin, float divider = 2.0f, float vRef = 3.3f);
  float readVolts() const;
};

class ToFSensor {
public:
  bool begin(int sda, int scl, uint8_t address = 0x29);
  int readMillimeters();

private:
#ifdef USE_VL53L1X
  static uint8_t nextBus;
  TwoWire* bus = nullptr;
  VL53L1X sensor;
  bool ok = false;
#endif
};

extern RcReceiver rcReceiver;
extern BatteryMonitor batteryMonitor;
extern ToFSensor tofFL;
extern ToFSensor tofFR;
extern ToFSensor tofBK;

void hardwareSetup();
void hardwareLoop();
