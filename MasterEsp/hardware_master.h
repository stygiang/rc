#pragma once

#include <Arduino.h>

// Uncomment to enable Adafruit VL53L0X support when the library is installed.
#define USE_ADAFRUIT_VL53L0X

#ifdef USE_ADAFRUIT_VL53L0X
#include <Adafruit_VL53L0X.h>
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
  bool readMeasurement(int& mm, uint8_t& status);

private:
#ifdef USE_ADAFRUIT_VL53L0X
  Adafruit_VL53L0X sensor;
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
void hardwareLoggingEnable();
void hardwareLoggingDisable();
