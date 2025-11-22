#include "hardware_master.h"
#include "config/hardware_pins.h"

RcReceiver rcReceiver;
BatteryMonitor batteryMonitor;
ToFSensor tofFL;
ToFSensor tofFR;
ToFSensor tofBK;

namespace {
unsigned long lastHwLogMs = 0;
bool hwLoggingEnabled = false;
}

void RcReceiver::begin(const PinDef* pinTable, uint8_t pinCount) {
  pins = pinTable;
  count = pinCount;
  for (uint8_t i = 0; i < count; ++i) {
    pinMode(pins[i].pinNumber, INPUT);
  }
}

uint16_t RcReceiver::readUs(uint8_t idx, uint32_t timeoutUs) const {
  if (!pins || idx >= count) return 0;
  return pulseIn(pins[idx].pinNumber, HIGH, timeoutUs);
}

void BatteryMonitor::begin(int analogPin, float divider, float vRef) {
  pin = analogPin;
  dividerRatio = divider;
  vref = vRef;
  pinMode(pin, INPUT);
  analogReadResolution(12);
}

float BatteryMonitor::readVolts() const {
  if (pin < 0) return 0.0f;
  const int raw = analogRead(pin);
  const float v = (static_cast<float>(raw) / 4095.0f) * vref * dividerRatio;
  return v;
}

bool ToFSensor::begin(int sda, int scl, uint8_t address) {
#ifdef USE_ADAFRUIT_VL53L0X
  Wire.begin(sda, scl);
  if (!sensor.begin(address, false, &Wire)) {
    ok = false;
    Serial.printf("[Master][ToF] init failed at addr 0x%02X on SDA=%d SCL=%d\n", address, sda, scl);
    return false;
  }
  sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
  ok = true;
  Serial.printf("[Master][ToF] init ok addr 0x%02X SDA=%d SCL=%d\n", address, sda, scl);
  return true;
#else
  (void)sda; (void)scl; (void)address;
  return true;
#endif
}

bool ToFSensor::readMeasurement(int& mm, uint8_t& status) {
#ifdef USE_ADAFRUIT_VL53L0X
  if (!ok) return false;
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  status = measure.RangeStatus;
  if (status == 0) {
    mm = static_cast<int>(measure.RangeMilliMeter);
    return true;
  }
  return false;
#else
  (void)mm; (void)status;
  return false;
#endif
}

int ToFSensor::readMillimeters() {
#ifdef USE_ADAFRUIT_VL53L0X
  int mm = -1;
  uint8_t status = 0xFF;
  if (readMeasurement(mm, status)) return mm;
  return -1;
#else
  return -1;
#endif
}

void hardwareSetup() {
  rcReceiver.begin(MASTER_PINS, 6); // first 6 entries are RC channels
  constexpr float BATTERY_DIVIDER_RATIO = 2.0f; // update to match your resistor ladder
  constexpr float BATTERY_VREF = 3.3f;          // update if using a different ADC reference
  batteryMonitor.begin(26, BATTERY_DIVIDER_RATIO, BATTERY_VREF);
  tofFL.begin(18, 19);
  tofFR.begin(22, 23);
  tofBK.begin(27, 14);
}

void hardwareLoop() {
  const unsigned long now = millis();
  if (now - lastHwLogMs < 1000) return;
  lastHwLogMs = now;

  if (!hwLoggingEnabled) return;

  const uint16_t ch1 = rcReceiver.readUs(0);
  const uint16_t ch2 = rcReceiver.readUs(1);
  const float battery = batteryMonitor.readVolts();

  Serial.printf("[Master] RC ch1=%u ch2=%u battery=%.2fV\n", ch1, ch2, battery);
  Serial.printf("[Master] ToF FL=%dmm FR=%dmm BK=%dmm\n",
                tofFL.readMillimeters(),
                tofFR.readMillimeters(),
                tofBK.readMillimeters());
}

void hardwareLoggingEnable() { hwLoggingEnabled = true; }
void hardwareLoggingDisable() { hwLoggingEnabled = false; }
