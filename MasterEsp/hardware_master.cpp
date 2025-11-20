#include "hardware_master.h"
#include "config/hardware_pins.h"

RcReceiver rcReceiver;
BatteryMonitor batteryMonitor;
ToFSensor tofFL;
ToFSensor tofFR;
ToFSensor tofBK;

namespace {
unsigned long lastHwLogMs = 0;
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

#ifdef USE_VL53L1X
uint8_t ToFSensor::nextBus = 0;
#endif

bool ToFSensor::begin(int sda, int scl, uint8_t address) {
#ifdef USE_VL53L1X
  const uint8_t busIndex = nextBus % WIRE_INTERFACES_COUNT;
  ++nextBus;
  bus = new TwoWire(busIndex);
  if (!bus) return false;
  bus->begin(sda, scl, 400000);
  sensor.setBus(bus);
  sensor.setAddress(address);
  sensor.setTimeout(10);
  if (!sensor.init()) {
    ok = false;
    return false;
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  ok = true;
  return true;
#else
  (void)sda; (void)scl; (void)address;
  return true;
#endif
}

int ToFSensor::readMillimeters() {
#ifdef USE_VL53L1X
  if (!ok) return -1;
  if (!sensor.dataReady()) return -1;
  sensor.read();
  return static_cast<int>(sensor.rangingData.range_mm);
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

  const uint16_t ch1 = rcReceiver.readUs(0);
  const uint16_t ch2 = rcReceiver.readUs(1);
  const float battery = batteryMonitor.readVolts();

  Serial.printf("[Master] RC ch1=%u ch2=%u battery=%.2fV\n", ch1, ch2, battery);
  Serial.printf("[Master] ToF FL=%dmm FR=%dmm BK=%dmm\n",
                tofFL.readMillimeters(),
                tofFR.readMillimeters(),
                tofBK.readMillimeters());
}
