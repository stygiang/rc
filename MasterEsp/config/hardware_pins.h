#pragma once

#include <Arduino.h>

// Hardware + pin map for the Master ESP32 only.

// -----------------------------
// ENUMS: what kind of hardware?
// -----------------------------
enum HardwareKind {
  HW_MASTER_ESP
};

// -----------------------------
// ENUMS: what is a pin used for?
// -----------------------------
enum PinFunction {
  PINF_UNASSIGNED = 0,
  PINF_RC_CHANNEL,
  PINF_UART_TX,
  PINF_UART_RX,
  PINF_I2C_SDA,
  PINF_I2C_SCL,
  PINF_MOTOR_IN,      // IN1/IN2 etc
  PINF_MOTOR_PWM,     // PWM from PCA / MCU
  PINF_TOF_SDA,
  PINF_TOF_SCL,
  PINF_TOF_SHUTDOWN,
  PINF_BATTERY_SENSE,
  PINF_RGB_RED,
  PINF_RGB_GREEN,
  PINF_RGB_BLUE
};

// -----------------------------
// Pin definition: universal pin
// -----------------------------
struct PinDef {
  const char*   name;        // logical name (CH1, SDA_TOF_FL, etc)
  int           pinNumber;   // GPIO number
  PinFunction   function;    // purpose
  const char*   component;   // human readable description
};

// -----------------------------
// Hardware definition: universal
// -----------------------------
struct Hardware {
  const char*       name;        // "Master ESP32-32D"
  const char*       description; // free-text description
  HardwareKind      kind;

  // Optional connection info
  int               rxPin;       // for UART (if applicable, else -1)
  int               txPin;       // for UART (if applicable, else -1)

  // I2C bus pins (if this hardware sits on an MCU)
  int               sdaPin;      // -1 if not directly tied on this board
  int               sclPin;      // -1 if not directly tied on this board

  // I2C address if this device is on a bus (0 = not used / not I2C)
  uint8_t           i2cAddress;

  // Pin table
  const PinDef*     pins;
  uint8_t           pinCount;
};

// TX0 / RX0 for classic ESP32
#define ESP32_TX0_PIN 1
#define ESP32_RX0_PIN 3

// -----------------------------
// MASTER ESP32 PINS
// -----------------------------
const PinDef MASTER_PINS[] = {
  // RC receiver channels
  { "CH1",       15, PINF_RC_CHANNEL, "RC receiver channel 1" },
  { "CH2",        2, PINF_RC_CHANNEL, "RC receiver channel 2" },
  { "CH3",        4, PINF_RC_CHANNEL, "RC receiver channel 3" },
  { "CH4",       16, PINF_RC_CHANNEL, "RC receiver channel 4" },
  { "CH5",       17, PINF_RC_CHANNEL, "RC receiver channel 5" },
  { "CH6",        5, PINF_RC_CHANNEL, "RC receiver channel 6" },

  // UART link to partner ESP
  { "UART_TX0",  ESP32_TX0_PIN, PINF_UART_TX, "TX0 to partner ESP" },
  { "UART_RX0",  ESP32_RX0_PIN, PINF_UART_RX, "RX0 from partner ESP" },

  // ToF front-left sensor on its own I2C pair
  { "SDA_TOF_FL", 18, PINF_TOF_SDA,  "ToF FL sensor SDA" },
  { "SCL_TOF_FL", 19, PINF_TOF_SCL,  "ToF FL sensor SCL" },

  // ToF front-right sensor
  { "SDA_TOF_FR", 22, PINF_TOF_SDA,  "ToF FR sensor SDA" },
  { "SCL_TOF_FR", 23, PINF_TOF_SCL,  "ToF FR sensor SCL" },

  // ToF back sensor
  { "SCL_TOF_BK", 14, PINF_TOF_SCL,  "ToF BK sensor SCL" },
  { "SDA_TOF_BK", 27, PINF_TOF_SDA,  "ToF BK sensor SDA" },

  // Battery measurement
  { "BATTERY_VOLT", 26, PINF_BATTERY_SENSE, "Battery voltage divider input" },

  // Free / untitled pins (from your doc)
  { "FREE_25",   25, PINF_UNASSIGNED, "Available GPIO" },
  { "FREE_32",   32, PINF_UNASSIGNED, "Available GPIO" },
  { "FREE_33",   33, PINF_UNASSIGNED, "Available GPIO" },
  { "FREE_34",   34, PINF_UNASSIGNED, "Available GPIO (input only on ESP32)" },
  { "FREE_35",   35, PINF_UNASSIGNED, "Available GPIO (input only on ESP32)" }
};

const Hardware MASTER_ESP32 = {
  "Master ESP32-32D",
  "Handles RC receiver, ToF sensors and battery sense",
  HW_MASTER_ESP,
  ESP32_RX0_PIN,      // rxPin
  ESP32_TX0_PIN,      // txPin
  -1,                 // sdaPin (multiple I2C busses, so leave -1)
  -1,                 // sclPin
  0,                  // i2cAddress (not an I2C peripheral)
  MASTER_PINS,
  (uint8_t)(sizeof(MASTER_PINS) / sizeof(MASTER_PINS[0]))
};
