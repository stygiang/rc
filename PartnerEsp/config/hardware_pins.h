#pragma once

#include <Arduino.h>

// Hardware + pin map for the Partner ESP32, plus attached peripherals.

// -----------------------------
// ENUMS: what kind of hardware?
// -----------------------------
enum HardwareKind {
  HW_PARTNER_ESP,
  HW_IO_EXPANDER_PCF8575,
  HW_PWM_PCA9685
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
  int           pinNumber;   // GPIO number or channel number (for IO expander / PCA9685)
  PinFunction   function;    // purpose
  const char*   component;   // human readable description
};

// -----------------------------
// Hardware definition: universal
// -----------------------------
struct Hardware {
  const char*       name;        // "Partner ESP32", "PCF8575", ...
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

// -----------------------------
// PCA9685 16-CH PWM PINS
// -----------------------------
const PinDef PCA9685_PINS[] = {
  // Front-left headlight RGB
  { "FL_R",  0, PINF_RGB_RED,   "Front-left headlight Red (CH0)" },
  { "FL_G",  1, PINF_RGB_GREEN, "Front-left headlight Green (CH1)" },
  { "FL_B",  2, PINF_RGB_BLUE,  "Front-left headlight Blue (CH2)" },

  // Front-right headlight RGB
  { "FR_R",  3, PINF_RGB_RED,   "Front-right headlight Red (CH3)" },
  { "FR_G",  4, PINF_RGB_GREEN, "Front-right headlight Green (CH4)" },
  { "FR_B",  5, PINF_RGB_BLUE,  "Front-right headlight Blue (CH5)" },

  // Back-left reverse light RGB
  { "BL_R",  6, PINF_RGB_RED,   "Back-left reverse light Red (CH6)" },
  { "BL_G",  7, PINF_RGB_GREEN, "Back-left reverse light Green (CH7)" },
  { "BL_B",  8, PINF_RGB_BLUE,  "Back-left reverse light Blue (CH8)" },

  // Back-right reverse light RGB
  { "BR_R",  9, PINF_RGB_RED,   "Back-right reverse light Red (CH9)" },
  { "BR_G", 10, PINF_RGB_GREEN, "Back-right reverse light Green (CH10)" },
  { "BR_B", 11, PINF_RGB_BLUE,  "Back-right reverse light Blue (CH11)" }
};

// Typical default address for PCA9685 is 0x40
const Hardware PCA9685_PWM = {
  "PCA9685 16-channel PWM",
  "RGB headlights + reverse lights",
  HW_PWM_PCA9685,
  -1,                // rxPin
  -1,                // txPin
  8,                 // sdaPin on the partner ESP
  7,                 // sclPin on the partner ESP
  0x40,              // I2C address (change if you move jumpers)
  PCA9685_PINS,
  (uint8_t)(sizeof(PCA9685_PINS) / sizeof(PCA9685_PINS[0]))
};

// -----------------------------
// PARTNER ESP32 PINS
// -----------------------------
const PinDef PARTNER_PINS[] = {
  // I2C for PCF8575 IO expander
  { "SDA_IOEXP",  9, PINF_I2C_SDA, "PCF8575 SDA" },
  { "SCL_IOEXP", 10, PINF_I2C_SCL, "PCF8575 SCL" },

  // UART connection back to master
  { "UART_RX_MASTER", 21, PINF_UART_RX, "RX from master" },
  { "UART_TX_MASTER", 20, PINF_UART_TX, "TX to master" },

  // I2C for PCA9685 16-channel PWM
  { "SDA_PCA",    8, PINF_I2C_SDA, "PCA9685 SDA" },
  { "SCL_PCA",    7, PINF_I2C_SCL, "PCA9685 SCL" },

  // Motor driver PWM inputs (to the 3 H-bridge boards)
  { "PWM_FRONT_A", 6, PINF_MOTOR_PWM, "Front driver PWMA" },
  { "PWM_FRONT_B", 5, PINF_MOTOR_PWM, "Front driver PWMB" },

  { "PWM_MID_A",   1, PINF_MOTOR_PWM, "Middle driver PWMA" },
  { "PWM_MID_B",   2, PINF_MOTOR_PWM, "Middle driver PWMB" },

  { "PWM_BACK_A",  3, PINF_MOTOR_PWM, "Back driver PWMA" },
  { "PWM_BACK_B",  4, PINF_MOTOR_PWM, "Back driver PWMB" }
};

const Hardware PARTNER_ESP32 = {
  "Partner ESP32",
  "Handles IO expander + PCA9685 and talks to master",
  HW_PARTNER_ESP,
  21,                 // rxPin (from master)
  20,                 // txPin (to master)
  8,                  // sdaPin (PCA bus as 'main' I2C, arbitrarily chosen)
  7,                  // sclPin
  0,                  // not an I2C peripheral
  PARTNER_PINS,
  (uint8_t)(sizeof(PARTNER_PINS) / sizeof(PARTNER_PINS[0]))
};

// -----------------------------
// PCF8575 IO EXPANDER PINS
// -----------------------------
const PinDef PCF8575_PINS[] = {
  // Front driver direction
  { "FRONT_AIN1",  0, PINF_MOTOR_IN,     "Front driver AIN1 (P0)" },
  { "FRONT_AIN2",  1, PINF_MOTOR_IN,     "Front driver AIN2 (P1)" },
  { "FRONT_BIN1",  2, PINF_MOTOR_IN,     "Front driver BIN1 (P2)" },
  { "FRONT_BIN2",  3, PINF_MOTOR_IN,     "Front driver BIN2 (P3)" },

  // Middle driver direction
  { "MID_AIN1",    4, PINF_MOTOR_IN,     "Middle driver AIN1 (P4)" },
  { "MID_AIN2",    5, PINF_MOTOR_IN,     "Middle driver AIN2 (P5)" },
  { "MID_BIN1",    6, PINF_MOTOR_IN,     "Middle driver BIN1 (P6)" },
  { "MID_BIN2",    7, PINF_MOTOR_IN,     "Middle driver BIN2 (P7)" },

  // Back driver direction
  { "BACK_AIN1",   8, PINF_MOTOR_IN,     "Back driver AIN1 (P8)" },
  { "BACK_AIN2",   9, PINF_MOTOR_IN,     "Back driver AIN2 (P9)" },
  { "BACK_BIN1",  10, PINF_MOTOR_IN,     "Back driver BIN1 (P10)" },
  { "BACK_BIN2",  11, PINF_MOTOR_IN,     "Back driver BIN2 (P11)" },

  // ToF shutdown lines
  { "SHUT_TOF_FL", 12, PINF_TOF_SHUTDOWN, "ToF front-left shutdown (P12)" },
  { "SHUT_TOF_FR", 13, PINF_TOF_SHUTDOWN, "ToF front-right shutdown (P13)" }, // assumption
  { "SHUT_TOF_BK", 14, PINF_TOF_SHUTDOWN, "ToF back shutdown (P14)" }
};

// On your partner ESP the PCF8575 is on SDA=9, SCL=10 and default address 0x20.
const Hardware IOEXP_PCF8575 = {
  "PCF8575 IO Expander",
  "Drives 3x motor driver direction pins + ToF SHUT",
  HW_IO_EXPANDER_PCF8575,
  -1,          // rxPin (not UART)
  -1,          // txPin
  9,           // sdaPin on the partner ESP
  10,          // sclPin on the partner ESP
  0x20,        // typical default address from your notes
  PCF8575_PINS,
  (uint8_t)(sizeof(PCF8575_PINS) / sizeof(PCF8575_PINS[0]))
};
