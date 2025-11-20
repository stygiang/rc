#pragma once

#include <Arduino.h>

enum MessageType : uint8_t {
  MSG_PING = 0x01,
  MSG_PONG = 0x02,
  MSG_TELEMETRY = 0x03,       // partner -> master
  MSG_SET_MOTOR_PWM = 0x10,   // master -> partner
  MSG_SET_LIGHT_RGB = 0x11,   // master -> partner
  MSG_SET_IOEXP_STATE = 0x12  // master -> partner
};

enum LightTarget : uint8_t {
  LIGHT_FRONT_LEFT = 0,
  LIGHT_FRONT_RIGHT = 1,
  LIGHT_BACK_LEFT = 2,
  LIGHT_BACK_RIGHT = 3
};

void linkSetup();
void linkLoop();
