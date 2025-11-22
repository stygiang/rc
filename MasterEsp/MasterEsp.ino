#include <Arduino.h>
#include "config/hardware_pins.h"
#include "link_master.h"
#include "hardware_master.h"
#include "rc_control.h"
#include "console_master.h"
#include <Wire.h>




void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n[Master] booting");

  linkSetup();
  hardwareSetup();
  rcControlSetup();
  consoleSetup();
  
}

void loop() {
  linkLoop();
  hardwareLoop();
  rcControlLoop();
  consoleLoop();
  //
   
  

  // TODO: feed desired motor/light/IO commands from RC input and sensors.
  // Example placeholders (commented out until ready):
  // sendMotorPwm(128, 128, 128, 128, 128, 128);
  // sendLightRgb(LIGHT_FRONT_LEFT, 64, 64, 255);
  // sendIoExpanderState(0xFFFF);
}
