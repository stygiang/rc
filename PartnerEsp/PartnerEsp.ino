#include <Arduino.h>
#include "config/hardware_pins.h"
#include "link_partner.h"
#include "hardware_partner.h"

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n[Partner] booting");

  linkSetup();
  partnerHardwareSetup();
}

void loop() {
  linkLoop();
}
