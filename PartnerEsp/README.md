# Partner ESP sketch layout

Arduino-friendly folder structure to hold the partner controller code for the RC car. Fill in your logic before compiling.

- `PartnerEsp.ino` — main sketch tab; define `setup()` and `loop()` here.
- `config/` — pin maps, Wi-Fi credentials, and other settings you do not want hard-coded (see `config/hardware_pins.h` for partner layout with PCA9685 + PCF8575).
- `lib/` — local libraries shared across sketches or reused modules.
- `data/` — files to upload to SPIFFS/LittleFS (if used).
- `docs/` — notes, wiring diagrams, and other project documentation.

The folder name already matches the `.ino` filename so the Arduino IDE/CLI can open and build it once you add the required functions.
