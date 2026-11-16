# Dependencies and build notes

This document records the **versions found in the uploaded `Waveshare43Clean` ZIP**, not a newly tested toolchain lockfile. Exact Arduino IDE/ESP32 board-core versions and board-menu settings were absent from the archive. To reproduce the user's build, keep the original local environment/version details once known.

## Versions recorded in the supplied `libraries/`

| Component | Version observed | Why it matters |
| --- | --- | --- |
| LVGL (`lvgl`) | **8.4.0** | Touchscreen UI uses LVGL 8 APIs; do not substitute 9.x |
| ESP32_Display_Panel | **1.0.0** | ESP panel board, display, touch, expander integration |
| ESP32_IO_Expander | **1.0.1** | Board expander control |
| esp-lib-utils | **0.1.2** | Utility layer used by display-panel libraries |
| SD | **1.3.0** (bundled) | SD interface; confirm which installed library is selected to avoid ESP32 core `SD` collisions |
| Adafruit BME280 Library | 2.3.0 | Bundled in original ZIP; no direct use in the display sketch |
| Adafruit BMP3XX Library | 2.1.6 | Bundled, likely for external sensor work |
| Adafruit BNO08x | 1.2.5 | Bundled, likely for external sensor work |
| Adafruit GPS Library | 1.7.5 | Bundled, likely for external sensor work |
| Adafruit BusIO | 1.17.4 | Bundled supporting library |
| Adafruit Unified Sensor | 1.1.15 | Bundled supporting library |

Only LVGL, ESP32_Display_Panel, ESP32_IO_Expander, esp-lib-utils and an appropriate SD implementation are **directly relevant** to building the display sketch, plus the Espressif ESP32 Arduino core (Arduino.h, WiFi, ESP-NOW, and SPI). The other bundled Adafruit libraries may belong to the **separate Feather firmware**, which was not supplied. The exact transitive dependencies can vary with library version.

## Arduino IDE setup

1. Install the Espressif **ESP32 boards** package. The code uses `esp_now_recv_info_t`, so a package with the modern ESP-NOW receive-callback API is required (typically ESP32 Arduino core 3.x). **The exact previously working version is unknown.**
2. Install the library versions listed above through Arduino Library Manager or their official repositories. Use a clean sketchbook or resolve duplicate libraries when they conflict.
3. Keep the `FlightManagementComputer` sketch folder and its four source files together. The main source file name matches the folder name so Arduino IDE can open it as a sketch.
4. Put this repository's `config/lv_conf.h` into the parent directory of `lvgl/` as `libraries/lv_conf.h`, matching the original ZIP's layout. The configuration has `LV_COLOR_DEPTH 16`.
5. Build for the **Waveshare ESP32-S3-Touch-LCD-4.3** using the board parameters that match the hardware. Document the flash/PSRAM, partition, and USB settings that compile and boot successfully.
6. With the board powered, open Serial Monitor at `115200`. Look for the `ESP-NOW SENSOR LINK` and `SD NAVDATA ONLINE` messages or error statuses.

## Example sketchbook layout

```text
Arduino/
  libraries/
    lv_conf.h                 # copied from this repo's config/
    lvgl/                     # v8.4.0
    ESP32_Display_Panel/      # v1.0.0
    ESP32_IO_Expander/        # v1.0.1
    esp-lib-utils/            # v0.1.2
  # ...other installed libraries as required
```

## Not included and not asserted

- Arduino IDE / Arduino CLI version, ESP32 board-core version and menu selections
- Known-good compiler output and successful reproduction from this source-only package
- Feather sensor-computer sender firmware
- SD `/NAVDATA` directory and data generator

The source-only package was checked for file integrity and credential-like literals, **not compiled**: an Arduino toolchain and the physical device were not available in this environment.
