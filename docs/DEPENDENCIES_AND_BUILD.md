# Dependencies and build notes

The table below records the library versions used in the development sketchbook. It is a **dependency reference**, not a fully pinned or independently reproduced build environment. The exact Arduino IDE/ESP32 core version and board-menu settings are still to be documented.

## Development library versions

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

Only LVGL, ESP32_Display_Panel, ESP32_IO_Expander, esp-lib-utils and an appropriate SD implementation are **directly relevant** to building the display sketch, plus the Espressif ESP32 Arduino core (Arduino.h, WiFi, ESP-NOW, and SPI). The Adafruit sensor libraries support the **companion Feather firmware**. Its source directly uses the Adafruit GPS, BME280 and BNO08x libraries. The exact transitive dependencies can vary with library version.

## Arduino IDE setup

1. Install the Espressif **ESP32 boards** package. The code uses `esp_now_recv_info_t`, so a package with the modern ESP-NOW receive-callback API is required (typically ESP32 Arduino core 3.x). **The exact previously working version is unknown.**
2. Install the library versions listed above through Arduino Library Manager or their official repositories. Use a clean sketchbook or resolve duplicate libraries when they conflict.
3. Keep the `FlightManagementComputer` sketch folder and its four source files together. The main source file name matches the folder name so Arduino IDE can open it as a sketch.
4. Put this repository's `config/lv_conf.h` into the parent directory of `lvgl/` as `libraries/lv_conf.h`, matching the development sketchbook layout. The configuration has `LV_COLOR_DEPTH 16`.
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

## Reproducibility status

- Arduino IDE / Arduino CLI version, ESP32 board-core version and menu selections
- A recorded clean build with the versions and board settings listed above
- A complete, current SD `/NAVDATA` directory: the historical AIRAC 2608 snapshot is intentionally not committed
- The **FAA CIFP-to-index conversion tool** used to create `Mini_FMC_NAVDATA_260806.zip`; the included worldwide overlay builder requires a preconverted base package

The current source tree includes both display and sensor firmware. Hardware photos document on-device interface demonstrations, while a reproducible clean-build record, a measured ESP-NOW link test, and calibrated sensor results remain in the [verification plan](TEST_PLAN.md).

## Companion firmware and data builder

- **Feather firmware:** [Arduino sketch](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino) and [setup notes](FEATHER_SENSOR_COMPUTER.md). Select the appropriate ESP32-S2 Feather target; confirm that the transmitter sketch's `Wire.begin()` configuration and default sensor addresses match the actual board.
- **Historical world NAVDATA overlay builder:** [Python 3 script](../tools/build_world_hybrid_navdata.py) and [Windows batch launcher](../tools/Build_World_NAVDATA.bat). The Python script uses only the standard library. It requires an **already converted** FAA NAVDATA ZIP and downloads OurAirports CSVs unless you specify local versions. See [data generation notes](WORLD_NAVDATA.md).
