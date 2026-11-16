# Third-party code and distribution notes

The original upload bundled over 2,000 files across LVGL, Espressif display utilities, Adafruit libraries and SD-related code, including demonstration assets and fonts. This GitHub preparation **does not copy those full dependency trees**. It references separately installed libraries by version instead, reducing size and making the user's firmware the focus.

## Included adapted porting/configuration files

- `FlightManagementComputer/esp_panel_board_custom_conf.h` — retains Espressif's **Apache-2.0** SPDX header.
- `FlightManagementComputer/lvgl_v8_port.h` and `.cpp` — retain Espressif's **CC0-1.0** SPDX header.
- `config/lv_conf.h` — LVGL configuration adapted from the original installed LVGL 8.4.0 layout.
- `FlightManagementComputer/FlightManagementComputer.ino` — application source from the supplied project, developed starting from vendor porting patterns. Its original contents were preserved except for renaming the `.ino` file to match the Arduino sketch folder.

Third-party library license obligations still apply when dependencies are distributed. Before selecting a blanket license for the **original application source**, verify authorship of any reused portions, confirm required copyright notices, and review the applicable licenses in the official upstream sources.

## Dependency versions in the upload

- `lvgl` 8.4.0
- `ESP32_Display_Panel` 1.0.0
- `ESP32_IO_Expander` 1.0.1
- `esp-lib-utils` 0.1.2
- `SD` 1.3.0
- `Adafruit BME280 Library` 2.3.0
- `Adafruit BMP3XX Library` 2.1.6
- `Adafruit BNO08x` 1.2.5
- `Adafruit BusIO` 1.17.4
- `Adafruit GPS Library` 1.7.5
- `Adafruit Unified Sensor` 1.1.15

The user-supplied ZIP should be preserved separately as an environment snapshot, **not committed as a single 36 MB binary repository file**.

## Additional uploaded components

- The [Feather ESP-NOW sender sketch](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino) is included separately from the display firmware. Its Adafruit GPS, BME280, and BNO08x dependencies are **not vendored**.
- The [world NAVDATA builder](../tools/build_world_hybrid_navdata.py) and [Windows launcher](../tools/Build_World_NAVDATA.bat) were preserved from the user's supplied hybrid upgrade archive. They draw on a previously converted **FAA CIFP** base and **OurAirports** CSV files; no original bulk data is committed.
- OurAirports states that its published data is [public domain](https://ourairports.com/data/) but provides no accuracy or suitability warranty. The [FAA CIFP](https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/) is updated every 28 days. Record data sources and cycle dates when preparing a new snapshot.
- The supplied 2608 dataset and legacy v0.8.2 CDU sketch remain in the original user archive, **not** in the current Git repository. The current, more feature-complete display sketch was not overwritten.
