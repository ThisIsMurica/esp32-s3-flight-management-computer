# Third-party code and distribution notes

This project uses LVGL, Espressif display utilities, Adafruit sensor libraries, and SD-related components. Their complete dependency trees are **not vendored into this repository**. The build notes document library versions so the repository stays focused on application firmware and hardware integration.

## Included adapted porting/configuration files

- `FlightManagementComputer/esp_panel_board_custom_conf.h` — retains Espressif's **Apache-2.0** SPDX header.
- `FlightManagementComputer/lvgl_v8_port.h` and `.cpp` — retain Espressif's **CC0-1.0** SPDX header.
- `config/lv_conf.h` — LVGL configuration adapted from the original installed LVGL 8.4.0 layout.
- `FlightManagementComputer/FlightManagementComputer.ino` — application firmware and navigation/UI logic developed on top of hardware-support and vendor-porting patterns.

Third-party library license obligations still apply when dependencies are distributed. Before selecting a blanket license for the **original application source**, verify authorship of any reused portions, confirm required copyright notices, and review the applicable licenses in the official upstream sources.

## Dependency versions used for development

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

A local dependency snapshot can help reproduce older builds, but binary library archives are intentionally excluded from Git history.

## Companion firmware and data tooling

- The [Feather ESP-NOW sender sketch](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino) is included separately from the display firmware. Its Adafruit GPS, BME280, and BNO08x dependencies are **not vendored**.
- The [world NAVDATA builder](../tools/build_world_hybrid_navdata.py) and [Windows launcher](../tools/Build_World_NAVDATA.bat) are tracked with the FMC project. They draw on a previously converted **FAA CIFP** base and **OurAirports** CSV files; no original bulk data is committed.
- OurAirports states that its published data is [public domain](https://ourairports.com/data/) but provides no accuracy or suitability warranty. The [FAA CIFP](https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/) is updated every 28 days. Record data sources and cycle dates when preparing a new snapshot.
- Historical AIRAC 2608 data and the earlier v0.8.2 CDU sketch are retained outside this repository. The maintained display firmware includes the newer Direct-To, progress and ESP-NOW capabilities.
