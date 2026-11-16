# ESP32-S3 Flight Management Computer

**A touchscreen flight-management / control-display-unit (CDU) prototype built around the Waveshare ESP32-S3-Touch-LCD-4.3.**

This is an ongoing embedded-systems project exploring avionics-style navigation workflows, real-time sensor integration, and human–machine interfaces. The main firmware implements a CDU-like UI with route and procedure pages, navigation-data lookup, Direct-To functionality, and a progress display. It combines **C++/Arduino**, **LVGL**, **SD-card data access**, and **ESP-NOW** communications.

> **Prototype / educational use only.** This is not certified avionics, does not contain a validated operational navigation database, and must not be used for real flight planning, guidance, or navigation.

## Features present in the source

- **CDU touch interface:** MENU, IDENT, POS INIT, INDEX, DEP/ARR, ROUTE, LEGS, DIRECT TO, PROGRESS, LAT/LON, NAV DATA, and STATUS pages.
- **Route editing:** origin and destination, waypoint entry, leg insertion/deletion, route modification and execution state, and discontinuities.
- **Departure/arrival workflows:** runway, SID/STAR, transition, and approach selection where records exist in the navigation data.
- **Direct-To and progress:** target selection, distance-to-go, desired track, and GPS-dependent ground speed, estimated time enroute, and ETA calculations.
- **SD-backed navdata:** indexed airport/waypoint/procedure/runway records and AIRAC-cycle metadata, with a small built-in **test** database for development.
- **Live sensor interface:** receives ESP-NOW packets from a separate Feather sensor computer carrying GPS position/UTC, barometric data, and orientation values. The receiver validates packet CRC-16/CCITT and tracks link health.
- **Display:** 800 × 480 RGB LCD with GT911 touch, LVGL 8-based UI, and anti-tearing/display-buffer configuration.

**Status:** The main display firmware and companion Feather sensor-transmitter source are now published. The hybrid NAVDATA builder is included, but the large generated data files are not. A clean build and on-device end-to-end test have **not** been completed as part of this repository preparation.

## Hardware and architecture

- **Display MCU:** Waveshare ESP32-S3-Touch-LCD-4.3, 800 × 480 ST7262 RGB LCD, GT911 I²C touch controller.
- **Navigation storage:** TF/microSD interface with custom chip-select handling through a CH422G I/O expander.
- **Sensor source:** separate Feather board (exact model and transmitter source are not in this repository) communicating via ESP-NOW on channel 6.

See [Hardware & wiring](docs/HARDWARE.md) and [Architecture](docs/ARCHITECTURE.md) for implementation details grounded in the firmware.

## Repository structure

```text
FlightManagementComputer/
  FlightManagementComputer.ino       Main firmware, route logic, and LVGL pages
  esp_panel_board_custom_conf.h     Waveshare RGB LCD, touch, and expander setup
  lvgl_v8_port.cpp                   LVGL display/touch FreeRTOS port
  lvgl_v8_port.h                     LVGL port settings
SensorComputer/
  Feather_Sensor_Computer_ESPNOW_v1/
    Feather_Sensor_Computer_ESPNOW_v1.ino   ESP32-S2 transmitter, GPS, BME280, BNO085
config/
  lv_conf.h                         LVGL 8.4 configuration from the original setup
tools/
  build_world_hybrid_navdata.py         FAA + OurAirports overlay generator
  Build_World_NAVDATA.bat               Windows historical-data build launcher
tests/
  test_navdata_builder.py              Offline index and overlay tests
docs/
  FEATHER_SENSOR_COMPUTER.md            Sensor hardware, protocol, and setup
  WORLD_NAVDATA.md                      Data provenance and generation workflow
  ARCHITECTURE.md                   Data and UI architecture
  DEPENDENCIES_AND_BUILD.md         Library versions and build checklist
  HARDWARE.md                       Pin definitions and connected subsystems
  NAVDATA_AND_SENSOR_PROTOCOL.md    Required external data and link format
  TEST_PLAN.md                      Verification plan and unverified assumptions
  THIRD_PARTY.md                    Third-party component attribution
```

## Getting started

1. Install the **Espressif ESP32 Arduino board support** and select a compatible ESP32-S3 board profile. The exact board-core version and Arduino Tools menu settings used for the supplied firmware were **not** included in the archive.
2. Install the dependencies and match the library versions recorded in [Dependencies and build](docs/DEPENDENCIES_AND_BUILD.md). In particular, use **LVGL 8.4.0**, not LVGL 9.
3. Copy `config/lv_conf.h` into the parent of the `lvgl` library folder (as it was in the original archive), or configure the compiler's LVGL config include path equivalently.
4. Open `FlightManagementComputer/FlightManagementComputer.ino` in Arduino IDE. Keep all four source files in the same sketch folder.
5. Select suitable ESP32-S3 flash/PSRAM and board options for the Waveshare hardware, build, and upload. Record the working options after verifying them on the device.
6. Open Serial Monitor at **115200 baud** to inspect ESP-NOW and SD navigation-data status.
7. To run the **Feather sensor computer**, use [its dedicated setup guide](docs/FEATHER_SENSOR_COMPUTER.md); match channel 6 on both devices.
8. To prepare **world NAVDATA**, review [the hybrid data-generation guide](docs/WORLD_NAVDATA.md). The original uploaded dataset is a **historical, expired 2608 snapshot**; do not assume it is current.

The supplied archive included a large local `libraries/` directory. This GitHub version deliberately **does not vendor those third-party libraries**. Install them separately; see the version manifest in the build documentation. The original archive remains the reference if a future dependency update causes a regression.

## Offline navigation-data tests

The worldwide-data builder has three standard-library unit tests for record indexing and overlay behavior. From the repository root:

```bash
python -m unittest discover -s tests -v
```

The original builder was also run offline against the supplied historic processed FAA package plus **small synthetic** airport/runway/navaid CSV fixtures; it produced a ZIP and resolved sample GOBD, EGLL and KPHL airport entries. These checks **do not** validate live upstream downloads, the complete worldwide database's correctness, or any hardware-dependent feature. See the [test plan](docs/TEST_PLAN.md).

## Important limitations

- The full `/NAVDATA` SD-card dataset is **not checked in**. The uploaded 2608 hybrid dataset was audited offline, but expired September 3, 2026; see [world NAVDATA](docs/WORLD_NAVDATA.md). The built-in list of airports, VORs, and fixes remains small, hard-coded **test data**.
- The **Feather sensor transmitter** source is [included](SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino), but live values still require compatible wired sensors, board setup, and an on-device radio test.
- GPS-derived calculations depend on valid sensor input; runtime behavior was not independently verified in this repository-preparation pass.
- ESP-NOW packet CRC is an error-detection mechanism, **not authentication or encryption**. Do not treat received packets as trustworthy for safety-critical purposes.
- The bundled porting files are adapted from Espressif examples and retain their existing SPDX notices. See [Third-party acknowledgements](docs/THIRD_PARTY.md).

## Planned improvements

- Add actual hardware photos/screenshots and a wiring/block diagram based on the assembled device.
- Capture a repeatable dual-board test showing valid AV1 packets, CRC checks, GPS data, and Feather-to-Waveshare link performance.
- Rebuild and validate an up-to-date SD navigation database using a separately prepared FAA base package and documented source snapshots.
- Extract navigation math and parsing into independently testable modules and add unit tests.
- Capture reproducible board/PSRAM/build options and complete the [test plan](docs/TEST_PLAN.md).

## License and attribution

No blanket open-source license is assigned to this combined repository at this stage. The Espressif-derived port files preserve their own license notices and third-party dependencies have their own licenses; see [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md). Decide on a license for the original application code before inviting reuse.
