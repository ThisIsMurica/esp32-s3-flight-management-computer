# Hardware interfaces

These pin assignments are defined by the firmware and board configuration. They are a reference for reproducing the hardware setup; continuity measurements and board-revision-specific wiring should be verified before modifications.

## LCD and touch

- Board profile configured as `Waveshare:ESP32-S3-Touch-LCD-4.3`.
- 800 × 480 ST7262 RGB LCD with a 16-bit RGB interface.
- GT911 touch controller using I²C; config names **SDA GPIO8, SCL GPIO9**.
- CH422G I/O expander on the same configured I²C bus (address `0x20`).
- LVGL color depth **16-bit**, with a display anti-tearing mode enabled.

## microSD / TF card

| Signal | Value in code | Comment |
| --- | --- | --- |
| SPI MOSI | GPIO11 | `SD_SPI_MOSI` |
| SPI SCK | GPIO12 | `SD_SPI_SCK` |
| SPI MISO | GPIO13 | `SD_SPI_MISO` |
| Real SD chip select | CH422G **EXIO4** | Active-low, set via expander |
| SD library dummy CS | GPIO6 | Selected to avoid GPIO10 |

**Important board-specific detail:** GPIO10 is an LCD blue-data line on this module. Using the default ESP32-S3 `SD.begin()` chip-select on GPIO10 can disrupt the display. This firmware explicitly uses GPIO6 as a dummy CS for the Arduino SD API and drives the actual SD select through the expander. Do not 'simplify' this wiring without verifying the hardware.

## Sensor / wireless input

- ESP-NOW receiver enabled in Wi-Fi station mode.
- Channel fixed at **6** (`SENSOR_ESPNOW_CHANNEL`).
- Packet timeout is **1500 ms** and sensor-dependent pages have a **1000 ms** refresh setting.
- Source data is transmitted by the **Adafruit ESP32-S2 Feather** sensor computer; the companion [firmware](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino) and [sensor setup guide](FEATHER_SENSOR_COMPUTER.md) are now in this repository. The code identifies PA1010D GPS (I²C `0x10`), onboard BME280 (`0x77`) and BNO085 (`0x4A`/`0x4B`); match wiring and power to the exact assembled board revision.

See [navdata and sensor protocol](NAVDATA_AND_SENSOR_PROTOCOL.md) for the receiver's expected packet fields.
