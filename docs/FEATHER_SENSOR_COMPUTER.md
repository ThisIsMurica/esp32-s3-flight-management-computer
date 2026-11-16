# Feather ESP-NOW sensor computer

Source: [`SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino`](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino)

## Sensor-computer hardware

| Component | Connection or role |
| --- | --- |
| Adafruit ESP32-S2 Feather with onboard BME280 | Sensor processor and ESP-NOW transmitter |
| PA1010D GPS | I²C address `0x10`, NMEA RMC/GGA at 1 Hz |
| BME280 | I²C address `0x77`; pressure, temperature, humidity |
| BNO085 | I²C address `0x4A` or fallback `0x4B`; rotation vector |
| Waveshare ESP32-S3 FMC | ESP-NOW broadcast receiver, fixed Wi-Fi channel 6 |

The code configures `Wire.begin()` and supplies I²C power where `PIN_I2C_POWER` exists. Board pin mapping and physical wiring should be checked against the actual Feather revision.

## Firmware behavior

- The Feather formats `@AV1,...*XXXX\r\n` messages with **24 comma-separated fields** (including the `AV1` prefix) and **CRC-16/CCITT-FALSE** (initial value `0xFFFF`, polynomial `0x1021`).
- ESP-NOW sends are attempted every **100 ms** (10 Hz), on **channel 6**, to broadcast MAC `FF:FF:FF:FF:FF:FF`.
- BME280 values are updated every **500 ms** (2 Hz); BNO085 rotation-vector reports are requested every **50 ms** (20 Hz); the GPS is configured for **1 Hz**.
- GPS fix data is marked unavailable if valid NMEA messages are not parsed within **3 seconds**.
- Quaternion orientation is converted into Euler roll, pitch, and yaw in degrees.
- The transmitter prints sensor and link counters to the USB serial monitor at **115200 baud**.

**Firmware interface:** Both the Feather transmitter and Waveshare receiver use the same `AV1` version, 24-field packet layout, CRC-16/CCITT parameters, and channel 6. Reproducible over-the-air delivery and sensor-accuracy results are future validation milestones.

## Build and flash

1. Install the **Espressif ESP32 Arduino board package** and select the appropriate **Adafruit ESP32-S2 Feather** board profile for the physical hardware.
2. Install the **Adafruit GPS Library**, **Adafruit BME280 Library**, **Adafruit BNO08x**, and their transitive dependencies.
3. Open the `.ino` file from its matching-named sketch directory in Arduino IDE.
4. Check I²C wiring, sensor addresses, Feather board revision, and power supply.
5. Build and upload to the Feather; open Serial Monitor at **115200**.
6. Power the FMC, verify the same Wi-Fi channel is configured, and check valid ESP-NOW packet count and GPS/sensor status.

The development-library versions are recorded in [dependencies and build notes](DEPENDENCIES_AND_BUILD.md). A pinned Feather board-core and library combination, together with a clean-build log, is still to be documented.

## Limitations and security

- ESP-NOW **broadcast is unencrypted and unauthenticated**. CRC detects accidental data corruption but does **not** authenticate the sender. Use only in the prototype/test environment.
- The serial `TX queued/fail` counter reflects `esp_now_send()` return status; it is **not** confirmation that the receiver got the packet.
- BME280 altitude uses a fixed **1013.25 hPa** reference, not a verified local altimeter setting.
- Planned bench tests cover packet receipt, GPS accuracy, BNO orientation conventions, and actual over-the-air delivery rate relative to the configured 10 Hz transmit interval.
- **Educational prototype only. Not certified aviation instrumentation.**
