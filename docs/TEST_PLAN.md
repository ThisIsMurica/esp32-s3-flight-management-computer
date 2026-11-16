# Hardware and functional verification plan

**All hardware and integration test items below are planned, not passed.** Publishing the supplied Feather firmware and world NAVDATA builder does not establish a successful build or on-device operation. No hardware measurements, serial logs, or photos/videos were supplied for this pass.

| Area | Proposed acceptance check | Result |
| --- | --- | --- |
| Arduino build | Build with pinned ESP32/LVGL/display dependencies | Not run |
| LCD | 800 × 480 graphics, consistent colors, no blue tint | Not run |
| Touch | Touch coordinates and CDU keys correspond to intended UI elements | Not run |
| Pages | MENU, IDENT, POS INIT, DEP/ARR, ROUTE, LEGS, DIRECT TO, PROGRESS, STATUS render correctly | Not run |
| Scratchpad | Characters, delete, transfer, and error messaging work | Not run |
| Route edits | Insert/delete legs, discontinuities, and EXEC state behave as expected | Not run |
| SD initialization | Card mounts with CH422G EXIO4 CS and GPIO6 dummy CS without corrupting LCD | Not run |
| Navdata | Missing/valid indexed NAVDATA cases display appropriate status | Not run |
| Feather build | Arduino IDE builds the Adafruit ESP32-S2 Feather sensor sketch with GPS/BME280/BNO085 libraries | Not run |
| ESP-NOW | Receiver accepts properly formed packet and rejects bad CRC | Not run |
| Feather-to-FMC | Channel 6, AV1 format, 10 Hz transmit setting, link timeout, and reliable delivery | Not run |
| BME/BNO/GPS | Sensor measurements, update rates, calibration, and coordinate conventions | Not run |
| Sensor timeout | Data goes stale after configured 1500 ms timeout | Not run |
| GPS | Position, course, GS, ETE/ETA react sensibly to valid GPS updates | Not run |
| Reliability | Power cycling, missing GPS, missing SD, and intermittent wireless link | Not run |

## Suggested evidence for future commits

- Board model and ESP32 Arduino core version
- Arduino IDE menu settings, flash/PSRAM configuration, library version output
- Screenshot/photo of CDU pages and wired hardware
- Serial logs demonstrating SD mount and live ESP-NOW sensor packets (redact any personal or identifying data)
- Test vectors for navigation math and CRC parser

Do not mark these items complete without evidence from the actual hardware/software build.

## Offline checks completed on the supplied historical archive

These checks used the uploaded `Mini_FMC_WORLD_UPGRADE_ALL_IN_ONE_1.zip`, **not on-device execution**:

- 21/21 SHA-256 entries in `NAVDATA/SHA256SUMS.txt` matched the contained files.
- All five `.idx` files had lengths divisible by the specified 27-byte record size.
- Indexed airport entries for `KPHL`, `GOBD`, `EGLL`, and `RJTT` were found and pointed to readable records.
- Sensor transmitter and FMC receiver sources use the same AV1 field count (24), CRC-16/CCITT-FALSE parameters and ESP-NOW channel 6.
- The published original Python data builder's Git blob checksum matched the uploaded script (source integrity), not a successful execution.

**Not established by these checks:** current NAVDATA validity, geographic/procedure accuracy, firmware compilation, physical sensor reliability, packet receipt rate, or end-to-end navigation functionality. The archived data cycle 2608 expired September 3, 2026.
