# Hardware and functional verification plan

**All test items below are planned, not passed.** This source-only repository preparation did not compile or execute firmware on the Waveshare board, and no photos/videos or serial logs were supplied for this pass.

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
| ESP-NOW | Receiver accepts properly formed packet and rejects bad CRC | Not run |
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
