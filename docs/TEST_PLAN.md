# Verification record and test plan

This document tracks the difference between **prototype features demonstrated on physical hardware**, offline software checks, and tests that still need repeatable measurements. Photographs are useful engineering evidence, but they do not replace serial logs, test vectors, or a documented build.

## Prototype demonstrations

The [project photo gallery](../README.md#prototype-photographs) includes the Waveshare display running LAT/LON NAV, PROGRESS and STATUS pages, along with images of the separate Feather sensor hardware. The STATUS screen reports display/touch, SD, nav database, and sensor-computer states at capture time; it also shows a **GPS no-fix** condition. These images document a functioning prototype interface and do not by themselves establish continuous link reliability or GPS accuracy.

## Verification matrix

| Subsystem | Acceptance check | Evidence/status |
| --- | --- | --- |
| Arduino build | Rebuild both sketches with pinned ESP32 cores and library versions | Reproducible clean-build logs pending |
| LCD | Verify 800 × 480 output, colors, refresh and sustained display operation | UI running in photographs; quantitative display test pending |
| Touch | Confirm correct touch coordinates for controls and scratchpad | Full interaction regression pending |
| CDU pages | Exercise MENU, IDENT, POS INIT, DEP/ARR, ROUTE, LEGS, DIRECT TO, PROGRESS and STATUS | LAT/LON, STATUS and PROGRESS shown in photographs; full page sweep pending |
| Route editing | Insert/remove legs, manage discontinuities and verify EXEC state | Repeatable test cases pending |
| microSD | Confirm CH422G selection and correct indexed data reads without LCD interference | STATUS photo reports SD mounted and NAVDATA online; log and error-path tests pending |
| Feather sensor computer | Flash transmitter and check I²C devices, GPS, environmental readings and orientation | Hardware shown in photographs; calibration and build records pending |
| ESP-NOW | Verify AV1 parsing, CRC error rejection, receive rate and 1500 ms link timeout | Sender/receiver source protocols agree; measurement logs pending |
| Navigation math | Validate desired track, distance, ETE/ETA and coordinate handling against reference values | Test vectors pending |
| Resilience | Restart, no GPS fix, no SD, wireless interruption and power cycling | Regression results pending |

## Automated and offline checks

The navigation-data builder was tested separately from the embedded hardware:

- **3 Python unit tests passed:** fixed-record index round trip, airport overlay, and runway overlay behavior.
- **21/21 SHA-256 manifest entries matched** in the historical NAVDATA snapshot.
- All five `.idx` files have lengths divisible by the **27-byte** record size.
- Indexed records for **KPHL, GOBD, EGLL and RJTT** were found in the snapshot.
- The builder completed an offline integration run using the historical processed FAA base plus **small synthetic** OurAirports-format CSV fixtures. It produced a ZIP with sample airport lookups.
- The Feather transmitter and Waveshare receiver implement the same AV1 field count, CRC-16/CCITT parameters, and ESP-NOW channel 6.

Run the project unit tests from the repository root:

```bash
python -m unittest discover -s tests -v
```

These checks address code contracts and historical data-file integrity; they are **not** a complete test of aeronautical-data accuracy, live data retrieval, firmware compilation or the installed hardware. Historical AIRAC 2608 data expired in September 2026.

## Evidence to collect next

- Exact Arduino IDE/CLI, ESP32 core, flash/PSRAM settings and library versions, with clean-build output.
- Serial logs from both boards showing AV1 frame counts, CRC failures, receiver timeouts and GPS fix status.
- Repeatable test vectors for Direct-To, great-circle distance and desired-track calculations.
- Recorded sensor readings and comparison to references for location, pressure and attitude.
- A short demonstration of the full startup, route entry and sensor-link workflow.

Results will be added to this document as the validation work progresses.
