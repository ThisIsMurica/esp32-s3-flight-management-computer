# Hybrid FAA + worldwide navigation data

This repository includes the **original source scripts** supplied with the `Mini_FMC_WORLD_UPGRADE_ALL_IN_ONE` package:

- [`tools/build_world_hybrid_navdata.py`](../tools/build_world_hybrid_navdata.py): merges a **previously converted FAA FMC NAVDATA ZIP** with OurAirports CSV overlays
- [`tools/Build_World_NAVDATA.bat`](../tools/Build_World_NAVDATA.bat): Windows launcher for the original 2608 package

The uploaded source bundle also contained an older `Mini_Avionics_CDU_v08_2_WORLD_NAVDATA.ino` (about 4,790 lines). **Do not replace** the current `FlightManagementComputer.ino` (about 7,375 lines) with it: the current firmware includes additional Direct-To, progress, and ESP-NOW functions absent from the older sketch.

## Coverage and provenance

The historic package combines:

| Layer | Source | Coverage |
| --- | --- | --- |
| FAA base | Processed FAA CIFP / ARINC 424-18 NAVDATA | U.S. airports, runway data, fixes, procedures (SID/STAR/approach), and airways |
| Worldwide overlay | OurAirports `airports.csv`, `runways.csv`, `navaids.csv` | Additional non-closed airports, runway ends with published coordinates, and radio navaids outside FAA coverage |

**Limitations:** Outside FAA coverage, this **does not** provide a complete global set of fixes, airways, SIDs, STARs, approaches, or constraints. Availability and accuracy vary. No output should be used for actual flight planning or navigation.

- FAA CIFP downloads: https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/
- OurAirports datasets and public-domain terms: https://ourairports.com/data/

## Historical snapshot examined

The supplied `NAVDATA` directory identifies **AIRAC 2608**, effective **2026-08-06 09:01 UTC through 2026-09-03 09:01 UTC**. This cycle **is expired**; its package metadata reports the following overlay additions:

| Overlay record type | Reported added records |
| --- | ---: |
| Airports | 73,816 |
| Runway ends | 18,004 |
| Radio navaids | 8,184 |

Offline inspection of the supplied archive confirmed:
- **21 of 21** SHA-256 manifest entries match the contained data files.
- All five `.idx` files have lengths that are multiples of the **27-byte** index-record size.
- The airport index resolves **KPHL, GOBD, EGLL, and RJTT**.

These are **archive-integrity and lookup checks only**, not validation of individual aeronautical records or real-time currency.

## Build a new hybrid package

**Important:** The builder requires an *already processed* `Mini_FMC_NAVDATA_*.zip` containing `NAVDATA/airports.dat`, `airports.idx`, `points.dat`, `points.idx`, `runways.dat`, and `runways.idx`. It **does not parse a raw FAA CIFP download into these files**. The previously processed FAA base ZIP was provided in the uploaded bundle but is intentionally not committed here.

For a reproducible historical run on Windows:

1. Copy the supplied historical `Mini_FMC_NAVDATA_260806.zip` alongside the two scripts in `tools/`.
2. Run `Build_World_NAVDATA.bat`, or from that directory use:

   ```powershell
   py -3 build_world_hybrid_navdata.py --faa-package Mini_FMC_NAVDATA_260806.zip --output Mini_FMC_NAVDATA_260806_WORLD.zip
   ```

3. The script downloads the current OurAirports CSV files unless you supply `--airports-csv`, `--runways-csv`, and `--navaids-csv`. For reproducible results, save and supply a specific set of CSV snapshots.
4. Inspect the resulting `NAVDATA/meta.json`, `cycle.txt`, checksums, and sample lookups. Confirm the desired coverage and test with the FMC on the bench.
5. Copy the generated `NAVDATA/` folder to the microSD **root**, after backing up the existing card.

**Potential data loss from this builder:** The original script **deletes an existing directory with the same basename as the output ZIP**, as well as replacing the output ZIP. Use a fresh empty output destination and back up any previous generated data.

For a **newer FAA cycle**, first prepare a compatible processed base NAVDATA package, then pass it with `--faa-package` and change the output filename. Do not assume that merely rebuilding the worldwide overlay refreshes FAA procedures or that the historical 2608 firmware logic has been validated for every future cycle.

## Repository data policy

The large generated `.dat`/`.idx` files and historical ZIPs are excluded from Git. This keeps the code repository focused on original firmware, documented data-generation logic, and reproducible verification. External data versions and licenses should be reviewed before redistributing snapshots.

The original data was checked offline, but the builder has **not been rerun against live sources**, and the complete system has **not been rebuilt or hardware-tested** with the resulting NAVDATA.
