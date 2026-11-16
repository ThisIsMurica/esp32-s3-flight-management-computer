# Navigation-data and sensor input contracts

## SD navigation-data layout

The source expects these files in the card's `/NAVDATA/` directory:

```text
/NAVDATA/
  airports.idx
  airports.dat
  points.idx
  points.dat
  procedures.idx
  procedures.dat
  runways.idx
  runways.dat
  cycle.txt
```

Index-record size is fixed in source at **27 bytes**. The app checks file presence and reads cycle metadata from `cycle.txt` (including a `CYCLE=` entry). There is a small **built-in development database** containing sample airports, fixes, and VORs. The generated SD files are **not committed**, but the [world-data overlay builder](../tools/build_world_hybrid_navdata.py) and [Windows launcher](../tools/Build_World_NAVDATA.bat) are now published. The builder still requires a **previously converted FAA NAVDATA ZIP**; it cannot generate one from raw FAA CIFP. See [world data guide](WORLD_NAVDATA.md) for source provenance, steps and validation.

A code branch specially handles cycle `2608`; this is development logic, **not a guarantee of current AIRAC validity**. The historical 2608 dataset was effective August 6 through September 3, 2026 and is **expired**. Operational data currency must not be inferred from the UI.

## ESP-NOW sensor link

The receiver expects a frame with the schematic format:

```text
@AV1,<sequence>,<uptime_ms>,<gps_online>,<gps_fix>,<satellites>,<latitude>,<longitude>,<groundspeed_kt>,<track_deg>,<gps_altitude_m>,<utc_valid>,<hour>,<minute>,<second>,<bme_online>,<pressure_hpa>,<temperature_c>,<humidity_pct>,<baro_altitude_m>,<bno_online>,<roll_deg>,<pitch_deg>,<heading_deg>*<crc16_hex>
```

This string format is documented from the receiver parser; it is **not a captured example message**. It contains 24 comma-delimited fields before the `*` CRC suffix, including `AV1` as field zero. The CRC is CRC-16/CCITT with initial value `0xFFFF` and polynomial `0x1021`, calculated over the payload **without** the leading `@` or trailing `*CRC`.

The receive callback copies the ESP-NOW packet into a 384-byte buffer; the main loop performs CRC validation and field parsing. A GPS fix is rejected if latitude falls outside [-90, 90] or longitude outside [-180, 180]. The receiver requires a compatible ESP-NOW transmitter on **channel 6**.

**Transmitter source:** [Feather sensor-computer firmware](../SensorComputer/Feather_Sensor_Computer_ESPNOW_v1/Feather_Sensor_Computer_ESPNOW_v1.ino), with its [setup and interface notes](FEATHER_SENSOR_COMPUTER.md). The two firmware components define matching AV1 fields, CRC-16/CCITT-FALSE parameters, and channel 6. The transmitter attempts a broadcast every 100 ms (10 Hz); measured delivery and sensor-accuracy results remain part of the validation plan.

**Limitations:** CRC checks accidental corruption, not cryptographic integrity or sender authentication. The sender uses unencrypted broadcast; avoid safety-critical reliance and validate the packet link in a controlled test environment.
