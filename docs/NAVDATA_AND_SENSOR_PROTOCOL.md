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

Index-record size is fixed in source at **27 bytes**. The app checks file presence and reads cycle metadata from `cycle.txt` (including a `CYCLE=` entry). There is a small **built-in development database** containing sample airports, fixes, and VORs. The full SD data files and the script that generates them were **not included** in the uploaded ZIP, so this repo cannot independently reproduce the full NAVDATA functionality.

A code branch specially handles cycle `2608`; this is development logic, **not a guarantee of current AIRAC validity**. Operational data currency must not be inferred from the UI.

## ESP-NOW sensor link

The receiver expects a frame with the schematic format:

```text
@AV1,<sequence>,<uptime_ms>,<gps_online>,<gps_fix>,<satellites>,<latitude>,<longitude>,<groundspeed_kt>,<track_deg>,<gps_altitude_m>,<utc_valid>,<hour>,<minute>,<second>,<bme_online>,<pressure_hpa>,<temperature_c>,<humidity_pct>,<baro_altitude_m>,<bno_online>,<roll_deg>,<pitch_deg>,<heading_deg>*<crc16_hex>
```

This string format is documented from the receiver parser; it is **not a captured example message**. It contains 24 comma-delimited fields before the `*` CRC suffix, including `AV1` as field zero. The CRC is CRC-16/CCITT with initial value `0xFFFF` and polynomial `0x1021`, calculated over the payload **without** the leading `@` or trailing `*CRC`.

The receive callback copies the ESP-NOW packet into a 384-byte buffer; the main loop performs CRC validation and field parsing. A GPS fix is rejected if latitude falls outside [-90, 90] or longitude outside [-180, 180]. The receiver requires a compatible ESP-NOW transmitter on **channel 6**.

**Limitations:** A CRC checks accidental corruption but is not a cryptographic integrity check or sender authentication. The Feather sender code is not included; transmitter implementation, packet transmission rate and pairing details must be documented separately.
