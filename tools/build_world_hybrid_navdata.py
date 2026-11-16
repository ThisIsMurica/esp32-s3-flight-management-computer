#!/usr/bin/env python3
"""Build a hybrid Mini Avionics NAVDATA package.

Base layer:
  Existing Mini_FMC_NAVDATA_*.zip produced from FAA CIFP.

World overlay:
  OurAirports public-domain airports.csv, runways.csv, and navaids.csv.

Result:
  * FAA data remains authoritative where an identifier already exists.
  * Worldwide non-closed airports are added to airports.dat/airports.idx.
  * Worldwide runway ends are added for overlay airports when endpoint data exists.
  * Worldwide radio navaids outside FAA coverage are added to points.dat/points.idx.
  * U.S. SIDs, STARs, approaches, airways, and fixes remain from FAA CIFP.

No third-party Python packages are required.
Development use only; not approved for real-world navigation.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import ssl
import sys
import tempfile
import unicodedata
import urllib.request
import zipfile
from datetime import datetime, timezone
from pathlib import Path

INDEX_RECORD_SIZE = 27

URLS = {
    "airports": "https://davidmegginson.github.io/ourairports-data/airports.csv",
    "runways": "https://davidmegginson.github.io/ourairports-data/runways.csv",
    "navaids": "https://davidmegginson.github.io/ourairports-data/navaids.csv",
}

# FAA CIFP coverage we keep authoritative for radio navaids.
FAA_COUNTRY_CODES = {"US", "PR", "GU", "VI", "AS", "MP", "UM"}


def ascii_clean(value: object, max_len: int | None = None) -> str:
    text = "" if value is None else str(value)
    text = text.replace("|", "/").replace("\r", " ").replace("\n", " ").strip()
    text = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode("ascii")
    text = " ".join(text.split())
    return text[:max_len] if max_len else text


def to_int(value: object, default: int = 0) -> int:
    try:
        text = str(value).strip()
        return int(float(text)) if text else default
    except (TypeError, ValueError):
        return default


def to_float(value: object) -> float | None:
    try:
        text = str(value).strip()
        return float(text) if text else None
    except (TypeError, ValueError):
        return None


def coord_e7(value: object) -> int | None:
    number = to_float(value)
    if number is None:
        return None
    return int(round(number * 10_000_000))


def download(url: str, destination: Path) -> None:
    print(f"Downloading {destination.name} ...")
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "Mini-Avionics-World-Navdata/1.0"},
    )
    context = ssl.create_default_context()
    with urllib.request.urlopen(request, context=context, timeout=180) as response:
        with destination.open("wb") as output:
            shutil.copyfileobj(response, output)
    print(f"  {destination.stat().st_size:,} bytes")


def find_navdata_folder(root: Path) -> Path:
    direct = root / "NAVDATA"
    if direct.is_dir():
        return direct
    matches = list(root.rglob("NAVDATA"))
    if len(matches) == 1:
        return matches[0]
    raise RuntimeError("Could not uniquely locate NAVDATA in the FAA package")


def read_rows(path: Path) -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    with path.open("r", encoding="ascii", errors="replace", newline="") as source:
        for raw in source:
            line = raw.rstrip("\r\n")
            if not line or line.startswith("#"):
                continue
            key = line.split("|", 1)[0].strip().upper()
            if key:
                rows.append((key, line))
    return rows


def write_indexed(data_path: Path, index_path: Path, rows: list[tuple[str, str]]) -> int:
    rows.sort(key=lambda item: (item[0], item[1]))
    groups: list[tuple[str, int, int]] = []
    offset = 0
    current_key: str | None = None
    current_offset = 0
    current_count = 0

    with data_path.open("wb") as output:
        for key, line in rows:
            encoded = (line + "\n").encode("ascii", errors="replace")
            if current_key is None:
                current_key = key
                current_offset = offset
                current_count = 0
            elif key != current_key:
                groups.append((current_key, current_offset, current_count))
                current_key = key
                current_offset = offset
                current_count = 0
            output.write(encoded)
            offset += len(encoded)
            current_count += 1
        if current_key is not None:
            groups.append((current_key, current_offset, current_count))

    with index_path.open("wb") as index:
        for key, group_offset, count in groups:
            safe_key = ascii_clean(key).upper()[:8].ljust(8)
            record = f"{safe_key}|{group_offset:010d}|{count:06d}\n".encode("ascii")
            if len(record) != INDEX_RECORD_SIZE:
                raise AssertionError((key, len(record), record))
            index.write(record)
    return len(groups)


def airport_identifier(row: dict[str, str]) -> str:
    # Prefer the identifier intended for aviation GPS use, then ICAO, then OA ident.
    for field in ("gps_code", "icao_code", "ident"):
        value = ascii_clean(row.get(field, ""), 8).upper()
        if value:
            return value
    return ""


def build_airport_overlay(csv_path: Path, existing_keys: set[str]) -> tuple[list[tuple[str, str]], set[str]]:
    rows: list[tuple[str, str]] = []
    added_keys: set[str] = set()
    with csv_path.open("r", encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            if row.get("type", "").strip() == "closed_airport":
                continue
            ident = airport_identifier(row)
            if not ident or ident in existing_keys or ident in added_keys:
                continue
            lat = coord_e7(row.get("latitude_deg"))
            lon = coord_e7(row.get("longitude_deg"))
            if lat is None or lon is None:
                continue
            iata = ascii_clean(row.get("iata_code", ""), 4).upper()
            region = ascii_clean(row.get("iso_region") or row.get("iso_country", ""), 8).upper()
            elev = to_int(row.get("elevation_ft"), 0)
            name = ascii_clean(row.get("name", ""), 72)
            line = f"{ident}|{iata}|{region}|{lat}|{lon}|{elev}|{name}"
            rows.append((ident, line))
            added_keys.add(ident)
    return rows, added_keys


def build_runway_overlay(csv_path: Path, overlay_airports: set[str]) -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    seen: set[str] = set()
    with csv_path.open("r", encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            airport = ascii_clean(row.get("airport_ident", ""), 8).upper()
            if airport not in overlay_airports or row.get("closed", "0").strip() == "1":
                continue
            length = to_int(row.get("length_ft"), 0)
            width = to_int(row.get("width_ft"), 0)
            surface = ascii_clean(row.get("surface", ""), 18).upper()

            for prefix in ("le", "he"):
                end_ident = ascii_clean(row.get(f"{prefix}_ident", ""), 6).upper()
                lat = coord_e7(row.get(f"{prefix}_latitude_deg"))
                lon = coord_e7(row.get(f"{prefix}_longitude_deg"))
                if not end_ident or lat is None or lon is None:
                    continue
                runway = end_ident if end_ident.startswith("RW") else f"RW{end_ident}"
                heading = to_float(row.get(f"{prefix}_heading_degT"))
                bearing_tenths = int(round((heading or 0.0) * 10.0))
                elev = to_int(row.get(f"{prefix}_elevation_ft"), 0)
                displaced = to_int(row.get(f"{prefix}_displaced_threshold_ft"), 0)
                description = ascii_clean(f"OURAIRPORTS TRUE HDG {surface}", 50)
                line = (
                    f"{airport}|{runway}|{length}|{width}|{bearing_tenths}|"
                    f"{lat}|{lon}|{elev}|{displaced}|0|||{description}"
                )
                if line not in seen:
                    rows.append((airport, line))
                    seen.add(line)
    return rows


def build_navaid_overlay(csv_path: Path, existing_lines: set[str]) -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as source:
        for row in csv.DictReader(source):
            country = ascii_clean(row.get("iso_country", ""), 3).upper()
            if country in FAA_COUNTRY_CODES:
                continue
            ident = ascii_clean(row.get("ident", ""), 8).upper()
            if not ident:
                continue
            lat = coord_e7(row.get("latitude_deg"))
            lon = coord_e7(row.get("longitude_deg"))
            if lat is None or lon is None:
                continue
            navaid_type = ascii_clean(row.get("type", ""), 20).upper()
            kind = "N" if "NDB" in navaid_type else "V"
            parent = ascii_clean(row.get("associated_airport") or country, 8).upper()
            frequency = ascii_clean(row.get("frequency_khz", ""), 10)
            usage = ascii_clean(row.get("usageType", ""), 10).upper()
            name = ascii_clean(row.get("name", ""), 72)
            line = (
                f"{ident}|{kind}|{parent}|{country}|{lat}|{lon}|{frequency}|"
                f"{navaid_type}|{usage}|{name}"
            )
            if line not in existing_lines:
                rows.append((ident, line))
                existing_lines.add(line)
    return rows


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_index(data_path: Path, index_path: Path, key: str) -> list[str]:
    target = key.upper().ljust(8).encode("ascii")
    size = index_path.stat().st_size // INDEX_RECORD_SIZE
    with index_path.open("rb") as index:
        low, high = 0, size - 1
        hit: tuple[int, int] | None = None
        while low <= high:
            mid = (low + high) // 2
            index.seek(mid * INDEX_RECORD_SIZE)
            record = index.read(INDEX_RECORD_SIZE)
            record_key = record[:8]
            if record_key == target:
                parts = record.decode("ascii").rstrip("\n").split("|")
                hit = (int(parts[1]), int(parts[2]))
                break
            if record_key < target:
                low = mid + 1
            else:
                high = mid - 1
    if hit is None:
        return []
    offset, count = hit
    lines: list[str] = []
    with data_path.open("rb") as data:
        data.seek(offset)
        for _ in range(count):
            lines.append(data.readline().decode("ascii", errors="replace").rstrip("\r\n"))
    return lines


def build(args: argparse.Namespace) -> Path:
    base_package = args.faa_package.resolve()
    if not base_package.exists():
        raise FileNotFoundError(base_package)

    output_zip = args.output.resolve()
    output_folder = output_zip.with_suffix("")
    work = Path(tempfile.mkdtemp(prefix="mini_fmc_world_"))
    try:
        extracted = work / "faa"
        extracted.mkdir()
        with zipfile.ZipFile(base_package) as archive:
            archive.extractall(extracted)
        base_nav = find_navdata_folder(extracted)

        if output_folder.exists():
            shutil.rmtree(output_folder)
        nav = output_folder / "NAVDATA"
        shutil.copytree(base_nav, nav)

        downloads = work / "ourairports"
        downloads.mkdir()
        csv_paths: dict[str, Path] = {}
        for name, url in URLS.items():
            supplied = getattr(args, f"{name}_csv")
            if supplied:
                csv_paths[name] = supplied.resolve()
            else:
                csv_paths[name] = downloads / f"{name}.csv"
                download(url, csv_paths[name])

        airport_rows = read_rows(nav / "airports.dat")
        runway_rows = read_rows(nav / "runways.dat")
        point_rows = read_rows(nav / "points.dat")

        existing_airport_keys = {key for key, _ in airport_rows}
        world_airports, overlay_airport_keys = build_airport_overlay(
            csv_paths["airports"], existing_airport_keys
        )
        world_runways = build_runway_overlay(csv_paths["runways"], overlay_airport_keys)
        world_navaids = build_navaid_overlay(
            csv_paths["navaids"], {line for _, line in point_rows}
        )

        airport_rows.extend(world_airports)
        runway_rows.extend(world_runways)
        point_rows.extend(world_navaids)

        airport_keys = write_indexed(nav / "airports.dat", nav / "airports.idx", airport_rows)
        runway_keys = write_indexed(nav / "runways.dat", nav / "runways.idx", runway_rows)
        point_keys = write_indexed(nav / "points.dat", nav / "points.idx", point_rows)

        now = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
        cycle_path = nav / "cycle.txt"
        cycle_text = cycle_path.read_text(encoding="ascii", errors="replace").rstrip() + "\n"
        cycle_text += (
            "WORLD_SOURCE=OURAIRPORTS\n"
            f"WORLD_BUILT={now}\n"
            "WORLD_SCOPE=AIRPORTS_RUNWAYS_RADIO_NAVAIDS\n"
        )
        cycle_path.write_text(cycle_text, encoding="ascii")

        meta_path = nav / "meta.json"
        meta = json.loads(meta_path.read_text(encoding="ascii")) if meta_path.exists() else {}
        meta["dataset_standard"] = "FAA CIFP / ARINC 424-18 + OurAirports public-domain overlay"
        meta["world_overlay"] = {
            "source": "OurAirports",
            "source_urls": URLS,
            "built_utc": now,
            "coverage": "Worldwide airports, runway ends where available, and radio navaids",
            "limitations": "Worldwide fixes, airways, SIDs, STARs, and approaches are not included",
            "added_airports": len(world_airports),
            "added_runway_ends": len(world_runways),
            "added_radio_navaids": len(world_navaids),
        }
        meta.setdefault("unique_index_keys", {}).update({
            "airports": airport_keys,
            "runways": runway_keys,
            "points": point_keys,
        })
        meta["warning"] = "DEVELOPMENT USE ONLY - NOT APPROVED FOR REAL-WORLD NAVIGATION"
        meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="ascii")

        (nav / "WORLD_README.txt").write_text(
            "MINI AVIONICS HYBRID WORLD NAVDATA\n"
            "===================================\n"
            "FAA CIFP remains authoritative for U.S. airports, fixes, navaids, airways,\n"
            "SIDs, STARs, approaches, and procedure legs.\n\n"
            "OurAirports adds worldwide non-closed airports, runway ends where endpoint\n"
            "coordinates are available, and worldwide radio navaids outside FAA coverage.\n\n"
            "Outside FAA coverage, this package does NOT contain a complete worldwide set\n"
            "of enroute fixes, airways, SIDs, STARs, approaches, or constraints.\n\n"
            "DEVELOPMENT USE ONLY - NOT APPROVED FOR REAL-WORLD NAVIGATION.\n",
            encoding="ascii",
        )

        # Rebuild checksums last, excluding the checksum file itself.
        checksum_path = nav / "SHA256SUMS.txt"
        if checksum_path.exists():
            checksum_path.unlink()
        checksum_lines = []
        for file in sorted(p for p in nav.iterdir() if p.is_file()):
            checksum_lines.append(f"{sha256_file(file)}  {file.name}")
        checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="ascii")

        # Validation expected by this project.
        validations = {
            "GOBD": validate_index(nav / "airports.dat", nav / "airports.idx", "GOBD"),
            "EGLL": validate_index(nav / "airports.dat", nav / "airports.idx", "EGLL"),
            "KPHL": validate_index(nav / "airports.dat", nav / "airports.idx", "KPHL"),
        }
        for key, lines in validations.items():
            if not lines:
                raise RuntimeError(f"Validation failed: {key} not found")
            print(f"Validated {key}: {lines[0]}")

        (output_folder / "README_FIRST.txt").write_text(
            "Copy the NAVDATA folder directly to the microSD card root, replacing the old\n"
            "NAVDATA folder. The existing v0.8.1 firmware lookup format is compatible.\n"
            "Use the v0.8.2 sketch to display the source as FAA + WORLD.\n",
            encoding="ascii",
        )

        if output_zip.exists():
            output_zip.unlink()
        with zipfile.ZipFile(output_zip, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for file in sorted(output_folder.rglob("*")):
                if file.is_file():
                    archive.write(file, file.relative_to(output_folder))

        print("\nHybrid package complete")
        print(f"  Airports added:      {len(world_airports):,}")
        print(f"  Runway ends added:   {len(world_runways):,}")
        print(f"  Radio navaids added: {len(world_navaids):,}")
        print(f"  Output: {output_zip}")
        return output_zip
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--faa-package",
        type=Path,
        default=Path("Mini_FMC_NAVDATA_260806.zip"),
        help="Existing FAA Mini FMC NAVDATA ZIP",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Mini_FMC_NAVDATA_260806_WORLD.zip"),
        help="Output hybrid ZIP",
    )
    parser.add_argument("--airports-csv", type=Path)
    parser.add_argument("--runways-csv", type=Path)
    parser.add_argument("--navaids-csv", type=Path)
    args = parser.parse_args()
    try:
        build(args)
    except Exception as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
