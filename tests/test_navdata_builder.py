"""Offline unit checks for the NAVDATA indexing and world-overlay helpers."""
import csv
import tempfile
import unittest
from pathlib import Path

from tools import build_world_hybrid_navdata as nav


class NavdataBuilderTests(unittest.TestCase):
    def test_index_round_trip_and_fixed_records(self):
        with tempfile.TemporaryDirectory() as temp:
            data = Path(temp) / "airports.dat"
            index = Path(temp) / "airports.idx"
            rows = [
                ("KPHL", "KPHL|PHL|TEST"),
                ("GOBD", "GOBD|DSS|TEST"),
                ("GOBD", "GOBD|DSS|SECOND"),
            ]
            self.assertEqual(nav.write_indexed(data, index, rows), 2)
            self.assertEqual(index.stat().st_size, 2 * nav.INDEX_RECORD_SIZE)
            self.assertEqual(nav.validate_index(data, index, "GOBD"),
                             ["GOBD|DSS|SECOND", "GOBD|DSS|TEST"])
            self.assertEqual(nav.validate_index(data, index, "EGLL"), [])

    def test_airport_overlay_excludes_closed_and_existing(self):
        with tempfile.TemporaryDirectory() as temp:
            csv_file = Path(temp) / "airports.csv"
            with csv_file.open("w", newline="", encoding="utf-8") as fp:
                w = csv.DictWriter(fp, fieldnames=[
                    "ident", "gps_code", "icao_code", "type", "latitude_deg",
                    "longitude_deg", "iata_code", "iso_region", "elevation_ft", "name"
                ])
                w.writeheader()
                w.writerow(dict(ident="GOBD", gps_code="GOBD", type="large_airport",
                                latitude_deg="14.670878", longitude_deg="-17.072829",
                                iata_code="DSS", iso_region="SN-DK", name="Blaise Diagne"))
                w.writerow(dict(ident="KPHL", gps_code="KPHL", type="large_airport",
                                latitude_deg="39", longitude_deg="-75"))
                w.writerow(dict(ident="CLOSED", gps_code="CLOSED", type="closed_airport",
                                latitude_deg="10", longitude_deg="10"))
            records, keys = nav.build_airport_overlay(csv_file, {"KPHL"})
            self.assertEqual(keys, {"GOBD"})
            self.assertEqual(len(records), 1)
            self.assertTrue(records[0][1].startswith("GOBD|DSS|SN-DK|146708780|-170728290|"))

    def test_runway_overlay_requires_endpoint_coordinates(self):
        with tempfile.TemporaryDirectory() as temp:
            csv_file = Path(temp) / "runways.csv"
            with csv_file.open("w", newline="", encoding="utf-8") as fp:
                w = csv.DictWriter(fp, fieldnames=[
                    "airport_ident", "closed", "length_ft", "width_ft", "surface",
                    "le_ident", "le_latitude_deg", "le_longitude_deg", "le_heading_degT",
                    "he_ident", "he_latitude_deg", "he_longitude_deg", "he_heading_degT"
                ])
                w.writeheader()
                w.writerow(dict(airport_ident="GOBD", closed="0", length_ft="11000",
                                width_ft="150", surface="ASP", le_ident="01",
                                le_latitude_deg="14.64", le_longitude_deg="-17.08",
                                he_ident="19", he_latitude_deg="14.70", he_longitude_deg="-17.08"))
            records = nav.build_runway_overlay(csv_file, {"GOBD"})
            self.assertEqual(len(records), 2)
            self.assertTrue(all(key == "GOBD" for key, _ in records))
            self.assertIn("RW01", records[0][1])
            self.assertIn("RW19", records[1][1])


if __name__ == "__main__":
    unittest.main()
