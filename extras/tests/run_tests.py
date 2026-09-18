"""Discover and run every host regression with address/undefined sanitizers."""
import argparse
import importlib.util
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gps-dir", type=Path, required=True)
    parser.add_argument("--harness-dir", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    gps = args.gps_dir.resolve() / "src"
    tests = sorted((root / "extras/tests").glob("*/*.cpp"))
    if not tests or not (gps / "Adafruit_GNSS.h").is_file():
        raise RuntimeError("Tests and Adafruit GPS 1.9.0+ are required")
    sources = [root / "Adafruit_UBX.cpp", root / "Adafruit_UBloxDDC.cpp",
               gps / "Adafruit_NMEA.cpp", gps / "Adafruit_GNSS.cpp"]
    runner = args.harness_dir.resolve() / "run_tests.py"
    spec = importlib.util.spec_from_file_location("adafruit_test_harness", runner)
    harness = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(harness)
    fixtures = args.harness_dir.resolve() / "fixtures/ublox"
    harness.run_tests(tests, sources, [fixtures, root, gps])


if __name__ == "__main__":
    main()
