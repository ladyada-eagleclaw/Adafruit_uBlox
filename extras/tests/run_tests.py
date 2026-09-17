"""Discover and run every host regression with address/undefined sanitizers."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gps-dir", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    gps = args.gps_dir.resolve() / "src"
    tests = sorted((root / "extras/tests").glob("*/*.cpp"))
    if not tests or not (gps / "Adafruit_GNSS.h").is_file():
        raise RuntimeError("Tests and Adafruit GPS 1.9.0+ are required")
    sources = [root / "Adafruit_UBX.cpp", root / "Adafruit_UBloxDDC.cpp",
               gps / "Adafruit_NMEA.cpp", gps / "Adafruit_GNSS.cpp"]
    flags = ["-std=c++11", "-Wall", "-Wextra", "-Werror", "-g",
             "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
             "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
             "-I" + str(root / "extras/tests/support"),
             "-I" + str(root), "-I" + str(gps)]
    with tempfile.TemporaryDirectory(prefix="ublox-tests-") as build:
        for index, test in enumerate(tests):
            binary = str(Path(build) / str(index))
            subprocess.run([os.environ.get("CXX", "g++"), *flags,
                            *map(str, sources), str(test), "-o", binary],
                           check=True, timeout=120)
            subprocess.run([binary], check=True, timeout=30)
            print("PASS", test.relative_to(root), flush=True)
    print(f"All {len(tests)} regression sources passed.")


if __name__ == "__main__":
    main()
