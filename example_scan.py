"""
example_scan.py

Demonstrates the full scan workflow from plain Python (no Jupyter).
Run this as:
    python example_scan.py

or call scan_sequence() from your own script.
"""

import sys
import os
import numpy as np

# ── path setup ───────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))        # scan_controller package
sys.path.insert(0, os.path.join(_HERE, "../gui"))    # wja_caen_tcal, nbutil

from scan_controller import ScanController


def scan_sequence():
    # ── configure ────────────────────────────────────────────────────────────
    PICO_PORT    = None           # None = auto-detect; set to '/dev/ttyACM0' etc.
    N_EVENTS     = 10_000
    OUTPUT_DIR   = "scan_data"
    LABEL        = "example_grid"

    x_positions  = np.arange(0, 24, 3.0)    # 0 … 21 mm, 3 mm steps
    y_positions  = np.arange(0, 24, 3.0)    # same for Y/Z axis

    # ── run ──────────────────────────────────────────────────────────────────
    with ScanController(pico_port=PICO_PORT) as sc:
        sc.status()

        files = sc.grid_scan(
            x_positions=x_positions,
            y_positions=y_positions,
            nevents=N_EVENTS,
            output_dir=OUTPUT_DIR,
            label=LABEL,
            settle_time=0.5,
            home_first=True,
            serpentine=True,
        )

    print("\nAll done.  Files written:")
    for f in files:
        print(" ", f)


if __name__ == "__main__":
    scan_sequence()
