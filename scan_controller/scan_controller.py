"""
scan_controller.py

High-level orchestrator that unifies CAEN digitizer data acquisition
(via wja_caen_tcal.Caen) with Raspberry Pi Pico stepper-motor control
(via pico.pico_controller.PicoController).

Typical usage
-------------
from scan_controller import ScanController
import numpy as np

sc = ScanController(pico_port=None)   # auto-detect Pico; adjust as needed
sc.connect()
sc.caen.load_drs_corrections()

# Simple grid scan
x_positions = np.arange(0, 30, 3.0)   # mm
y_positions = np.arange(0, 24, 3.0)   # mm (Z-space for scissor stage)
sc.grid_scan(
    x_positions, y_positions,
    nevents=1000,
    output_dir="scan_data",
    label="myrun",
)
sc.disconnect()
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Sequence

import numpy as np
import h5py

# ── local imports ─────────────────────────────────────────────────────────────
# wja_caen_tcal.py and nbutil must be on sys.path (same folder, or installed).
# Adjust the sys.path manipulation below if your layout differs.
import sys
_HERE = Path(__file__).parent
# If wja_caen_tcal lives in ../gui relative to this file, add it:
for _candidate in [_HERE, _HERE.parent / "gui", _HERE.parent]:
    if (_candidate / "wja_caen_tcal.py").exists():
        sys.path.insert(0, str(_candidate))
        break

from wja_caen_tcal import Caen                       # noqa: E402
from pico.pico_controller import PicoController      # noqa: E402


# ── helpers ────────────────────────────────────────────────────────────────────

def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _build_hdf5_filename(label: str, x_mm: float, y_mm: float,
                          output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    fname = f"{_timestamp()}_{label}_x{x_mm:.2f}mm_y{y_mm:.2f}mm.hdf5"
    return os.path.join(output_dir, fname)


def _save_events_hdf5(caen: Caen, fpath: str,
                       x_mm: float, y_mm: float) -> None:
    """
    Write corrected events from caen.trigev to an HDF5 file.
    Mirrors the save pattern in caen_daq.ipynb.
    """
    ns = SimpleNamespace()
    ns.drsraw          = np.array([e.drsraw      for e in caen.trigev], dtype=np.int16)
    ns.drs             = np.array([e.drsgc       for e in caen.trigev], dtype=np.float32)
    ns.drsu            = np.array([e.drsu        for e in caen.trigev], dtype=np.float32)
    ns.traw            = np.array([e.traw        for e in caen.trigev], dtype=np.float32)
    ns.tcor            = np.array([e.tcor        for e in caen.trigev], dtype=np.float32)
    ns.drs_trig_cell   = np.array([e.drs_trig_cell for e in caen.trigev], dtype=np.int16)
    ns.drs_tstamp      = np.array([e.tstamp      for e in caen.trigev], dtype=np.int64)
    ns.drs_cellgains   = np.array([tc.cellgain   for tc in caen.tcal],  dtype=np.float32)
    ns.drs_cellwidth   = np.array([tc.celldt     for tc in caen.tcal],  dtype=np.float32)
    ns.drs_peds        = np.array([tc.cellpeds   for tc in caen.tcal],  dtype=np.float32)
    ns.drs_wiggle_shape= np.array([tc.wiggleshape for tc in caen.tcal], dtype=np.float32)
    ns.drs_mean_shape  = np.array([tc.meanshape  for tc in caen.tcal],  dtype=np.float32)

    _COMMENTS = {
        "drsraw":           "raw uncorrected DRS samples [event][channel][sample]",
        "drs":              "DRS samples with pedestal and gain corrections applied",
        "drsu":             "DRS corrected samples, resampled to equal time intervals",
        "traw":             "nominal uncorrected DRS sample times",
        "tcor":             "DRS sample times, corrected via timing calibration",
        "drs_trig_cell":    "DRS trigger/stop cell ID [event][channel]",
        "drs_tstamp":       "CAEN board time stamp [event]",
        "drs_cellgains":    "voltage gain [channel][cell] from DRS calibration",
        "drs_cellwidth":    "cell width (ns) [channel][cell] from DRS timing calibration",
        "drs_peds":         "DRS pedestal [channel][cell] from DRS calibration",
        "drs_wiggle_shape": "fitted 'wiggle' shape subtracted from each DRS waveform",
        "drs_mean_shape":   "mean waveform, in absence of signal, CAEN board artifact",
    }

    with h5py.File(fpath, "w") as hf:
        # Store scan position as root attributes
        hf.attrs["x_mm"]       = x_mm
        hf.attrs["y_mm"]       = y_mm
        hf.attrs["timestamp"]  = _timestamp()

        for name, comment in _COMMENTS.items():
            arr = ns.__dict__[name]
            ds = hf.create_dataset(
                name, data=arr,
                shuffle=True, compression="gzip", compression_opts=1,
            )
            ds.attrs["comment"] = comment

    print(f"  Saved {len(caen.trigev)} events → {fpath}")


# ── main class ────────────────────────────────────────────────────────────────

class ScanController:
    """
    Unified controller for CAEN digitizer + Pico stepper motors.

    Parameters
    ----------
    pico_port : str or None
        Serial port for the Pico.  None = auto-detect.
    pico_response_timeout : float
        Seconds to wait for slow Pico operations (homing).
    """

    def __init__(
        self,
        pico_port: Optional[str] = None,
        pico_response_timeout: float = 120.0,
    ):
        self.caen = Caen()
        self.pico = PicoController(
            port=pico_port,
            response_timeout=pico_response_timeout,
        )
        self._caen_corrections_loaded = False

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def connect(self, load_caen_corrections: bool = True) -> None:
        """Open CAEN + Pico connections and optionally load DRS corrections."""
        print("[ScanController] Connecting CAEN …")
        # Caen.__init__ opens the digitizer; nothing extra needed here.
        if load_caen_corrections:
            self.load_caen_corrections()

        print("[ScanController] Connecting Pico …")
        self.pico.connect()
        print("[ScanController] Ready.")

    def disconnect(self) -> None:
        self.pico.disconnect()
        print("[ScanController] Disconnected.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def load_caen_corrections(self) -> None:
        print("[ScanController] Loading DRS corrections …")
        self.caen.load_drs_corrections()
        self._caen_corrections_loaded = True
        print("[ScanController] DRS corrections loaded.")

    # ── data acquisition ──────────────────────────────────────────────────────

    def acquire(self, nevents: int = 1000) -> None:
        """
        Trigger CAEN readout and apply corrections.

        Results are stored in self.caen.trigev as usual.
        """
        if not self._caen_corrections_loaded:
            raise RuntimeError("Call load_caen_corrections() before acquiring.")
        print(f"  Acquiring {nevents} events …", end=" ", flush=True)
        t0 = time.time()
        self.caen.do_triggered_readout(nevents)
        print(f"({time.time()-t0:.1f}s acq) ", end="", flush=True)
        t0 = time.time()
        self.caen.correct_triggered_readout()
        print(f"({time.time()-t0:.1f}s corr)")

    def acquire_and_save(
        self,
        nevents: int,
        output_dir: str,
        label: str,
        x_mm: float,
        y_mm: float,
    ) -> str:
        """Acquire data and immediately write to HDF5.  Returns the file path."""
        self.acquire(nevents)
        fpath = _build_hdf5_filename(label, x_mm, y_mm, output_dir)
        _save_events_hdf5(self.caen, fpath, x_mm, y_mm)
        return fpath

    # ── motion helpers ────────────────────────────────────────────────────────

    def home_all(self) -> None:
        """Home both axes (X first, then Y)."""
        self.pico.home("x")
        self.pico.home("y")

    def move_to(self, x_mm: Optional[float] = None,
                y_mm: Optional[float] = None,
                fast: bool = True) -> None:
        """
        Move to an absolute position.  Either or both axes may be specified.

        x_mm  → absolute M (linear screw) position on X axis
        y_mm  → absolute Z (platform height) position on Y axis
        """
        if x_mm is not None:
            self.pico.move_to_mm("x", x_mm, fast=fast)
        if y_mm is not None:
            self.pico.move_to_mm("y", y_mm, fast=fast)

    # ── scan patterns ─────────────────────────────────────────────────────────

    def point_scan(
        self,
        positions: Sequence[tuple],   # list of (x_mm, y_mm)
        nevents: int,
        output_dir: str = "scan_data",
        label: str = "scan",
        settle_time: float = 0.5,
        home_first: bool = True,
    ) -> List[str]:
        """
        Visit each (x, y) position, acquire data, save HDF5.

        Parameters
        ----------
        positions    : sequence of (x_mm, y_mm) tuples
        nevents      : events per position
        output_dir   : where to write HDF5 files
        label        : string embedded in every filename
        settle_time  : seconds to wait after motion before acquiring
        home_first   : if True, home both axes before the scan

        Returns
        -------
        list of HDF5 file paths created
        """
        if home_first:
            print("[ScanController] Homing before scan …")
            self.home_all()

        files = []
        total = len(positions)
        for i, (x_mm, y_mm) in enumerate(positions):
            print(f"\n[ScanController] Point {i+1}/{total}  x={x_mm:.2f} y={y_mm:.2f}")
            self.move_to(x_mm=x_mm, y_mm=y_mm)
            time.sleep(settle_time)
            fpath = self.acquire_and_save(nevents, output_dir, label, x_mm, y_mm)
            files.append(fpath)

        print(f"\n[ScanController] Scan complete. {len(files)} files written.")
        return files

    def grid_scan(
        self,
        x_positions: Sequence[float],
        y_positions: Sequence[float],
        nevents: int,
        output_dir: str = "scan_data",
        label: str = "grid",
        settle_time: float = 0.5,
        home_first: bool = True,
        serpentine: bool = True,
    ) -> List[str]:
        """
        Perform a 2-D raster scan.

        Parameters
        ----------
        x_positions  : 1-D array of X positions [mm]
        y_positions  : 1-D array of Y positions [mm]
        nevents      : events per grid point
        output_dir   : HDF5 output directory
        label        : filename label
        settle_time  : post-move settle [s]
        home_first   : home both axes first
        serpentine   : if True, reverse X direction on alternate Y rows
                       (minimises travel distance)

        Returns
        -------
        list of HDF5 file paths
        """
        points = []
        for row_idx, y in enumerate(y_positions):
            row_x = list(x_positions) if (not serpentine or row_idx % 2 == 0) \
                    else list(reversed(x_positions))
            for x in row_x:
                points.append((x, y))

        print(f"[ScanController] Grid scan: "
              f"{len(x_positions)} × {len(y_positions)} = {len(points)} points, "
              f"{nevents} events each.")
        return self.point_scan(
            points, nevents,
            output_dir=output_dir, label=label,
            settle_time=settle_time, home_first=home_first,
        )

    def line_scan(
        self,
        axis: str,
        positions: Sequence[float],
        fixed_other: float,
        nevents: int,
        output_dir: str = "scan_data",
        label: str = "linescan",
        settle_time: float = 0.5,
        home_first: bool = True,
    ) -> List[str]:
        """
        Scan along one axis while keeping the other fixed.

        axis         : 'x' or 'y'
        positions    : positions along the scanned axis [mm]
        fixed_other  : position of the other axis [mm]
        """
        if axis.lower() == "x":
            points = [(p, fixed_other) for p in positions]
        else:
            points = [(fixed_other, p) for p in positions]
        return self.point_scan(
            points, nevents,
            output_dir=output_dir, label=label,
            settle_time=settle_time, home_first=home_first,
        )

    # ── status ────────────────────────────────────────────────────────────────

    @property
    def position(self) -> dict:
        return self.pico.position

    def status(self) -> None:
        print(f"CAEN corrections loaded : {self._caen_corrections_loaded}")
        print(f"Pico connected          : {self.pico.connected}")
        print(f"Pico position           : {self.pico.position}")
