# scan_controller

Unified Python package that combines CAEN digitizer data acquisition
(`wja_caen_tcal.Caen`) with Raspberry Pi Pico stepper-motor control, so you
can write automated scan scripts (or run them from a Jupyter notebook) without
juggling a separate VS Code window.

---

## How it works

```
Your PC
  │
  ├── USB (pyserial)  ──►  Raspberry Pi Pico  (running Main.py / MicroPython)
  │                          stepper drivers, limit switches
  │
  └── USB / PCIe      ──►  CAEN N6742 digitizer  (via wja_caen_tcal.Caen / pyvisa)
```

`pico_controller.PicoController` opens the Pico's USB CDC serial port and
sends the same text commands that the Pico's built-in REPL accepts (`home`,
`movemm`, `mul`, …).  No changes to `Main.py` are required — you just stop
typing in VS Code and let Python type for you.

`scan_controller.ScanController` wraps both pieces and exposes high-level
methods: `grid_scan()`, `line_scan()`, `point_scan()`, `acquire()`, etc.

---

## Project layout

```
your_project/
  scan_controller/
    scan_controller.py       ← unified controller  ← edit OUTPUT_DIR here
    pico/
      __init__.py
      pico_controller.py     ← serial interface to Pico
    scripts/
      example_scan.py        ← standalone Python example
    notebooks/
      scan_notebook.ipynb    ← Jupyter example / GUI
  gui/                       ← YOUR EXISTING CODE (unchanged)
    wja_caen_tcal.py
    nbutil.py
```

---

## Prerequisites

```bash
pip install pyserial h5py numpy scipy matplotlib pyvisa
```

Your existing `wja_caen_tcal` dependencies (pyvisa, h5py, etc.) are unchanged.

---

## Quickstart (Jupyter)

```python
from scan_controller import ScanController
import numpy as np

sc = ScanController(pico_port=None)   # auto-detects Pico USB port
sc.connect()                          # opens CAEN + Pico, loads DRS corrections

# Move manually
sc.move_to(x_mm=5.0, y_mm=10.0)

# Quick test acquire + plot
sc.acquire(nevents=10)

# Grid scan — saves one HDF5 per grid point
files = sc.grid_scan(
    x_positions=np.arange(0, 24, 3.0),
    y_positions=np.arange(0, 24, 3.0),
    nevents=10_000,
    output_dir='scan_data',
    label='run1',
)

sc.disconnect()
```

Open `notebooks/scan_notebook.ipynb` for a full worked example.

---

## Pico port detection

`PicoController` auto-detects the Pico by USB Vendor ID (0x2E8A).  If
auto-detection fails (e.g. the VID isn't exposed on your OS), pass the port
explicitly:

```python
sc = ScanController(pico_port='/dev/ttyACM0')   # Linux
sc = ScanController(pico_port='COM5')           # Windows
```

Run `python -m serial.tools.list_ports -v` to list available ports.

---

## No changes to Main.py required

`pico_controller.py` speaks the same text protocol that `Main.py`'s `repl()`
function already handles.  The only requirement is that `Main.py` is running
on the Pico (which it will be if it is saved as `main.py`).

---

## Position tracking

`PicoController` parses the Pico's print statements to keep a local mirror
of the current position:

```python
sc.pico.current_M_x   # X axis screw position [mm]
sc.pico.current_M_y   # Y axis screw position [mm]
sc.pico.current_Z     # Z platform height [mm]
sc.position           # dict with all three
```

These are advisory — the Pico is the source of truth.  After a `home()` call
the values are zeroed automatically.

---

## HDF5 output

Each call to `acquire_and_save()` (called internally by `point_scan`) writes
one HDF5 file named:

```
YYYYMMDD_HHMMSS_<label>_x<X>mm_y<Y>mm.hdf5
```

The file contains the same datasets as the original notebook
(`drsraw`, `drs`, `drsu`, `traw`, `tcor`, `drs_trig_cell`, `drs_tstamp`,
`drs_cellgains`, `drs_cellwidth`, `drs_peds`, `drs_wiggle_shape`,
`drs_mean_shape`) plus root attributes `x_mm`, `y_mm`, and `timestamp`.

---

## Building a GUI later

When you're ready, `ScanController` is straightforward to wrap with any Python
GUI toolkit (Qt, tkinter, Panel, ipywidgets).  The public API is:

| Method | Description |
|---|---|
| `connect()` | Open both devices |
| `disconnect()` | Close both |
| `home_all()` | Home X then Y |
| `move_to(x_mm, y_mm)` | Absolute move |
| `acquire(nevents)` | Take data, store in `sc.caen.trigev` |
| `acquire_and_save(...)` | Acquire + write HDF5 |
| `point_scan(positions, ...)` | Visit list of (x,y) points |
| `line_scan(...)` | 1-D scan |
| `grid_scan(...)` | 2-D raster scan |
| `status()` | Print current state |
| `position` | Dict of current motor positions |
