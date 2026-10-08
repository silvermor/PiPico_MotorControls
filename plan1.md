# Plan 1: crystal-addressed moves and automatic scans

## Goal

Replace the hand-computed relative moves in `sipm_10x10_xtal_array_daq_v0.ipynb` with commands that take crystal numbers:

```python
stage.goto(55)                      # move only, no data (e.g. to watch the discriminator rate)
stage.goto_center()                 # flood position
full_scan(nevents=1500)             # all 100 crystals, bottom row to top row, snake in X
scan([34, 35, 44, 45], nevents=1500)  # partial scan, order chosen to avoid Y homing
```

The Y axis must never be driven backwards to reach a target. If a target is below the current Y position, Y is homed first and then driven forward to the target.

## Facts about the current code that the design depends on

- **Crystal numbering.** Crystal `n` (1 to 100) is at row `(n-1)//10`, column `(n-1)%10`, position `(xs[col], ys[row])`. Crystal 1 is `(x0, y0)`, crystal 10 is `(x9, y0)`, crystal 91 is `(x0, y9)`, crystal 100 is `(x9, y9)`.
- **Y units.** `y0..y9` are platform height above the home height (the argument of `move_distance_mm_y` from home), not screw travel. `y9 = 30.83` (row 10) is the lowest row and `y0 = 44.65` (row 1) the highest, so a bottom-to-top scan goes row 10 to row 1 and only moves Y forward.
- **X direction.** `x9 = 6.73` is nearest home and `x0 = 20.55` farthest. X is moved in both directions today and that stays.
- **Position is tracked only on the Pico** (`current_M_xaxis`, `current_M_yaxis`). The host has no copy.
- **Re-importing `Main` zeroes the tracked position without moving the stage.** `Pico(...)` uploads and re-imports on every connect, and notebook cell 4 re-imports again. After any connect the firmware's position is wrong until the axes are homed.
- **`calibrate('y')` ends with a full-travel reverse move**, so Y is backlash-tainted right after calibration.
- **The Pico uses single-precision floats.** A simulation of the firmware arithmetic shows that asking for the row the stage is already on gives a residual of up to 0.498 microsteps (row 10). A "is the target behind me" test that rounds to whole steps could therefore trigger a needless homing. The test must use a deadband of one full microstep.
- **Homing is slow by design**: about 0.13 mm/s of screw travel plus two 10 s switch-stability waits. From row 10 (screw at about 19 mm) that is roughly 3 minutes; from row 1 (about 34 mm) roughly 5 minutes.

## 1. Firmware: `Main.py`

### Trust flags

Add two module globals, both `False` at import:

- `y_trusted`: True only when Y has been homed and every Y step since then was forward.
- `x_homed`: True once X has been homed.

Where they change:

| Place | Change |
|---|---|
| end of `home('y')` | `y_trusted = True` |
| end of `home('x')` | `x_homed = True` |
| start of `move_until_limit_y` | `y_trusted = False` (`home` sets it back to True afterwards; `calibrate` and the REPL `mul` command leave it False) |
| `move_steps_y`, `move_distance_mm_y` | `y_trusted = False` when the direction is reverse and at least one step was taken |
| `test_axis_backlash` | clear the flag for the axis under test |
| `disable_motor` | clear both (the stage can be moved by hand) |

These are all the callers of `safe_step_y`, so no reverse Y step can go unrecorded. The flag is set in the callers, not inside `safe_step_y`, to keep the per-step timing unchanged.

### Absolute moves

```python
def move_to_y(y, speed_fast=True):
    # y is height above home, same units as the notebook's y0..y9
    target_M = m_from_z(z_from_m(0.0) + y)
    delta_M = target_M - current_M_yaxis
    one_step = MM_PER_REV_Y / STEPS_PER_REV
    if (not y_trusted) or delta_M < -one_step:
        home('y')
    cur_y = z_from_m(current_M_yaxis) - z_from_m(0.0)
    if y - cur_y > 0:
        move_distance_mm_y(y - cur_y, speed_fast, True)

def move_to_x(x, speed_fast=True):
    if not x_homed:
        home('x')
    delta = x - current_M_xaxis
    if abs(delta) >= MM_PER_REV_X / STEPS_PER_REV:
        move_distance_mm_x(abs(delta), speed_fast, delta > 0)
```

`move_distance_mm_y` already ignores moves smaller than one microstep, so a residual inside the deadband results in no motion and no homing.

### Status line

```python
def status():
    y = z_from_m(current_M_yaxis) - z_from_m(0.0)
    print("STATUS x=%.4f y=%.4f x_homed=%d y_trusted=%d" % (current_M_xaxis, y, x_homed, y_trusted))
```

### Bug fix needed for position verification

`move_distance_mm_x` adds the requested distance to `current_M_xaxis` even when a limit switch stops the move early. Change it to count the steps actually taken and update the position from that count, as `move_distance_mm_y` does. Without this the host cannot detect a short X move.

## 2. Host link: `pico.py`

- **Timeout.** The loop in `exec` tests `start_time < deadline`, and `start_time` never changes, so it can never time out. Change the test to `time.time() < deadline` and raise `TimeoutError` instead of printing.
- **Errors.** Raise `RuntimeError` when the output contains `Traceback`, so a failed firmware call stops a scan.
- **Interrupt and timeout handling.** On `KeyboardInterrupt` or timeout inside `exec`: send Ctrl-C (`b"\x03"`) to stop the motion, then run `m.y_trusted = False; m.x_homed = False`, then re-raise. A move aborted mid-way never updates the tracked position, so both axes must be re-homed, and the flags make the next `goto` do that.
- **`status()`.** Run `m.status()`, find the `STATUS` line with a regex, return `dict(x, y, x_homed, y_trusted)`. Raise if the line is missing.
- **`exec(..., quiet=False)`.** Optional flag to suppress echoing, used by `status()`.

## 3. New host module: `stage.py`

```python
class XtalStage:
    def __init__(self, pico, xs, ys, center, tol=0.01): ...
    def xy(self, n): ...            # crystal number -> (x, y); ValueError outside 1..100
    def goto_xy(self, x, y): ...    # move_to_x, move_to_y, then verify
    def goto(self, n): ...
    def goto_center(self): ...
    def where(self): ...            # status plus nearest crystal
    def plan(self, xtals, keep_order=False): ...
    def scan(self, xtals, acquire, keep_order=False, is_done=None, dry_run=False): ...
```

- **`goto_xy`** calls `m.move_to_x`, then `m.move_to_y`, then reads `status()` and raises if either coordinate is more than `tol` (0.01 mm) from the target. This catches limit-switch stops and range clamping before any data is taken at a wrong position. Before moving, it prints a one-line warning when the firmware is going to home Y (untrusted, or target below current Y).
- **`plan`** returns the visiting order and the number of Y homings it implies, using `status()` for the starting point.
  - Default: drop duplicates, group by row, visit rows in ascending Y, and within each row sweep X starting from the end nearer to the current X. This order never moves Y backwards, so at most one homing is needed, before the first crystal.
  - `keep_order=True`: visit in the order given; every step to a lower row costs one homing.
- **`scan`** removes crystals for which `is_done(n)` is true, prints the plan (order, homing count), and unless `dry_run` is set does `goto(n)` then `acquire(n)` for each crystal.

The module has no dependency on the CAEN code. `plan` can be tested without hardware by passing a fake `pico` whose `status()` returns a fixed dict.

## 4. Notebook: `sipm_10x10_xtal_array_daq_v0.ipynb`

- **Upload path.** Cell 5 uploads `/home/emmo/proj/sipm/PiPico_MotorControls/Main.py`, which is a different directory from this repo (`/ssd1/data/sipm/PiPico_MotorControls`). The two files are identical today. Change the cell to upload `"Main.py"` relative to the notebook, or edits made here will not reach the Pico.
- **Geometry cell.** Replace `x0..x9`, `y0..y9` with lists and build the stage:

  ```python
  xs = [20.55, 19.05, 17.47, 15.97, 14.39, 12.89, 11.31, 9.81, 8.23, 6.73]
  ys = [44.65, 43.15, 41.57, 40.07, 38.49, 36.99, 35.41, 33.91, 32.33, 30.83]
  stage = XtalStage(pico, xs, ys, center=(13.61, 37.74))
  ```

- **Run configuration.** `notes` and `acq_time_fnam` contain the event count, so they must be rebuilt when it changes. Wrap cell 9 in `configure_run(n)` which sets the globals `nevents`, `notes`, `acq_time_fnam` (and creates `acq_log` if missing).
- **Acquisition.** `acquire_and_plot()` stays as it is. Add thin wrappers:

  ```python
  def hdf5_name(n):  return f"{today}_{notes}_xtal{n:03d}.hdf5"

  def acquire(n):
      global xtaln
      xtaln = f"xtal{n:03d}"
      acquire_and_plot()

  def scan(xtals, nevents=None, **kw):
      if nevents is not None: configure_run(nevents)
      stage.scan(xtals, acquire, is_done=lambda n: os.path.exists(hdf5_name(n)), **kw)

  FULL_ORDER = [n for r in range(9, -1, -1)
                  for n in (range(10*r + 10, 10*r, -1) if r % 2 else range(10*r + 1, 10*r + 11))]

  def full_scan(nevents=None, **kw):
      scan(FULL_ORDER, nevents, keep_order=True, **kw)
  ```

  `FULL_ORDER` is the present sequence (100 to 91, 81 to 90, 80 to 71, ..., 1 to 10), fixed so that every full scan visits crystals in the same order regardless of where the stage was parked.
- **Resume.** Because finished crystals are skipped by file name, re-running the same `scan(...)` or `full_scan(...)` call after an interruption continues where it stopped, and existing HDF5 files are not overwritten. To retake a crystal, delete or rename its file.
- **Plots.** With `doplot = True` a full scan puts 1700 figures in one cell output. Set `doplot = False` for scans.
- **Old cells.** Remove the 100 per-crystal cells once the new path has been checked.

## 5. Order of work and checks

1. `pico.py` fixes (timeout, error raising, interrupt handling, `status`).
2. Firmware flags, `move_to_x`, `move_to_y`, `status`, and the `move_distance_mm_x` fix. Bench checks:
   - after connect, `status()` shows both flags 0, and the first `goto` homes both axes;
   - `goto` to a higher row does not home; to the same row does not home and does not move Y; to a lower row homes;
   - after `calibrate('y')`, `y_trusted` is 0.
3. `stage.py`. With `dry_run=True`: the `FULL_ORDER` plan shows the expected sequence and no homing after the start; `scan([5, 95, 15])` is reordered to `95, 15, 5`; the same list with `keep_order=True` reports a homing before 95.
4. Notebook conversion, then a short real scan over a few crystals in two rows with a low event count. Interrupt it once and confirm the re-run resumes.
5. **Re-check the Y coordinates.** Until now the first move to a row came after `calibrate('y')`, whose reverse return leaves backlash in the mechanism. From now on it will always come after a fresh home. The same `y` numbers may therefore land at a slightly different height. Check one or two rows against the trigger rate before trusting a full scan, and shift `ys` if needed.

## Behaviour changes to expect

- The first `goto` after connecting or re-uploading homes both axes (several minutes).
- The first Y move after `calibrate('y')` homes Y. The stage is near home at that point, so this costs little more than the two 10 s stability waits.
- Interrupting a move costs a re-home of both axes on the next `goto`.

## Not included

- **Faster homing** (fast travel to about 2 mm above the switch, slow final approach). Left out because slow motion was made the default for reliability.
- **Keeping position across reconnects** (saving it on the Pico flash or skipping the re-import). Would remove the homing after each connect, at the cost of trusting a saved position.
- **One-directional X approach.** Only needed if X turns out to have a backlash problem too.
