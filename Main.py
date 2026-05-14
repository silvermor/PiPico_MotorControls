"""
main.py

SiPM Motor Controls for the reference detector

"""

import time
from machine import Pin
import math

# Pin assignments
led = Pin("LED", Pin.OUT)
dir_pin_y = Pin("GP1", Pin.OUT)   # Direction pin: 0 = forward, 1 = reverse
en_pin_y = Pin("GP0", Pin.OUT)    # Enable pin: 0 = enabled, 1 = disabled
pul_pin_y = Pin("GP2", Pin.OUT)
max_limit_y = Pin("GP9", Pin.IN, Pin.PULL_UP)    # High limit switch
min_limit_y = Pin("GP10", Pin.IN, Pin.PULL_UP)    # Low limit switch (Home)

dir_pin_x = Pin("GP5", Pin.OUT)   # Direction pin: 0 = forward, 1 = reverse
en_pin_x = Pin("GP4", Pin.OUT)    # Enable pin: 0 = enabled, 1 = disabled
pul_pin_x = Pin("GP6", Pin.OUT)
max_limit_x = Pin("GP14", Pin.IN, Pin.PULL_UP)    # High limit switch
min_limit_x = Pin("GP13", Pin.IN, Pin.PULL_UP)    # Low limit switch (Home)

# Motion constants
SLOW_STEP_DELAY = 3000  # microseconds
FAST_STEP_DELAY = 300  # microseconds
MM_PER_REV_Y = 1.25   # lead screw pitch in mm/rev
MM_PER_REV_X = 1   # lead screw pitch in mm/rev
SUBDIVISION = 8   # microstepping setting
STEP_ANGLE = 1.8 / SUBDIVISION  # degrees per step
MM_PER_STEP = MM_PER_REV_Y / (STEP_ANGLE/360)  # mm per microstep
STEPS_PER_REV = int(360 / STEP_ANGLE)  # steps per revolution, should be 1600
Y_HOME = 3.2        # mm, measured at M=0 using a feeler gauge
L = math.sqrt(10388)   #101.9215384     # mm, length of scissor arms

# Position tracking
current_M_yaxis = 0.000000
current_Z = 0.000000
M_max_y = None  # Set by calibrate(); was hardcoded 41.53
M_max_x = None  # Set by calibrate(); was hardcoded 65.64
current_M_xaxis = 0.000000

def enable_motor():
    en_pin_y.value(0)
    en_pin_x.value(0)


def disable_motor():
    en_pin_y.value(1)
    en_pin_x.value(1)


def safe_step_y(direction, speed_fast=False):
    """
    Perform one step in the given direction.
    Returns False if limit was hit before stepping, True if step executed.-
    """
    # Pick correct limit for direction
    limit_pin = max_limit_y if direction == 0 else min_limit_y
    if limit_pin.value() == 0:  # active-low
        print("Limit reached! Stopping.")
        time.sleep(0.05)  # debounce delay
        return False
    
    # Set direction
    dir_pin_y.value(direction)

    # Pulse
    delay_us = FAST_STEP_DELAY if speed_fast else SLOW_STEP_DELAY
    pul_pin_y.value(1)
    time.sleep_us(delay_us)
    pul_pin_y.value(0)
    time.sleep_us(delay_us)
    return True


def safe_step_x(direction, speed_fast=False):
    """
    Perform one step in the given direction.
    Returns False if limit was hit before stepping, True if step executed.-
    """
    # Pick correct limit for direction
    limit_pin = max_limit_x if direction == 0 else min_limit_x
    if limit_pin.value() == 0:  # active-low
        print("Limit reached! Stopping.")
        time.sleep(0.05)  # debounce delay
        return False
    
    # Set direction
    dir_pin_x.value(direction)

    # Pulse
    delay_us = FAST_STEP_DELAY if speed_fast else SLOW_STEP_DELAY
    pul_pin_x.value(1)
    time.sleep_us(delay_us)
    pul_pin_x.value(0)
    time.sleep_us(delay_us)
    return True


def move_until_limit_y(direction, limit_pin, speed_fast=False, backoff_slow=True):
    """
    Move in given direction until limit switch triggers,
    then back off slowly until switch releases.
    Returns net forward steps (forward - backoff).
    """
    enable_motor()
    time.sleep(0.01)  # brief pause

    opposite_dir = 1 if direction == 0 else 0
    forward_steps = 0
    backoff_steps = 0
    wait_s = 10

    # 1 Approach the limit
    while limit_pin.value() == 1:  # active-low
        if not safe_step_y(direction, speed_fast):
            break
        forward_steps += 1

    # 2. Press stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 0:  # still pressed
            break  # stable press
        print("Limit released during press check, re-approaching...")
        while limit_pin.value() == 1:  # re-approach until pressed
            if not safe_step_y(direction, speed_fast=False):
                break
            forward_steps += 1
        # loop will re-check after another wait

    # 3. Back off until released
    while limit_pin.value() == 0:  # still pressed
        if not safe_step_y(opposite_dir, speed_fast=False):
            break
        backoff_steps += 1

    # 4. Release stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 1:  # still released
            break  # stable release
        print("Limit pressed again during release check — backing off more...")
        while limit_pin.value() == 0:  # back off until released
            if not safe_step_y(opposite_dir, speed_fast=False):
                break
            backoff_steps += 1
        # loop will re-check after another wait

    time.sleep(0.01)  # brief pause
    # Return net steps moved
    net_steps = forward_steps - backoff_steps
    return net_steps


def move_until_limit_x(direction, limit_pin, speed_fast=False, backoff_slow=True):
    """
    Move in given direction until limit switch triggers,
    then back off slowly until switch releases.
    Returns net forward steps (forward - backoff).
    """
    enable_motor()
    time.sleep(0.01)  # brief pause

    opposite_dir = 1 if direction == 0 else 0
    forward_steps = 0
    backoff_steps = 0
    wait_s = 10

    # 1 Approach the limit
    while limit_pin.value() == 1:  # active-low
        if not safe_step_x(direction, speed_fast):
            break
        forward_steps += 1

    # 2. Press stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 0:  # still pressed
            break  # stable press
        print("Limit released during press check, re-approaching...")
        while limit_pin.value() == 1:  # re-approach until pressed
            if not safe_step_x(direction, speed_fast=False):
                break
            forward_steps += 1
        # loop will re-check after another wait

    # 3. Back off until released
    while limit_pin.value() == 0:  # still pressed
        if not safe_step_x(opposite_dir, speed_fast=False):
            break
        backoff_steps += 1

    # 4. Release stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 1:  # still released
            break  # stable release
        print("Limit pressed again during release check — backing off more...")
        while limit_pin.value() == 0:  # back off until released
            if not safe_step_x(opposite_dir, speed_fast=False):
                break
            backoff_steps += 1
        # loop will re-check after another wait

    time.sleep(0.01)  # brief pause
    # Return net steps moved
    net_steps = forward_steps - backoff_steps
    return net_steps


def home(axis):
    global current_M_yaxis, current_M_xaxis, current_Z
    print("Homing to MIN " + axis + " limit...")
    if axis == "y":
        net_steps = move_until_limit_y(1, min_limit_y, speed_fast=False)
        current_M_yaxis = 0.0
        current_Z = z_from_m(0.0)
    elif axis == "x":
        net_steps = move_until_limit_x(1, min_limit_x, speed_fast=False)
        current_M_xaxis = 0.0
    print(f"Moved a total of {net_steps} steps to Home position")
    print("Home set: M=0.00 mm")


def calibrate(axis):
    global current_M_yaxis, current_M_xaxis, M_max_y, M_max_x, current_Z

    if axis == "y":
        print("=== Calibrating Y axis ===")

        print("Step 1: Homing to MIN limit...")
        home("y")
        print(f"  Z_min = {z_from_m(0.0):.3f} mm")

        print("Step 2: Moving to MAX limit...")
        net_steps_travel = move_until_limit_y(0, max_limit_y, speed_fast=False)
        M_max_y = (net_steps_travel / STEPS_PER_REV) * MM_PER_REV_Y
        current_M_yaxis = M_max_y
        current_Z = z_from_m(M_max_y)
        print(f"  Z_max = {current_Z:.3f} mm  (M_max_y = {M_max_y:.4f} mm, {net_steps_travel} steps)")

        print("Step 3: Returning to MIN position by step count...")
        move_steps_y(net_steps_travel, speed_fast=True, forward=False)
        # current_M_yaxis = 0.0 # shouldnt be necessary. M here should be calculated back to 0 here
        current_Z = z_from_m(current_M_yaxis) 

        print(f"\nY calibration complete:")
        print(f"  Z range: {z_from_m(0.0):.3f} mm  →  {z_from_m(M_max_y):.3f} mm")
        print(f"  M range: 0.0000 mm  →  {M_max_y:.4f} mm  ({net_steps_travel} steps)")

    elif axis == "x":
        print("=== Calibrating X axis ===")

        print("Step 1: Homing to MIN limit...")
        home("x")

        print("Step 2: Moving to MAX limit...")
        net_steps_travel = move_until_limit_x(0, max_limit_x, speed_fast=False)
        M_max_x = (net_steps_travel / STEPS_PER_REV) * MM_PER_REV_X
        current_M_xaxis = M_max_x
        print(f"  M_max_x = {M_max_x:.4f} mm  ({net_steps_travel} steps)")

        print("Step 3: Returning to MIN position by step count...")
        move_steps_x(net_steps_travel, speed_fast=False, forward=False)
        # current_M_xaxis = 0.0 # shouldnt be necessary. M here should be calculated back to 0 here

        print(f"\nX calibration complete:")
        print(f"  M range: 0.0000 mm  →  {M_max_x:.4f} mm  ({net_steps_travel} steps)")

    else:
        print(f"Unknown axis '{axis}'. Use 'x' or 'y'.")


def move_steps_y(num_steps=1, speed_fast=False, forward=True):
    global current_M_yaxis
    actual_steps_moved = -1
    direction = 0 if forward else 1
    enable_motor()
    time.sleep(0.04)  # brief pause to stabilize
    for p in range(num_steps):
        if not safe_step_y(direction, speed_fast):
            actual_steps_moved = p
            break
    if (actual_steps_moved == -1):
        actual_steps_moved = num_steps
    time.sleep(0.005)  # brief pause
    #disable_motor()
    time.sleep(0.01)  # brief pause
    dist_mm = (actual_steps_moved / STEPS_PER_REV) * MM_PER_REV_Y
    current_M_yaxis += dist_mm if forward else -dist_mm
    print(f"Moved {'FWD' if forward else 'REV'} {num_steps} steps ({dist_mm:.4f} mm), M y axis={current_M_yaxis:.4f}")


def move_steps_x(num_steps=1, speed_fast=False, forward=True):
    global current_M_xaxis
    actual_steps_moved = -1
    direction = 0 if forward else 1
    enable_motor()
    time.sleep(0.04)  # brief pause to stabilize
    for p in range(num_steps):
        if not safe_step_x(direction, speed_fast):
            actual_steps_moved = p
            break
    if (actual_steps_moved == -1):
        actual_steps_moved = num_steps
    time.sleep(0.005)  # brief pause
    #disable_motor()
    time.sleep(0.01)  # brief pause
    dist_mm = (actual_steps_moved / STEPS_PER_REV) * MM_PER_REV_X
    current_M_xaxis += dist_mm if forward else -dist_mm
    print(f"Moved {'FWD' if forward else 'REV'} {num_steps} steps ({dist_mm:.4f} mm), M x axis={current_M_xaxis:.4f}")


def move_distance_mm_y(dist_mm, speed_fast=False, forward=True):
    global current_M_yaxis, current_Z
    
    # Determine current Z from current M
    current_Z = z_from_m(current_M_yaxis)
    prev_Z = current_Z
    # Compute target Z
    if forward:
        target_Z = current_Z + dist_mm
    else:
        target_Z = current_Z - dist_mm
    # Clamp physical limits
    Z_min = z_from_m(0)
    if target_Z < Z_min:
        print(f"WARNING: requested ΔZ would go below Z_min ({Z_min:.3f} mm). Clamping.")
        target_Z = Z_min
    if M_max_y is None:
        print("WARNING: M_max_y not calibrated — upper limit not enforced. Run 'calibrate y' first.")
    else:
        Z_max = z_from_m(M_max_y)
        if target_Z > Z_max:
            print(f"WARNING: requested ΔZ would exceed Z_max ({Z_max:.3f} mm). Clamping.")
            target_Z = Z_max
    
    # Convert target Z -> target M
    target_M = m_from_z(target_Z)

    # Compute delta M (screw travel)
    delta_M = target_M - current_M_yaxis

    # If delta_M is tiny, do nothing
    if abs(delta_M) < 0.00078125:  # corresponds to 1 microstep
        print("No move: at limit or zero displacement.")
        return
    
    # Convert delta M to steps
    steps = round((abs(delta_M) / MM_PER_REV_Y) * STEPS_PER_REV)
    direction = 0 if delta_M > 0 else 1

    # Execute the motion
    enable_motor()
    moved_steps = 0
    for p in range(steps):
        if not safe_step_y(direction, speed_fast):
            moved_steps = p
            print(f"Expected to move {steps} steps, but only moved {moved_steps}.")
            break
        moved_steps = p + 1
    time.sleep(0.01)  # brief pause

    # Update M and Z based on actual steps moved
    actual_delta_M = (moved_steps / STEPS_PER_REV) * MM_PER_REV_Y
    if delta_M < 0:
        actual_delta_M = -actual_delta_M

    current_M_yaxis += actual_delta_M
    current_Z = z_from_m(current_M_yaxis)

    # Report
    print(f"Requested ΔZ = {dist_mm:.3f} mm {'FWD' if forward else 'REV'}")
    print(f"Actual ΔZ    = {current_Z - prev_Z:.3f} mm")
    print(f"Moved ΔM = {actual_delta_M:.4f} mm → New Z = {current_Z:.3f} mm")
    print(f"Steps moved: {moved_steps}")


def move_distance_mm_x(dist_mm, speed_fast=False, forward=True):
    global current_M_xaxis
    if M_max_x is not None:
        if forward:
            dist_mm = min(M_max_x - current_M_xaxis, dist_mm)
        else:
            dist_mm = min(dist_mm, current_M_xaxis)
    if dist_mm <= 0:
        print("No move: at limit.")
        return
    distance_in_x = dist_mm
    steps = round((distance_in_x / MM_PER_REV_X) * STEPS_PER_REV)
    direction = 0 if forward else 1
    enable_motor()
    for p in range(steps):
        if not safe_step_x(direction, speed_fast):
            break
    
    time.sleep(0.005)  # brief pause
    #disable_motor()
    time.sleep(0.005)  # brief pause
    current_M_xaxis += distance_in_x if forward else -distance_in_x
    print(f"Moved in x {'FWD' if forward else 'REV'} {dist_mm:.2f} mm, M x axis={current_M_xaxis:.2f}")


def z_from_m(M):
    """Convert screw travel Z to platform height M using scissor geometry."""
    X =  98 - M - Y_HOME #98.3725 97.4099999
    Z = math.sqrt(L*L - (X*X))
    return Z


def m_from_z(Z):
    """Convert platform height M to screw travel Z using scissor geometry."""
    X = math.sqrt(L*L - (Z*Z))
    M = 98 - X - Y_HOME #98.3725 97.4099999
    return M


def test_axis_backlash(axis = "y", direction = 1, speed_fast = False):
    opposite_dir = 1 if direction == 0 else 0
    if axis == "y":
        safe_step = safe_step_y
        limit_pin = min_limit_y if direction == 1 else max_limit_y
        mm_per_rev = MM_PER_REV_Y
    elif axis == "x":
        safe_step = safe_step_x
        limit_pin = min_limit_x if direction == 1 else max_limit_x
        mm_per_rev = MM_PER_REV_X
    else:
        print(f"Unknown axis '{axis}'. Use 'x' or 'y'.")
        return

    # move until limit
    while limit_pin.value() == 1:
        safe_step(direction, speed_fast)
    # backoff from limit
    for p in range(2400):
        if not safe_step(opposite_dir, speed_fast):
            break
    # now we check the consistency of the limit switch
    loops = []
    release_loops = []
    for loop in range(20):
        # approach the limit switch
        actual_steps = 0
        for p in range(3200):
            if not safe_step(direction, speed_fast):
                actual_steps = p
                break
            actual_steps = p + 1
        approach_mm = (actual_steps / STEPS_PER_REV) * mm_per_rev
        print(f"Approach {loop + 1}, steps to limit: {actual_steps} ({approach_mm:.4f} mm)")
        loops.append(actual_steps)
        time.sleep(1)
        # back off, tracking when the limit switch releases
        actual_steps = 0
        release_steps = 0
        for p in range(2400):
            if not safe_step(opposite_dir, speed_fast):
                actual_steps = p
                break
            actual_steps = p + 1
            if release_steps == 0 and limit_pin.value() == 1:
                release_steps = actual_steps
        backoff_mm = (actual_steps / STEPS_PER_REV) * mm_per_rev
        release_mm = (release_steps / STEPS_PER_REV) * mm_per_rev
        print(f"Backoff {loop + 1}, steps backed off: {actual_steps} ({backoff_mm:.4f} mm), limit released at step: {release_steps} ({release_mm:.4f} mm)")
        release_loops.append(release_steps)
    average_steps = sum(loops) / len(loops)
    median_steps = sorted(loops)[len(loops) // 2]
    std_steps = math.sqrt(sum((s - average_steps) ** 2 for s in loops) / len(loops))
    print(f"Average steps to hit limit: {average_steps:.2f} ({(average_steps / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    print(f"Median steps to hit limit: {median_steps} ({(median_steps / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    print(f"Std dev steps to hit limit: {std_steps:.2f} ({(std_steps / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    average_release = sum(release_loops) / len(release_loops)
    median_release = sorted(release_loops)[len(release_loops) // 2]
    std_release = math.sqrt(sum((s - average_release) ** 2 for s in release_loops) / len(release_loops))
    print(f"Average steps to release limit: {average_release:.2f} ({(average_release / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    print(f"Median steps to release limit: {median_release} ({(median_release / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    print(f"Std dev steps to release limit: {std_release:.2f} ({(std_release / STEPS_PER_REV) * mm_per_rev:.4f} mm)")
    

def repl():
    print("Manual command mode. Type 'help' for options.")
    while True:
        cmd = input(">> ").strip().lower()
        if cmd == "exit":
            print("Exiting REPL.")
            break
        elif cmd == "help":
            print("Commands:")
            print("  home <axis>")
            print("  calibrate <axis>            # travel MIN→MAX→MIN, sets M_max")
            print("  move <axis> <steps> [rev] [fast]")
            print("  movemm <axis> <mm> [rev|fwd] [fast]")
            print("  mul <axis> <fwd|rev> [fast|slow]   # move until limit")
            print("  backlash <axis> [direction] [fast|slow]  # test limit switch repeatability")
            print("  z")
            print("  exit")
        elif cmd.startswith("home"):
            try:
                parts = cmd.split()
                axis = parts[1]
                home(axis)
            except Exception:
                print("Usage: home <axis>")
        elif cmd.startswith("calibrate"):
            try:
                parts = cmd.split()
                axis = parts[1]
                calibrate(axis)
            except Exception:
                print("Usage: calibrate <axis>")
        elif cmd.startswith("move "):
            try:
                parts = cmd.split()
                axis = parts[1]
                steps = int(parts[2])
                forward = True
                speed_fast = False
                if len(parts) > 3:
                    if parts[3] == "rev":
                        forward = False
                    elif parts[4] == "fast":
                        speed_fast = True
                if len(parts) > 4 and parts[4] == "fast":
                    speed_fast = True
                if (axis == "y"):
                    move_steps_y(steps, speed_fast, forward)
                elif (axis == "x"):
                    move_steps_x(steps, speed_fast, forward)
            except Exception:
                print("Usage: move <axis> <steps> [rev] [fast]")
        elif cmd.startswith("movemm "):
            try:
                parts = cmd.split()
                axis = parts[1]
                mm = float(parts[2])
                forward = True
                speed_fast = False
                if len(parts) > 3:
                    if parts[3] == "rev":
                        forward = False
                    elif parts[3] == "fast":
                        speed_fast = True
                if len(parts) > 4 and parts[4] == "fast":
                    speed_fast = True
                if (axis == "y"):
                    move_distance_mm_y(mm, speed_fast, forward)
                elif (axis == "x"):
                    move_distance_mm_x(mm, speed_fast, forward)
            except Exception:
                print("Usage: movemm <axis> <mm> [rev] [fast]")
        elif cmd.startswith("mul "):
            try:
                parts = cmd.split()
                if len(parts) < 3:
                    print("Usage: mul <axis> <fwd|rev> [fast|slow]")
                    continue
                axis = parts[1]
                forward = True if parts[2] == "fwd" else False
                speed_fast = True
                if len(parts) > 3:
                    if parts[3] == "slow":
                        speed_fast = False
                    elif parts[3] == "fast":
                        speed_fast = True
                direction = 0 if forward else 1
                if (axis == "x"):
                    limit_pin = max_limit_x if forward else min_limit_x
                    net_steps = move_until_limit_x(direction, limit_pin, speed_fast=speed_fast)
                elif (axis == "y"):
                    limit_pin = max_limit_y if forward else min_limit_y
                    net_steps = move_until_limit_y(direction, limit_pin, speed_fast=speed_fast)
                dist_mm = (net_steps / STEPS_PER_REV) * MM_PER_REV_Y
                global current_M_yaxis, current_M_xaxis
                if (axis == "x"):
                    current_M_xaxis += dist_mm if forward else -dist_mm
                elif (axis == "y"):
                    current_M_yaxis += dist_mm if forward else -dist_mm
                print(f"Moved until limit: {net_steps} net steps ({dist_mm:.2f} mm), M={current_M_xaxis:.2f}")
            except Exception:
                print("Usage: mul <axis> <fwd|rev> [fast|slow]")
        elif cmd.startswith("backlash"):
            try:
                parts = cmd.split()
                axis = parts[1] if len(parts) > 1 else "y"
                direction = int(parts[2]) if len(parts) > 2 else 1
                speed_fast = len(parts) > 3 and parts[3] == "fast"
                test_axis_backlash(axis, direction, speed_fast)
            except Exception:
                print("Usage: backlash <axis> [direction] [fast|slow]")
        elif cmd == "z":
            print(f"Current Z height: {z_from_m(current_M_yaxis):.6f} mm")
        else:
            print("Unknown command. Type 'help'.")

if __name__ == "__main__":
    # Simple REPL for manual command entry
    # Start REPL after running the script
    repl()
    disable_motor()
