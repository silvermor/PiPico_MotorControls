import time
from machine import Pin
import math

# Pin assignments
led = Pin("LED", Pin.OUT)
dir_pin = Pin("GP0", Pin.OUT)   # Direction pin: 0 = forward, 1 = reverse
en_pin = Pin("GP1", Pin.OUT)    # Enable pin: 0 = enabled, 1 = disabled
pul_pin = Pin("GP2", Pin.OUT)
max_limit = Pin("GP14", Pin.IN, Pin.PULL_UP)    # High limit switch
min_limit = Pin("GP15", Pin.IN, Pin.PULL_UP)    # Low limit switch (Home)

# Motion constants
SLOW_STEP_DELAY = 3000  # microseconds
FAST_STEP_DELAY = 300  # microseconds
MM_PER_REV = 1.25   # lead screw pitch in mm/rev
SUBDIVISION = 8   # microstepping setting
STEP_ANGLE = 1.8 / SUBDIVISION  # degrees per step
MM_PER_STEP = MM_PER_REV / (STEP_ANGLE/360)  # mm per microstep
STEPS_PER_REV = int(360 / STEP_ANGLE)  # steps per revolution
Y_HOME = 3.3        # mm, measured at M=0
L = 98.0          # mm, length of scissor arms


# Position tracking
current_M = 0.000000
current_Z = 0.000000
M_max = None

def enable_motor():
    en_pin.value(0)

def disable_motor():
    en_pin.value(1)

def safe_step(direction, speed_fast=False):
    """
    Perform one step in the given direction.
    Returns False if limit was hit before stepping, True if step executed.
    """
    # Pick correct limit for direction
    limit_pin = max_limit if direction == 0 else min_limit
    if limit_pin.value() == 0:  # active-low
        print("Limit reached! Stopping.")
        time.sleep(0.05)  # debounce delay
        return False
    
    # Set direction
    dir_pin.value(direction)

    # Pulse
    delay_us = FAST_STEP_DELAY if speed_fast else SLOW_STEP_DELAY
    pul_pin.value(1)
    time.sleep_us(delay_us)
    pul_pin.value(0)
    time.sleep_us(delay_us)
    return True

def move_until_limit(direction, limit_pin, speed_fast=True, backoff_slow=True):
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
        if not safe_step(direction, speed_fast):
            break
        forward_steps += 1

    # 2. Press stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 0:  # still pressed
            break  # stable press
        print("Limit released during press check, re-approaching...")
        while limit_pin.value() == 1:  # re-approach until pressed
            if not safe_step(direction, speed_fast=False):
                break
            forward_steps += 1
        # loop will re-check after another wait

    # 3. Back off until released
    while limit_pin.value() == 0:  # still pressed
        if not safe_step(opposite_dir, speed_fast=False):
            break
        backoff_steps += 1

    # 4. Release stability check
    while True:
        time.sleep(wait_s)
        if limit_pin.value() == 1:  # still released
            break  # stable release
        print("Limit pressed again during release check — backing off more...")
        while limit_pin.value() == 0:  # back off until released
            if not safe_step(opposite_dir, speed_fast=False):
                break
            backoff_steps += 1
        # loop will re-check after another wait

    time.sleep(0.01)  # brief pause
    # Return net steps moved
    net_steps = forward_steps - backoff_steps
    return net_steps

def home():
    global current_M
    print("Homing to MIN limit...")
    net_steps = move_until_limit(1, min_limit, speed_fast=True)
    current_M = 0.0

    print(f"Moved a total of {net_steps} steps to Home position")
    print("Home set: M=0.00 mm")

def calibrate():
    global current_M, M_max
    home()
    print("Traversing to MAX limit...")
    net_steps = move_until_limit(0, max_limit, speed_fast=True)
    M_max = (net_steps / STEPS_PER_REV) * MM_PER_REV
    current_M = M_max
    print(f"Calibration complete: M_max = {M_max:.2f} mm")
    print("Returning home...")
    home()

def move_steps(num_steps=1, speed_fast=False, forward=True):
    global current_M
    direction = 0 if forward else 1
    enable_motor()
    time.sleep(0.04)  # brief pause to stabilize
    for p in range(num_steps):
        if not safe_step(direction, speed_fast):
            break
    time.sleep(0.005)  # brief pause
    #disable_motor()
    time.sleep(0.01)  # brief pause
    dist_mm = (num_steps / STEPS_PER_REV) * MM_PER_REV
    current_M += dist_mm if forward else -dist_mm
    print(f"Moved {'FWD' if forward else 'REV'} {num_steps} steps ({dist_mm:.2f} mm), M={current_M:.2f}")

def move_distance_mm(dist_mm, forward=True, speed_fast=True):
    global current_M
    if M_max is not None:
        if forward:
            dist_mm = min(dist_mm, M_max - current_M)
        else:
            dist_mm = min(dist_mm, current_M)
    if dist_mm <= 0:
        print("No move: at limit.")
        return
    steps = int((dist_mm / MM_PER_REV) * STEPS_PER_REV)
    direction = 0 if forward else 1
    enable_motor()
    for p in range(steps):
        if not safe_step(direction, speed_fast):
            break
    time.sleep(0.005)  # brief pause
    #disable_motor()
    time.sleep(0.005)  # brief pause
    current_M += dist_mm if forward else -dist_mm
    print(f"Moved {'FWD' if forward else 'REV'} {dist_mm:.2f} mm, M={current_M:.2f}")

def z_from_m(M):
    """Convert screw travel M to platform height Z using scissor geometry."""
    X = 98.000 - M - Y_HOME
    Z = math.sqrt(L*L - (X*X))
    return Z

def repl():
    print("Manual command mode. Type 'help' for options.")
    while True:
        cmd = input(">> ").strip().lower()
        if cmd == "exit":
            print("Exiting REPL.")
            break
        elif cmd == "help":
            print("Commands:")
            print("  home")
            print("  calibrate")
            print("  move <steps> [rev] [fast]")
            print("  mul <fwd|rev> [fast|slow]   # move until limit")
            print("  z")
            print("  exit")
        elif cmd == "home":
            home()
        elif cmd == "calibrate":
            calibrate()
        elif cmd.startswith("move "):
            try:
                parts = cmd.split()
                steps = int(parts[1])
                forward = True
                speed_fast = False
                if len(parts) > 2:
                    if parts[2] == "rev":
                        forward = False
                    elif parts[2] == "fast":
                        speed_fast = True
                if len(parts) > 3 and parts[3] == "fast":
                    speed_fast = True
                move_steps(steps, speed_fast, forward)
            except Exception:
                print("Usage: move <steps> [rev] [fast]")
        elif cmd.startswith("mul "):
            try:
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: mul <fwd|rev> [fast|slow]")
                    continue
                forward = True if parts[1] == "fwd" else False
                speed_fast = True
                if len(parts) > 2:
                    if parts[2] == "slow":
                        speed_fast = False
                    elif parts[2] == "fast":
                        speed_fast = True
                direction = 0 if forward else 1
                limit_pin = max_limit if forward else min_limit
                net_steps = move_until_limit(direction, limit_pin, speed_fast=speed_fast)
                dist_mm = (net_steps / STEPS_PER_REV) * MM_PER_REV
                global current_M
                current_M += dist_mm if forward else -dist_mm
                print(f"Moved until limit: {net_steps} net steps ({dist_mm:.2f} mm), M={current_M:.2f}")
            except Exception:
                print("Usage: mul <fwd|rev> [fast|slow]")
        elif cmd == "z":
            print(f"Current Z height: {z_from_m(current_M):.6f} mm")
        else:
            print("Unknown command. Type 'help'.")


if __name__ == "__main__":
    # Simple REPL for manual command entry
    # Start REPL after running the script
    repl()
    disable_motor()