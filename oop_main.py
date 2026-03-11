# Do not use this yet.
# Not fully working yet. The Y motor movement is wrong. movemm y 5 moves about 3mm instead of 5mm.

# Object Oriented version of the motor control

import time
from machine import Pin
import math


class Motor:
    def __init__(self, dir_pin, en_pin, pulse_pin, max_limit_pin, min_limit_pin, mm_per_rev, vertical=False):
        self.direction = Pin(dir_pin, Pin.OUT)
        self.enable = Pin(en_pin, Pin.OUT)
        self.pulse = Pin(pulse_pin, Pin.OUT)
        self.max_limit = Pin(max_limit_pin, Pin.IN, Pin.PULL_UP)
        self.min_limit = Pin(min_limit_pin, Pin.IN, Pin.PULL_UP)
        self.mm_per_rev = mm_per_rev
        self.vertical = vertical
        self.position_mm = 0.0
        self.subdivision = 8
        self.SLOW_STEP_DELAY = 3000  # microseconds
        self.FAST_STEP_DELAY = 300  # microseconds
        self.STEP_ANGLE = 1.8 / self.subdivision  # degrees per microstep
        self.STEPS_PER_REV = int(360 / self.STEP_ANGLE)
        self.MM_PER_STEP = self.mm_per_rev / self.STEPS_PER_REV

    def enable_motor(self):
        self.enable.value(0)

    def disable_motor(self):
        self.enable.value(1)

    def safe_step(self, direction, speed_fast=False):
        """Perform one microstep in the given direction.
        Returns False if a limit was hit before stepping, True if step executed.
        """
        limit_pin = self.max_limit if direction == 0 else self.min_limit
        if limit_pin.value() == 0:  # active-low
            print("Limit reached! Stopping.")
            time.sleep(0.05)
            return False

        # set direction
        self.direction.value(direction)

        delay_us = self.FAST_STEP_DELAY if speed_fast else self.SLOW_STEP_DELAY
        self.pulse.value(1)
        time.sleep_us(delay_us)
        self.pulse.value(0)
        time.sleep_us(delay_us)
        return True

    def move_until_limit(self, direction, limit_pin, speed_fast=True):
        """Move in given direction until `limit_pin` triggers, then back off slowly until released.
        Returns net forward steps (forward - backoff).
        """
        self.enable_motor()
        time.sleep(0.01)

        opposite_dir = 1 if direction == 0 else 0
        forward_steps = 0
        backoff_steps = 0
        wait_s = 10

        # approach limit
        while limit_pin.value() == 1:
            if not self.safe_step(direction, speed_fast):
                break
            forward_steps += 1

        # press stability check
        while True:
            time.sleep(wait_s)
            if limit_pin.value() == 0:
                break
            print("Limit released during press check, re-approaching...")
            while limit_pin.value() == 1:
                if not self.safe_step(direction, speed_fast=False):
                    break
                forward_steps += 1

        # back off until released
        while limit_pin.value() == 0:
            if not self.safe_step(opposite_dir, speed_fast=False):
                break
            backoff_steps += 1

        # release stability check
        while True:
            time.sleep(wait_s)
            if limit_pin.value() == 1:
                break
            print("Limit pressed again during release check — backing off more...")
            while limit_pin.value() == 0:
                if not self.safe_step(opposite_dir, speed_fast=False):
                    break
                backoff_steps += 1

        time.sleep(0.01)
        return forward_steps - backoff_steps

    def move_steps(self, num_steps=1, speed_fast=False, forward=True):
        actual_steps_moved = -1
        direction = 0 if forward else 1
        self.enable_motor()
        time.sleep(0.04)
        for p in range(num_steps):
            if not self.safe_step(direction, speed_fast):
                actual_steps_moved = p
                break
        if actual_steps_moved == -1:
            actual_steps_moved = num_steps
        time.sleep(0.015)
        dist_mm = (actual_steps_moved / self.STEPS_PER_REV) * self.mm_per_rev
        self.position_mm += dist_mm if forward else -dist_mm
        print(f"Moved {'FWD' if forward else 'REV'} {num_steps} steps ({dist_mm:.2f} mm), position={self.position_mm:.2f}")

    def move_distance_mm(self, dist_mm, speed_fast=True, forward=True):
        # generic linear move for axes without extra geometry (X motor uses this)
        if dist_mm <= 0:
            print("No move: distance <= 0")
            return
        steps = round((dist_mm / self.mm_per_rev) * self.STEPS_PER_REV)
        direction = 0 if forward else 1
        self.enable_motor()
        for _ in range(steps):
            if not self.safe_step(direction, speed_fast):
                break
        time.sleep(0.015)
        self.position_mm += dist_mm if forward else -dist_mm
        print(f"Moved {'FWD' if forward else 'REV'} {dist_mm:.2f} mm, position={self.position_mm:.2f}")


class ScissorPlatform:
    def __init__(self, motor_x: Motor, motor_y: Motor):
        self.motor_x = motor_x
        self.motor_y = motor_y
        # geometry constants
        self.Y_HOME = 3.3
        self.L = 101.9215384
        self.X_BASE = 98.0
        self.M_max_y = 41.53
        self.M_max_x = 65.64

    def z_from_m(self, M):
        X = self.X_BASE - M - self.Y_HOME
        Z = math.sqrt(self.L * self.L - (X * X))
        return Z

    def m_from_z(self, Z):
        X = math.sqrt(self.L * self.L - (Z * Z))
        M = self.X_BASE - X - self.Y_HOME
        return M

    def home(self, axis):
        print("Homing to MIN " + axis + " limit...")
        if axis == "y":
            net_steps = self.motor_y.move_until_limit(1, self.motor_y.min_limit, speed_fast=True)
            self.motor_y.position_mm = 0.0
        elif axis == "x":
            net_steps = self.motor_x.move_until_limit(1, self.motor_x.min_limit, speed_fast=True)
            self.motor_x.position_mm = 0.0
        else:
            print("Unknown axis for home")
            return
        print(f"Moved a total of {net_steps} steps to Home position")
        print("Home set: M=0.00 mm")

    def move_distance_mm_y(self, dist_mm, speed_fast=True, forward=True):
        # dist_mm is platform vertical travel (Z) in original script
        # apply M_max_y guard
        current_M = self.motor_y.position_mm
        if self.M_max_y is not None:
            if forward:
                dist_mm = min(dist_mm, self.z_from_m(self.M_max_y) - self.z_from_m(current_M))
            else:
                dist_mm = min(dist_mm, self.z_from_m(current_M))
        if dist_mm <= 0:
            print("No move: at limit.")
            return
        if forward:
            distance_in_m = self.m_from_z(self.z_from_m(current_M) + dist_mm) - current_M
        else:
            distance_in_m = current_M - self.m_from_z(self.z_from_m(current_M) - dist_mm)
        steps = round((distance_in_m / self.motor_y.mm_per_rev) * self.motor_y.STEPS_PER_REV)
        direction = 0 if forward else 1
        self.motor_y.enable_motor()
        for _ in range(steps):
            if not self.motor_y.safe_step(direction, speed_fast):
                break
        time.sleep(0.015)
        self.motor_y.position_mm += distance_in_m if forward else -distance_in_m
        print(f"Moved in y {'FWD' if forward else 'REV'} {dist_mm:.2f} mm, Z={self.z_from_m(self.motor_y.position_mm):.2f}")

    def move_distance_mm_x(self, dist_mm, speed_fast=True, forward=True):
        current_M = self.motor_x.position_mm
        if self.M_max_x is not None:
            if forward:
                dist_mm = min(self.M_max_x - current_M, dist_mm)
            else:
                dist_mm = min(dist_mm, current_M)
        if dist_mm <= 0:
            print("No move: at limit.")
            return
        distance_in_m = dist_mm
        steps = round((distance_in_m / self.motor_x.mm_per_rev) * self.motor_x.STEPS_PER_REV)
        direction = 0 if forward else 1
        self.motor_x.enable_motor()
        for _ in range(steps):
            if not self.motor_x.safe_step(direction, speed_fast):
                break
        time.sleep(0.015)
        self.motor_x.position_mm += distance_in_m if forward else -distance_in_m
        print(f"Moved in x {'FWD' if forward else 'REV'} {dist_mm:.2f} mm, M x axis={self.motor_x.position_mm:.2f}")

    def automatedGridMovement(self):
        for i in range(8):
            self.home("x")
            for j in range(8):
                self.move_distance_mm_x(3, True, True)
                # add waiting/collection here if needed
            self.move_distance_mm_y(3, True, True)

    def repl(self):
        print("Manual command mode. Type 'help' for options.")
        while True:
            try:
                cmd = input(">> ").strip().lower()
            except Exception:
                cmd = ""
            if cmd == "exit":
                print("Exiting REPL.")
                break
            elif cmd == "help":
                print("Commands:")
                print("  home <x|y>")
                print("  move <axis> <steps> [rev] [fast]")
                print("  mul <axis> <fwd|rev> [fast|slow]   # move until limit")
                print("  movemm <axis> <mm> [rev] [fast]")
                print("  z")
                print("  exit")
            elif cmd.startswith("home"):
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: home <axis>")
                    continue
                axis = parts[1]
                self.home(axis)
            elif cmd.startswith("move "):
                parts = cmd.split()
                try:
                    axis = parts[1]
                    steps = int(parts[2])
                    forward = True
                    speed_fast = False
                    if len(parts) > 3 and parts[3] == "rev":
                        forward = False
                    if "fast" in parts:
                        speed_fast = True
                    if axis == "y":
                        self.motor_y.move_steps(steps, speed_fast, forward)
                    elif axis == "x":
                        self.motor_x.move_steps(steps, speed_fast, forward)
                except Exception:
                    print("Usage: move <axis> <steps> [rev] [fast]")
            elif cmd.startswith("movemm "):
                parts = cmd.split()
                try:
                    axis = parts[1]
                    mm = float(parts[2])
                    forward = True
                    speed_fast = False
                    if len(parts) > 3 and parts[3] == "rev":
                        forward = False
                    if "fast" in parts:
                        speed_fast = True
                    if axis == "y":
                        self.move_distance_mm_y(mm, speed_fast, forward)
                    elif axis == "x":
                        self.move_distance_mm_x(mm, speed_fast, forward)
                except Exception:
                    print("Usage: movemm <axis> <mm> [rev] [fast]")
            elif cmd.startswith("mul "):
                parts = cmd.split()
                if len(parts) < 3:
                    print("Usage: mul <axis> <fwd|rev> [fast|slow]")
                    continue
                axis = parts[1]
                forward = True if parts[2] == "fwd" else False
                speed_fast = True
                if len(parts) > 3 and parts[3] == "slow":
                    speed_fast = False
                direction = 0 if forward else 1
                try:
                    if axis == "x":
                        limit_pin = self.motor_x.max_limit if forward else self.motor_x.min_limit
                        net_steps = self.motor_x.move_until_limit(direction, limit_pin, speed_fast=speed_fast)
                    elif axis == "y":
                        limit_pin = self.motor_y.max_limit if forward else self.motor_y.min_limit
                        net_steps = self.motor_y.move_until_limit(direction, limit_pin, speed_fast=speed_fast)
                    else:
                        print("Unknown axis")
                        continue
                    dist_mm = (net_steps / self.motor_y.STEPS_PER_REV) * self.motor_y.mm_per_rev
                    if axis == "x":
                        self.motor_x.position_mm += dist_mm if forward else -dist_mm
                    elif axis == "y":
                        self.motor_y.position_mm += dist_mm if forward else -dist_mm
                    print(f"Moved until limit: {net_steps} net steps ({dist_mm:.2f} mm)")
                except Exception:
                    print("Usage: mul <axis> <fwd|rev> [fast|slow]")
            elif cmd == "z":
                print(f"Current Z height: {self.z_from_m(self.motor_y.position_mm):.6f} mm")
            else:
                print("Unknown command. Type 'help'.")


# Instantiate motors and platform using the same pin names as in Main.py
motor_y = Motor("GP1", "GP0", "GP2", "GP9", "GP10", mm_per_rev=1.25, vertical=True)
motor_x = Motor("GP5", "GP4", "GP6", "GP14", "GP13", mm_per_rev=1.0, vertical=False)
platform = ScissorPlatform(motor_x, motor_y)

if __name__ == "__main__":
    platform.repl()
