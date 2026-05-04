"""
pico_controller.py

Host-side serial interface to the Raspberry Pi Pico running Main.py (MicroPython).

The Pico's Main.py exposes a REPL-style command interface over its USB CDC serial port.
This module sends those same text commands from the host and reads back responses,
so Jupyter / plain Python can drive the motors without VS Code.

Command protocol (mirrors Main.py repl()):
  home <axis>
  move <axis> <steps> [rev] [fast]
  movemm <axis> <mm> [rev] [fast]
  mul <axis> <fwd|rev> [fast|slow]
  z

Usage:
    from pico.pico_controller import PicoController
    pico = PicoController()          # auto-detects port, or pass port="/dev/ttyACM0"
    pico.connect()
    pico.home("x")
    pico.home("y")
    pico.move_mm("x", 10.0)
    pico.move_mm("y", 5.0, forward=False)
    pico.get_z()
    pico.disconnect()
"""

import serial
import serial.tools.list_ports
import time
import re
from typing import Optional


# ── port auto-detection ───────────────────────────────────────────────────────

def find_pico_port() -> Optional[str]:
    """
    Scan available serial ports and return the first one that looks like a
    Raspberry Pi Pico (USB VID 0x2E8A).  Returns None if not found.
    """
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x2E8A:          # Raspberry Pi VID
            return port.device
        # Fallback: name heuristic (ttyACM* on Linux, COM* on Windows)
        if "ACM" in (port.device or "") or "usbmodem" in (port.device or ""):
            return port.device
    return None


# ── main class ────────────────────────────────────────────────────────────────

class PicoController:
    """
    Controls the Raspberry Pi Pico stepper-motor controller over USB serial.

    Parameters
    ----------
    port : str, optional
        Serial port path (e.g. '/dev/ttyACM0', 'COM3').
        If None (default), auto-detection is attempted.
    baud : int
        Baud rate.  MicroPython's USB CDC is baud-rate-agnostic, but
        115200 is conventional.
    timeout : float
        Per-read timeout in seconds.
    response_timeout : float
        How long to wait for a command to finish (homing can take a while).
    """

    PROMPT = ">>"

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 115200,
        timeout: float = 0.5,
        response_timeout: float = 120.0,
    ):
        self.port = port or find_pico_port()
        self.baud = baud
        self.timeout = timeout
        self.response_timeout = response_timeout
        self._ser: Optional[serial.Serial] = None

        # Mirrored position state (updated after every move reply)
        self.current_M_x: float = 0.0
        self.current_M_y: float = 0.0
        self.current_Z: float = 0.0

    # ── connection ────────────────────────────────────────────────────────────

    def connect(self) -> None:
        if self.port is None:
            raise RuntimeError(
                "No Pico port found.  Is the Pico plugged in?  "
                "You can pass port= explicitly."
            )
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.5)         # let MicroPython settle after DTR toggle
        self._flush_input()
        # Nudge the REPL to get a prompt
        self._ser.write(b"\r\n")
        time.sleep(0.3)
        self._flush_input()
        print(f"[PicoController] Connected on {self.port}")

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[PicoController] Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def _require_connected(self):
        if not self.connected:
            raise RuntimeError("Not connected.  Call connect() first.")

    # ── low-level serial I/O ──────────────────────────────────────────────────

    def _flush_input(self) -> None:
        self._ser.reset_input_buffer()

    def _send_command(self, cmd: str, response_timeout: Optional[float] = None) -> str:
        """
        Send a command string and collect all output lines until the next
        prompt (">>" ) appears or the timeout expires.

        Returns the collected output as a single string.
        """
        self._require_connected()
        timeout = response_timeout or self.response_timeout

        # Send command
        self._ser.write((cmd.strip() + "\r\n").encode())

        # Collect response
        lines = []
        t0 = time.time()
        buf = ""
        while time.time() - t0 < timeout:
            chunk = self._ser.read(256).decode(errors="replace")
            if chunk:
                buf += chunk
                # Split on CRLF / LF and keep incomplete tail
                parts = buf.replace("\r\n", "\n").replace("\r", "\n").split("\n")
                buf = parts[-1]
                for line in parts[:-1]:
                    stripped = line.strip()
                    if stripped:
                        lines.append(stripped)
                # Check if the prompt appeared
                if self.PROMPT in buf or any(self.PROMPT in l for l in lines[-3:]):
                    break
            else:
                time.sleep(0.05)

        response = "\n".join(lines)
        self._parse_position_update(response)
        return response

    def _parse_position_update(self, response: str) -> None:
        """Extract position updates from command output."""
        # "M x axis=12.34" or "M y axis=5.67"
        for m in re.finditer(r"M\s+([xy])\s+axis\s*=\s*([\d.]+)", response, re.I):
            axis, val = m.group(1).lower(), float(m.group(2))
            if axis == "x":
                self.current_M_x = val
            else:
                self.current_M_y = val
        # "New Z = 99.123"
        m = re.search(r"New Z\s*=\s*([\d.]+)", response, re.I)
        if m:
            self.current_Z = float(m.group(1))
        # Z from "z" command: "Current Z height: 99.123456 mm"
        m = re.search(r"Current Z height:\s*([\d.]+)", response, re.I)
        if m:
            self.current_Z = float(m.group(1))

    # ── high-level motor commands ─────────────────────────────────────────────

    def home(self, axis: str) -> str:
        """
        Home the given axis ('x' or 'y').  Blocks until complete.
        Homing can take tens of seconds — response_timeout is generous.
        """
        axis = axis.lower()
        print(f"[Pico] Homing {axis} axis …")
        resp = self._send_command(f"home {axis}", response_timeout=self.response_timeout)
        print(f"[Pico] {resp}")
        if axis == "x":
            self.current_M_x = 0.0
        else:
            self.current_M_y = 0.0
        return resp

    def move_steps(self, axis: str, steps: int,
                   forward: bool = True, fast: bool = False) -> str:
        """Move by a number of microsteps."""
        axis = axis.lower()
        parts = ["move", axis, str(steps)]
        if not forward:
            parts.append("rev")
        if fast:
            parts.append("fast")
        resp = self._send_command(" ".join(parts))
        print(f"[Pico] {resp}")
        return resp

    def move_mm(self, axis: str, mm: float,
                forward: bool = True, fast: bool = True) -> str:
        """
        Move a given distance in millimetres.

        For the Y axis this moves in *Z* space (scissor geometry) — mm is ΔZ.
        For the X axis mm is linear travel.
        """
        axis = axis.lower()
        parts = ["movemm", axis, f"{mm:.4f}"]
        if not forward:
            parts.append("rev")
        if fast:
            parts.append("fast")
        resp = self._send_command(" ".join(parts))
        print(f"[Pico] {resp}")
        return resp

    def move_to_mm(self, axis: str, target_mm: float, fast: bool = True) -> str:
        """
        Move to an absolute position in mm (relative to home).

        For the X axis: absolute M (screw) position.
        For the Y axis: absolute Z (platform height) position.
        """
        axis = axis.lower()
        current = self.current_M_x if axis == "x" else self.current_Z
        delta = target_mm - current
        if abs(delta) < 1e-3:
            print(f"[Pico] Already at {target_mm:.3f} mm on {axis}")
            return ""
        forward = delta > 0
        return self.move_mm(axis, abs(delta), forward=forward, fast=fast)

    def move_until_limit(self, axis: str, forward: bool = True,
                         fast: bool = True) -> str:
        """Drive to a limit switch."""
        axis = axis.lower()
        direction = "fwd" if forward else "rev"
        speed = "fast" if fast else "slow"
        resp = self._send_command(f"mul {axis} {direction} {speed}",
                                  response_timeout=self.response_timeout)
        print(f"[Pico] {resp}")
        return resp

    def get_z(self) -> float:
        """Query current Z height from the Pico and return it."""
        resp = self._send_command("z", response_timeout=5.0)
        print(f"[Pico] {resp}")
        return self.current_Z

    def wait_idle(self, poll_interval: float = 0.5) -> None:
        """
        Placeholder — the serial protocol is synchronous so commands already
        block until complete.  This exists for API symmetry if you later add
        async motion.
        """
        pass

    # ── convenience ──────────────────────────────────────────────────────────

    @property
    def position(self) -> dict:
        return {
            "M_x_mm": self.current_M_x,
            "M_y_mm": self.current_M_y,
            "Z_mm":   self.current_Z,
        }

    def __repr__(self):
        status = "connected" if self.connected else "disconnected"
        return (
            f"PicoController(port={self.port!r}, status={status}, "
            f"M_x={self.current_M_x:.3f}, M_y={self.current_M_y:.3f}, "
            f"Z={self.current_Z:.3f})"
        )
