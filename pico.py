from pathlib import Path
import serial
import time

class Pico:
    def __init__(self, port, baudrate=115200, timeout=5):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(0.5)
        self._flush() # clear any boot messages
        self.upload("Main.py") # push the latest version of Main.py
        self.exec("import sys; sys.modules.pop('Main', None); import Main as m") # force fresh load

    def _flush(self):
        self.ser.read_all()
    
    def exec(self, cmd, timeout=1800, debug=False): # 30 minutes should be enough time for everything
        """
        Send a command to the Pico REPL and return its output
        timeout = hard limit in seconds
        """
        self._flush()
        self.ser.write((cmd + "\r\n").encode())
        output = ""
        self.ser.timeout = 1 # read in 1-second chunks
        start_time = time.time()
        deadline = start_time + timeout
        while (start_time < deadline):
            chunk = self.ser.read_all().decode(errors="replace")
            if chunk:
                # The (keepend=True) preserves the newline characters when splitting so lines print with proper spacing, and end="" prevents print from adding an extra newline on top of that.
                for line in chunk.splitlines(keepends=True):
                    if line.strip() and not line.strip().startswith(">>>") and line.strip() != cmd.strip():
                        print(line, end="", flush=True)
                output += chunk
            if ">>>" in output: # REPL is ready again = command finished
                break
            time.sleep(0.1)
        else:
            print("\nTimed out!\n")
        if debug:
            print(f"\nCommand took {time.time() - start_time:.2f} seconds.")
        return output
    
    def upload(self, local_path, remote_path=None, chunk_size=1024):
        """
        Upload a file to the Pico using raw REPL mode (Ctrl+A), which is
        much faster than exec() — no prompt-polling overhead, and larger chunks.

        local_path  : path to the file on the host
        remote_path : destination filename on the Pico (default: basename of local_path)
        chunk_size  : bytes per write call; 1024 is a safe fast default
        """
        if remote_path is None:
            remote_path = Path(local_path).name

        with open(local_path, "rb") as f:
            data = f.read()
        total = len(data)

        # --- enter raw REPL (Ctrl+A) ---
        self.ser.write(b"\x01")
        self.ser.flush()
        deadline = time.time() + 5
        buf = b""
        while time.time() < deadline:
            buf += self.ser.read_all()
            if buf.endswith(b">"):
                break
            time.sleep(0.05)

        def raw_exec(cmd):
            """Send one command in raw REPL mode and wait for \x04\x04 completion."""
            self.ser.write(cmd.encode() + b"\x04")
            self.ser.flush()
            result = b""
            n_delimiters = 0
            while n_delimiters < 2:
                c = self.ser.read(1)
                if c == b"\x04":
                    n_delimiters += 1
                result += c
            return result

        # Remove stale .mpy so MicroPython doesn't load it instead of the fresh .py
        stem = remote_path.rsplit(".", 1)[0]
        raw_exec(f"try:\n import os\n os.remove('{stem}.mpy')\nexcept OSError:\n pass")

        raw_exec(f"_f = open('{remote_path}', 'wb')")
        for offset in range(0, total, chunk_size):
            chunk = data[offset : offset + chunk_size]
            raw_exec(f"_f.write({chunk!r})")
            pct = min(offset + chunk_size, total) / total * 100
            print(f"\rUploading {remote_path}: {pct:.0f}%", end="", flush=True)
        raw_exec("_f.close(); del _f")

        # --- exit raw REPL (Ctrl+B) ---
        self.ser.write(b"\x02")
        time.sleep(0.2)
        self.ser.read_all()

        print(f"\r✓ Uploaded {local_path} → {remote_path} ({total} bytes)")

    def close(self):
        self.ser.close()