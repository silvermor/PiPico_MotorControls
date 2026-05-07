import serial
import time

class Pico:
    def __init__(self, port, baudrate=115200, timeout=5):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(0.5)
        self._flush() # clear any boot messages
        self.exec("import Main as m") # load main once on connect

    def _flush(self):
        self.ser.read_all()
    
    def exec(self, cmd, timeout=300, debug=False): # 5 minutes should be enough time for everything
        """
        Send a command to the Pico REPL and return its output
        timeout = hard limit in seconds
        idle-timeout = how long to wait with no new output before giving up
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
    
    def close(self):
        self.ser.close()