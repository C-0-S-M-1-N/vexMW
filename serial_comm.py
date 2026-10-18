import serial, serial.tools.list_ports, time, sys
from datetime import datetime

OUTFILE = "robot_log.txt"
BAUD = 115200

def find_ports():
    return [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x2888]

def main():
    ports = find_ports()
    if not ports:
        sys.exit("No VEX device found. Is the Brain/controller plugged in?")

    # The Brain exposes two ports; the user port is usually the second.
    # Pass the port as an argument to override: python log.py COM5
    port = sys.argv[1] if len(sys.argv) > 1 else ports[-1]
    print(f"Logging {port} -> {OUTFILE} (Ctrl+C to stop)")

    ser = serial.Serial(port, BAUD, timeout=0.05)
    buf = b""

    with open(OUTFILE, "a", encoding="utf-8") as f:
        f.write(f"\n--- session {datetime.now().isoformat()} ---\n")
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode(errors="ignore").rstrip("\r")
                    if text:
                        f.write(text + "\n")
                        f.flush()          # don't lose lines if the program dies
                        print(text)        # also show it live; remove if unwanted
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            ser.close()

main()
