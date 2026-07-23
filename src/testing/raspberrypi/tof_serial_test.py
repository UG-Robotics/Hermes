"""
Pi-side ToF bring-up test.

The two VL53L1X ToF sensors are wired to the ESP32, not the Pi (see
hardware/tof.py and firmware/esp_controller/tof.cpp). The Pi only ever sees
them as the tof1_mm / tof2_mm fields inside the ESP32's TEL telemetry line:

    TEL,ax,ay,az,gx,gy,gz,tof1_mm,tof2_mm

(field order = communication/packet_parser.py's TEL_FIELD_NAMES; tof1 = LEFT,
tof2 = RIGHT.) This script opens the serial link, decodes those TEL lines, and
prints the two distances live so you can wave a hand in front of each sensor
and confirm the full ESP32 -> serial -> Pi path works.

This is deliberately self-contained (only needs pyserial) -- it imports nothing
from the project, so it stays runnable even mid-refactor, matching the other
scripts in testing/raspberrypi/.

This tests the PRODUCTION wiring, where the ToF sensors hang off the ESP32.
For a bench test with a sensor wired straight to the Pi's I2C bus, use
tof_i2c_test.py instead.

Run on the Pi:
    python3 tof_serial_test.py                # auto-detect the ESP32 port
    python3 tof_serial_test.py /dev/ttyUSB0   # force a specific port
    python3 tof_serial_test.py --raw          # dump every raw line (use when nothing decodes)
"""

import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
TIMEOUT = 0.1

# Same default + fallbacks as config/robot_config.py, so this test finds the
# ESP32 on whatever the current wiring exposes (USB-serial vs. the UART header).
PORT_CANDIDATES = ["/dev/ttyUSB0", "/dev/serial0", "/dev/ttyACM0", "/dev/ttyAMA0", "/dev/ttyS0"]

# Matches hardware/tof.py's ToFArray.OUT_OF_RANGE_MM and tof.cpp's sentinel.
# A reading at/above this means "nothing in range" or "sensor never came up",
# NOT a wall against the bumper.
OUT_OF_RANGE_MM = 2000.0

# TEL field order must match communication/packet_parser.py.
TEL_FIELD_NAMES = ["ax", "ay", "az", "gx", "gy", "gz", "tof1_mm", "tof2_mm"]


def find_port():
    """Return the first candidate port that exists, else the first port pyserial
    can see, else None. Does not open it -- just picks a name to try."""
    present = {p.device for p in list_ports.comports()}
    for name in PORT_CANDIDATES:
        if name in present:
            return name
    # Nothing from our known list is present -- fall back to whatever is there.
    return next(iter(sorted(present)), None)


def open_serial(port):
    print(f"Opening {port} @ {BAUD} ...")
    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    # The ESP32 typically resets when the port opens; give it a moment so we
    # don't count its boot-time silence as "no telemetry".
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser


def fmt(mm):
    """Format one distance, flagging out-of-range / never-initialised sensors."""
    if mm >= OUT_OF_RANGE_MM:
        return f"{mm:7.1f} mm  [OUT OF RANGE]"
    return f"{mm:7.1f} mm"


def parse_tel(line):
    """Return (tof1_mm, tof2_mm) from a TEL line, or None if it isn't one."""
    parts = line.split(",")
    if not parts or parts[0].strip().upper() != "TEL":
        return None
    values = parts[1:]
    if len(values) != len(TEL_FIELD_NAMES):
        return None
    try:
        floats = [float(v) for v in values]
    except ValueError:
        return None
    fields = dict(zip(TEL_FIELD_NAMES, floats))
    return fields["tof1_mm"], fields["tof2_mm"]


def main():
    args = [a for a in sys.argv[1:]]
    raw_mode = "--raw" in args
    args = [a for a in args if a != "--raw"]
    port = args[0] if args else find_port()

    if not port:
        print("No serial port found. Is the ESP32 plugged in / powered?")
        print(f"Tried: {', '.join(PORT_CANDIDATES)} plus anything pyserial lists.")
        return 1

    try:
        ser = open_serial(port)
    except Exception as e:
        print(f"Could not open {port}: {e}")
        print("Check the cable, that no other program holds the port, and that")
        print("you're in the 'dialout' group (or run with sudo).")
        return 1

    print("Listening for ESP32 TEL telemetry. Ctrl-C to stop.\n")
    if raw_mode:
        print("(--raw) dumping every line verbatim:\n")

    tel_count = 0
    last_tel_time = None
    last_display = 0.0
    started = time.time()
    warned_no_tel = False

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()

            if not line:
                # No TEL in the first few seconds usually means the firmware
                # isn't sending telemetry yet (telemetry.cpp only emits
                # STATUS,OK until the real sendTelemetry() path is wired in).
                if (
                    not warned_no_tel
                    and tel_count == 0
                    and time.time() - started > 4.0
                ):
                    print(
                        "\n[!] No TEL telemetry yet after 4s. Check that the ESP32 "
                        "firmware is actually streaming TEL lines (not just STATUS), "
                        "and that the baud/port are right.\n"
                    )
                    warned_no_tel = True
                continue

            if raw_mode:
                print(line)
                continue

            # Surface STATUS lines -- the ToF init errors from tof.cpp
            # ("STATUS,ERR ToF LEFT: init/address failed", etc.) come through
            # here and tell you a sensor never came up on the ESP32 side.
            if line.upper().startswith("STATUS"):
                print(f"  [STATUS] {line[len('STATUS'):].lstrip(', ')}")
                continue

            tof = parse_tel(line)
            if tof is None:
                continue

            left_mm, right_mm = tof
            tel_count += 1
            now = time.time()
            rate = 0.0
            if last_tel_time is not None:
                dt = now - last_tel_time
                if dt > 0:
                    rate = 1.0 / dt
            last_tel_time = now

            # Throttle the live display to ~10 Hz so a 20 Hz+ stream stays
            # readable, and update in place with \r.
            if now - last_display >= 0.1:
                last_display = now
                print(
                    f"\rLEFT {fmt(left_mm)}   RIGHT {fmt(right_mm)}   "
                    f"[{tel_count} pkts, {rate:4.1f} Hz]   ",
                    end="",
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print(f"Received {tel_count} TEL packet(s) total.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
