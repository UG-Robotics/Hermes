"""
Direct-to-Pi VL53L1X ToF bench test (I2C).

BENCH WIRING ONLY. In the real robot the ToF sensors are on the ESP32 and the
Pi reads them over serial (see hardware/tof.py, firmware/esp_controller/tof.cpp,
and tof_serial_test.py in this folder). For THIS test the VL53L1X is wired
straight to the Pi's I2C bus, so the Pi does the ranging itself:

    VL53L1X            Raspberry Pi (40-pin header)
    -------            ---------------------------
    VIN / VDD  ------> 3V3   (pin 1)   -- NOT 5V
    GND        ------> GND   (pin 6)
    SDA        ------> SDA1  (pin 3, GPIO2)
    SCL        ------> SCL1  (pin 5, GPIO3)
    (XSHUT/GPIO1 left unconnected for a single sensor at 0x29)

Prereqs on the Pi:
    * I2C enabled:  sudo raspi-config -> Interface Options -> I2C -> Yes
      (then /dev/i2c-1 exists; check with `ls /dev/i2c-*` or `i2cdetect -y 1`).
    * A VL53L1X Python driver. This script auto-detects either:
        - Pimoroni:  pip install VL53L1X        (import VL53L1X)
        - Adafruit:  pip install adafruit-circuitpython-vl53l1x  (needs Blinka)
      and, if present, smbus2 for a raw bus presence-probe.

Run on the Pi:
    python3 tof_i2c_test.py                 # bus 1, address 0x29 (defaults)
    python3 tof_i2c_test.py --bus 1 --addr 0x29
"""

import sys
import time

# Default single-sensor setup: VL53L1X powers on at 0x29, Pi I2C bus 1.
DEFAULT_BUS = 1
DEFAULT_ADDR = 0x29

# Mirrors hardware/tof.py's ToFArray.OUT_OF_RANGE_MM: readings at/above this
# mean "nothing in range", not a wall against the bumper.
OUT_OF_RANGE_MM = 2000.0

# A healthy VL53L1X returns this from its model-id register (0x010F).
MODEL_ID_REG = 0x010F
MODEL_ID_OK = 0xEACC


# --------------------------------------------------------------------------- #
# Raw bus presence-probe (optional diagnostic, needs smbus2).
# Answers the "is the sensor even physically on the bus?" question BEFORE we
# blame the driver -- same philosophy as utils/I2C_Scanner and the .ino tests.
# --------------------------------------------------------------------------- #
def probe_bus(bus_num, addr):
    try:
        from smbus2 import SMBus, i2c_msg
    except ImportError:
        print("  (smbus2 not installed -- skipping raw presence probe; "
              "`pip install smbus2` for it)")
        return

    try:
        with SMBus(bus_num) as bus:
            # VL53L1X uses 16-bit register addresses: write the pointer, then
            # read 2 bytes back in one combined transaction.
            write = i2c_msg.write(addr, [MODEL_ID_REG >> 8, MODEL_ID_REG & 0xFF])
            read = i2c_msg.read(addr, 2)
            bus.i2c_rdwr(write, read)
            data = list(read)
            model = (data[0] << 8) | data[1]
            if model == MODEL_ID_OK:
                print(f"  0x{addr:02X} ACK, model id 0x{model:04X} -> healthy VL53L1X")
            else:
                print(f"  0x{addr:02X} ACK, but model id 0x{model:04X} "
                      f"(expected 0x{MODEL_ID_OK:04X}) -- clone or wrong device?")
    except FileNotFoundError:
        print(f"  /dev/i2c-{bus_num} not found -- is I2C enabled (raspi-config)?")
    except Exception as e:
        print(f"  0x{addr:02X} did not answer ({e}). This is WIRING/POWER, not the "
              f"driver: check VIN=3V3/GND/SDA/SCL and that SDA+SCL have pull-ups.")


# --------------------------------------------------------------------------- #
# Driver wrappers. Each exposes: check_import(), read_mm() -> float|None, close().
# read_mm returns mm as a float, or None when there's no valid reading this cycle.
# --------------------------------------------------------------------------- #
class PimoroniDriver:
    name = "Pimoroni VL53L1X"

    @staticmethod
    def check_import():
        import VL53L1X  # noqa: F401

    def __init__(self, bus, addr):
        import VL53L1X
        self.tof = VL53L1X.VL53L1X(i2c_bus=bus, i2c_address=addr)
        self.tof.open()
        # 1=short (~1.3m), 2=medium (~3m), 3=long (~4m). Long matches how the
        # robot uses these for wall/obstacle ranging.
        self.tof.start_ranging(3)

    def read_mm(self):
        d = self.tof.get_distance()
        if d is None or d <= 0:
            return None
        return float(d)

    def close(self):
        for fn in (self.tof.stop_ranging, self.tof.close):
            try:
                fn()
            except Exception:
                pass


class AdafruitDriver:
    name = "Adafruit CircuitPython VL53L1X"

    @staticmethod
    def check_import():
        import adafruit_vl53l1x  # noqa: F401

    def __init__(self, bus, addr):
        import board
        import busio
        import adafruit_vl53l1x
        # Blinka's board.SCL/SDA map to the Pi's primary I2C (bus 1).
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl = adafruit_vl53l1x.VL53L1X(i2c, address=addr)
        self.vl.distance_mode = 2  # 1=short, 2=long
        self.vl.timing_budget = 100
        self.vl.start_ranging()

    def read_mm(self):
        if not self.vl.data_ready:
            return None
        cm = self.vl.distance  # cm (float) or None when out of range
        self.vl.clear_interrupt()
        if cm is None:
            return None
        return cm * 10.0

    def close(self):
        try:
            self.vl.stop_ranging()
        except Exception:
            pass


def make_driver(bus, addr):
    """Pick the first installed driver. Raises RuntimeError if none is."""
    missing = []
    for cls in (PimoroniDriver, AdafruitDriver):
        try:
            cls.check_import()
        except ImportError:
            missing.append(cls.name)
            continue
        # Library is present -> use it. Let init errors propagate: they mean the
        # sensor/bus is the problem, not the library choice.
        print(f"Using driver: {cls.name}")
        return cls(bus, addr)

    raise RuntimeError(
        "No VL53L1X Python library found (tried: " + ", ".join(missing) + ").\n"
        "Install one on the Pi, e.g.:\n"
        "    pip install VL53L1X          # Pimoroni (simplest)\n"
        "  or\n"
        "    pip install adafruit-circuitpython-vl53l1x adafruit-blinka"
    )


def fmt(mm):
    if mm >= OUT_OF_RANGE_MM:
        return f"{mm:7.1f} mm  [OUT OF RANGE]"
    return f"{mm:7.1f} mm"


def parse_args(argv):
    bus, addr = DEFAULT_BUS, DEFAULT_ADDR
    it = iter(argv)
    for tok in it:
        if tok == "--bus":
            bus = int(next(it))
        elif tok == "--addr":
            addr = int(next(it), 0)  # accepts 0x29 or 41
        else:
            print(f"Unknown argument: {tok}")
            print(__doc__)
            sys.exit(2)
    return bus, addr


def main():
    bus, addr = parse_args(sys.argv[1:])

    print(f"VL53L1X direct-I2C test  (bus {bus}, address 0x{addr:02X})\n")
    print("Raw bus probe:")
    probe_bus(bus, addr)
    print()

    try:
        driver = make_driver(bus, addr)
    except RuntimeError as e:
        print(e)
        return 1
    except Exception as e:
        print(f"Driver init failed: {e}")
        print("If the raw probe above said the sensor didn't answer, fix the "
              "wiring/power first.")
        return 1

    print("\nRanging. Wave a hand in front of the sensor. Ctrl-C to stop.\n")

    count = 0
    last_display = 0.0
    try:
        while True:
            mm = driver.read_mm()
            if mm is None:
                time.sleep(0.01)  # no fresh sample yet -- don't busy-spin
                continue

            count += 1
            now = time.time()
            if now - last_display >= 0.1:  # throttle display to ~10 Hz
                last_display = now
                print(f"\rDistance {fmt(mm)}   [{count} reads]   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        driver.close()
        print(f"Took {count} reading(s) total.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
