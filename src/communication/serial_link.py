from utils.logger import get_logger
from utils.telemetry_hub import get_hub

logger = get_logger(__name__)
hub = get_hub()


class SerialLink:
    """
    Wraps the serial connection to the ESP32.

    Three ways to run, chosen at connect() time:

      * REAL       — a pyserial port to a physical ESP32.
      * SIMULATED  — a virtual ESP32 (hardware/sim_esp32.py) that speaks the
                     same protocol. Pass ``backend=SimulatedESP32()``. This is
                     how the software runs end-to-end with no hardware: the Pi
                     genuinely exchanges CMD/TEL packets, they're just crossing
                     an in-process object instead of a wire.
      * EMULATION  — no port and no backend; TX is logged, RX returns nothing.
                     The graceful fallback when a real port fails to open.

    Every packet that crosses the link (either direction) is published to the
    TelemetryHub so the dashboard can show live Pi<->ESP32 traffic.
    """

    def __init__(self, port: str, baud_rate: int, timeout: float, backend=None):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.emulation = False
        # A SimulatedESP32-like object (write_line/read_line). When set, the
        # link is in SIMULATED mode and never touches pyserial.
        self.backend = backend

    @property
    def simulated(self) -> bool:
        return self.backend is not None

    def connect(self) -> bool:
        """Open the link. Returns True if a real OR simulated link is usable.

        In simulated mode there is nothing to open — the backend is always
        ready — so we report success immediately.
        """
        if self.simulated:
            logger.info("SerialLink using SIMULATED backend (virtual ESP32).")
            self.emulation = False
            return True

        try:
            import serial
        except ImportError as e:
            logger.error(f"pyserial is not installed, cannot open a real serial link: {e}")
            self.ser = None
            self.emulation = True
            return False

        for port_name in self._candidate_ports():
            try:
                self.ser = serial.Serial(port_name, self.baud_rate, timeout=self.timeout)
                self.port = port_name
                self.emulation = False
                logger.info(f"Connected to ESP32 on {self.port}")
                return True
            except Exception as e:
                last_error = e

        logger.warning(f"Running in EMULATION mode. Serial port unavailable: {last_error}")
        self.ser = None
        self.emulation = True
        return False

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send(self, packet: str) -> bool:
        """Send a packet to the ESP32 (real, simulated or emulated)."""
        hub.comms("tx", packet, mode=self._mode())

        if self.simulated:
            self.backend.write_line(packet)
            return True

        if self.is_open:
            try:
                self.ser.write(packet.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception as serial_err:
                logger.error(f"Serial write error: {serial_err}")
                return False

        logger.debug(f"[EMU TX] {packet.strip()}")
        return False

    def send_emergency(self, emergency_packet: str) -> None:
        """Best-effort emergency stop write, used during shutdown."""
        hub.comms("tx", emergency_packet, mode=self._mode(), emergency=True)
        try:
            if self.simulated:
                self.backend.write_line(emergency_packet)
                return
            if self.is_open:
                self.ser.write(emergency_packet.encode('utf-8'))
                self.ser.flush()
                self.ser.close()
            else:
                logger.warning(f"[EMU EMERGENCY TX] {emergency_packet.strip()}")
        except Exception as fail_safe_err:
            logger.critical(f"Fail-safe error: {fail_safe_err}")

    def read_line(self):
        """Read one line from the ESP32, or None if nothing is waiting.

        Uses errors="replace" (not "ignore") so a corrupted/malformed line shows
        up as visible U+FFFD replacement characters in the log instead of
        silently disappearing. Noisy-wire issues become visible instead of
        invisible.
        """
        raw = None
        if self.simulated:
            raw = self.backend.read_line()
        elif self.is_open:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.readline().decode('utf-8', errors='replace').strip()
            except Exception as read_err:
                logger.error(f"Serial read error: {read_err}")
                return None

        if raw:
            hub.comms("rx", raw, mode=self._mode())
            return raw
        return None

    def _mode(self) -> str:
        if self.simulated:
            return "sim"
        if self.is_open:
            return "real"
        return "emu"

    def _candidate_ports(self):
        candidates = []

        def add(port_name):
            if port_name and port_name not in candidates:
                candidates.append(port_name)

        add(self.port)

        try:
            from config.robot_config import SERIAL_PORT_FALLBACKS
            for fallback in SERIAL_PORT_FALLBACKS:
                add(fallback)
        except Exception:
            pass

        try:
            from serial.tools import list_ports
            for port_info in list_ports.comports():
                device = getattr(port_info, "device", None)
                if device:
                    add(device)
        except Exception:
            pass

        return candidates

    def close(self) -> None:
        if self.is_open:
            try:
                self.ser.close()
            except Exception as e:
                logger.warning(f"Error closing serial port: {e}")

    # NOTE: reset_output_buffer() on send() and automatic reconnect-on-disconnect
    # are intentionally NOT added. Add reset_output_buffer() only if we actually
    # observe stale/duplicate packets on the wire; add reconnect only once basic
    # two-way comms are proven solid on the bench. Adding either preemptively
    # just adds surface area for bugs we don't have yet.