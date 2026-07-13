# from utils.logger import get_logger

# logger = get_logger(__name__)


# class SerialLink:
#     """
#     Wraps the serial connection to the Arduino/MCU.

#     Handles connect/write/close and degrades to EMULATION mode (no physical
#     port) instead of crashing, so the rest of the system can keep running
#     off-hardware for testing.
#     """

#     def __init__(self, port: str, baud_rate: int, timeout: float):
#         self.port = port
#         self.baud_rate = baud_rate
#         self.timeout = timeout
#         self.ser = None
#         self.emulation = False

#     def connect(self) -> bool:
#         """Attempts to open the serial port. Returns True if a real link is open."""
#         try:
#             import serial
#             self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
#             self.emulation = False
#             logger.info(f"Connected to Arduino on {self.port}")
#             return True
#         except (ImportError, Exception) as e:
#             logger.warning(f"Running in EMULATION mode. Serial port unavailable: {e}")
#             self.ser = None
#             self.emulation = True
#             return False

#     @property
#     def is_open(self) -> bool:
#         return self.ser is not None and self.ser.is_open

#     def send(self, packet: str) -> bool:
#         """Writes a packet string to the serial port, or logs it if in emulation mode."""
#         if self.is_open:
#             try:
#                 self.ser.write(packet.encode('utf-8'))
#                 self.ser.flush()
#                 return True
#             except Exception as serial_err:
#                 logger.error(f"Serial write error: {serial_err}")
#                 return False
#         else:
#             logger.debug(f"[EMU TX] {packet.strip()}")
#             return False

#     def send_emergency(self, emergency_packet: str) -> None:
#         """Best-effort emergency stop write, used during shutdown."""
#         try:
#             if self.is_open:
#                 self.ser.write(emergency_packet.encode('utf-8'))
#                 self.ser.flush()
#                 self.ser.close()
#             else:
#                 print(f"[EMU EMERGENCY TX] {emergency_packet.strip()}")
#         except Exception as fail_safe_err:
#             logger.critical(f"Fail-safe error: {fail_safe_err}")

#     def read_line(self):
#         """Non-blocking-ish read of a single line from the MCU, or None if unavailable."""
#         if not self.is_open:
#             return None
#         try:
#             if self.ser.in_waiting > 0:
#                 raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
#                 return raw if raw else None
#         except Exception as read_err:
#             logger.error(f"Serial read error: {read_err}")
#         return None

#     def close(self) -> None:
#         if self.is_open:
#             try:
#                 self.ser.close()
#             except Exception as e:
#                 logger.warning(f"Error closing serial port: {e}")

from utils.logger import get_logger

logger = get_logger(__name__)


class SerialLink:
    """
    Wraps the serial connection to the ESP32.

    Handles connect/write/close and degrades to EMULATION mode (no physical
    port) instead of crashing, so the rest of the system can keep running
    off-hardware for testing.
    """

    def __init__(self, port: str, baud_rate: int, timeout: float):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.emulation = False

    def connect(self) -> bool:
        """Attempts to open the serial port. Returns True if a real link is open.

        pyserial missing and "port exists but can't be opened" are handled
        as two separate cases instead of one blanket except, so the log tells
        you which problem you actually have.
        """
        try:
            import serial
        except ImportError as e:
            logger.error(f"pyserial is not installed, cannot open a real serial link: {e}")
            self.ser = None
            self.emulation = True
            return False

        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            self.emulation = False
            logger.info(f"Connected to ESP32 on {self.port}")
            return True
        except Exception as e:
            logger.warning(f"Running in EMULATION mode. Serial port unavailable: {e}")
            self.ser = None
            self.emulation = True
            return False

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send(self, packet: str) -> bool:
        """Writes a packet string to the serial port, or logs it if in emulation mode."""
        if self.is_open:
            try:
                self.ser.write(packet.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception as serial_err:
                logger.error(f"Serial write error: {serial_err}")
                return False
        else:
            logger.debug(f"[EMU TX] {packet.strip()}")
            return False

    def send_emergency(self, emergency_packet: str) -> None:
        """Best-effort emergency stop write, used during shutdown."""
        try:
            if self.is_open:
                self.ser.write(emergency_packet.encode('utf-8'))
                self.ser.flush()
                self.ser.close()
            else:
                print(f"[EMU EMERGENCY TX] {emergency_packet.strip()}")
        except Exception as fail_safe_err:
            logger.critical(f"Fail-safe error: {fail_safe_err}")

    def read_line(self):
        """Non-blocking-ish read of a single line from the MCU, or None if unavailable.

        Uses errors="replace" (not "ignore") so a corrupted/malformed line shows
        up as visible U+FFFD replacement characters in the log instead of
        silently disappearing. Noisy-wire issues become visible instead of
        invisible.
        """
        if not self.is_open:
            return None
        try:
            if self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('utf-8', errors='replace').strip()
                return raw if raw else None
        except Exception as read_err:
            logger.error(f"Serial read error: {read_err}")
        return None

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