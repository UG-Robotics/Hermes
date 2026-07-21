"""
Threaded drive-command dispatcher.

Separates *deciding* a drive command from *putting it on the wire*, so neither
can stall the other:

  * The 20 Hz control loop (runtime.tick) decides autonomous commands and hands
    each one to submit(). tick() can be slowed by a heavy vision frame; that no
    longer delays the command reaching the wheels, because...

  * A dedicated background thread is the sole writer to the ESP32. Every cycle
    (rate_hz) it sends the *current* command and nothing else. There is no
    queue: submit() overwrites a single slot, so the newest command always wins
    and no backlog of stale steer/throttle can build up -- pressing a then d
    lands as "d" on the very next send.

  * MANUAL DRIVING is streamed live. When a manual_source is supplied and
    reports manual mode active, the thread reads the operator's *current* input
    each cycle and sends that directly -- so keyboard -> wheels runs at the
    dispatch rate, fully independent of the (vision-heavy) control loop. That is
    what makes manual steering flow smoothly, with no lag and no centred
    dead-zone between commands. When manual is not active, the thread sends
    whatever the control loop last submit()ted (the autonomous / heading-hold
    command).

Safety notes:

  * SOLE WRITER. Only this thread writes drive commands, so two threads never
    interleave bytes on the wire. The control loop keeps ownership of serial
    *reads* (telemetry); a UART is full-duplex, so one reader + one writer on
    separate threads is fine.
  * send_emergency() (shutdown only) writes directly and must be the LAST thing
    on the wire -- a fresh normal CMD clears the ESP32's emergency latch. So the
    runtime stop()s this dispatcher BEFORE sending the emergency stop.
  * IDEMPOTENT. The ESP32 re-applies its latest command every firmware loop
    anyway, so re-sending the same command at the dispatch rate changes nothing
    for the hardware; it just keeps the command fresh.
"""

import threading
import time
from typing import Callable, Optional, Tuple

from communication.protocol import serialize_command
from utils.logger import get_logger

logger = get_logger(__name__)

# A manual_source returns the operator's current (speed, steer, action) while
# manual driving is active, or None when it isn't (hand back to autonomous).
ManualSource = Callable[[], Optional[Tuple[int, int, str]]]


class CommandDispatcher:
    """Streams the latest drive command to the ESP32 on its own thread."""

    def __init__(self, link, manual_source: Optional[ManualSource] = None,
                 rate_hz: float = 50.0):
        self._link = link
        self._manual_source = manual_source
        self._interval = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[int, int, str, int]] = None  # autonomous slot
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def submit(self, speed: int, steer: int, action: str, mode: int) -> None:
        """Hand the dispatcher the control loop's latest command (newest wins).

        Non-blocking: it just overwrites the single slot. Used for the
        autonomous / heading-hold command; manual driving is streamed live from
        manual_source instead (see the module docstring).
        """
        with self._lock:
            self._latest = (int(speed), int(steer), str(action), int(mode))

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="CommandDispatcher")
        self._thread.start()
        logger.info(f"Command dispatcher streaming at {1.0 / self._interval:.0f} Hz.")

    def stop(self, timeout: float = 0.5) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        self._thread = None

    # --------------------------------------------------------------- internals
    def _current_command(self) -> Optional[Tuple[int, int, str, int]]:
        """The command to send this cycle: live manual input when manual mode is
        active, otherwise the control loop's last autonomous submission."""
        if self._manual_source is not None:
            manual = self._manual_source()
            if manual is not None:
                speed, steer, action = manual
                return (int(speed), int(steer), str(action), 1)
        with self._lock:
            return self._latest

    def _run(self) -> None:
        while not self._stop.is_set():
            start = time.time()
            try:
                cmd = self._current_command()
                if cmd is not None:
                    self._link.send(serialize_command(*cmd))
            except Exception as e:
                logger.error(f"Command dispatch failed: {e}")
            # Event.wait is both the paced sleep and an instant wake on stop().
            self._stop.wait(max(0.0, self._interval - (time.time() - start)))
