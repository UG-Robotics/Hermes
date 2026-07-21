"""
Manual driving via the laptop keyboard.

Keys (hold to act, release to stop):
    w  - forward
    s  - backward
    a  - steer left
    d  - steer right
    space - immediate stop / straighten wheels
    m  - toggle MANUAL <-> AUTONOMOUS

The listener runs in a background thread using pynput when available, or a
raw terminal keyboard reader when that is the more reliable option on a Pi.
If both are unavailable (e.g. no TTY and no X/Wayland input backend), it
degrades to a no-op with manual mode permanently off, instead of crashing —
the dashboard can still drive manually over HTTP.
"""

import os
import sys
import time
import threading

from utils.logger import get_logger
from utils.telemetry_hub import get_hub
from config.robot_config import (
    KEY_TOGGLE_MANUAL,
    KEY_FORWARD,
    KEY_BACKWARD,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_STOP,
    SPEED_DEFAULT_FORWARD,
    SPEED_DEFAULT_BACKWARD,
    SPEED_STOP,
    STEER_MANUAL_DEGREE,
)

logger = get_logger(__name__)
hub = get_hub()

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except Exception as e:  # ImportError, or X display / permission failures
    keyboard = None
    PYNPUT_AVAILABLE = False
    logger.warning(f"pynput unavailable at import time: {e}. Keyboard manual override disabled.")


class KeyboardOverrideListener:
    """Background keyboard listener producing manual (speed, steer, action) targets."""

    def __init__(self):
        self._lock = threading.Lock()
        self._manual_mode_active = False
        self._pressed_keys = set()
        self._key_counts = {}
        # Press recency: char -> monotonically increasing sequence number, set
        # on every press. get_manual_target() uses it to resolve opposing keys
        # (a vs d, w vs s) by "last key wins" so a new direction takes over
        # instantly instead of cancelling the old one to a centred stop. Stale
        # entries for released keys are harmless -- resolution only consults
        # keys that are currently in _pressed_keys.
        self._press_seq = {}
        self._press_counter = 0
        self._listener = None
        self._stdin_thread = None
        self._stdin_stop = threading.Event()
        self._stdin_restore = None
        # Remote (dashboard) manual override, merged with keyboard state.
        self._remote_target = None

    def start(self):
        if self._start_stdin_listener():
            logger.info("Keyboard override listener active (terminal input: w/s/a/d, space, m).")
            return
        if not PYNPUT_AVAILABLE:
            logger.warning("Keyboard override listener skipped (pynput unavailable).")
            return
        try:
            self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
            self._listener.daemon = True
            self._listener.start()
            logger.info("Keyboard override listener active (w/s drive, a/d steer, space stop, m toggle).")
        except Exception as e:
            logger.warning(f"Keyboard override listener failed to start: {e}")
            self._listener = None

    def stop(self):
        self._stdin_stop.set()
        if self._stdin_thread and self._stdin_thread.is_alive():
            self._stdin_thread.join(timeout=0.5)
        if self._stdin_restore is not None:
            try:
                self._stdin_restore()
            except Exception:
                pass
        if self._listener:
            self._listener.stop()

    # ------------------------------------------------------------- key handling
    def _on_press(self, key):
        char = self._key_char(key)
        if char is None:
            return

        with self._lock:
            if char == KEY_TOGGLE_MANUAL:
                self._manual_mode_active = not self._manual_mode_active
                self._pressed_keys.clear()
                self._key_counts.clear()
                logger.warning(f"Manual mode: {'ENABLED' if self._manual_mode_active else 'DISABLED'}")
                hub.mode(self._manual_mode_active)
            elif char == KEY_STOP:
                self._pressed_keys.clear()
                self._key_counts.clear()
                logger.info("[MANUAL] STOP (spacebar)")
            elif char in (KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT):
                self._pressed_keys.add(char)
                self._key_counts[char] = self._key_counts.get(char, 0) + 1
                self._press_counter += 1
                self._press_seq[char] = self._press_counter
                logger.info(f"[MANUAL] key down: '{char}'")

    def _on_release(self, key):
        char = self._key_char(key)
        if char is None:
            return
        with self._lock:
            if char in self._pressed_keys:
                count = self._key_counts.get(char, 0)
                if count <= 1:
                    self._pressed_keys.discard(char)
                    self._key_counts.pop(char, None)
                else:
                    self._key_counts[char] = count - 1
                logger.info(f"[MANUAL] key up: '{char}'")

    @staticmethod
    def _key_char(key):
        """Normalise a pynput key to a lowercase character we recognise."""
        if keyboard is not None and key == keyboard.Key.space:
            return KEY_STOP
        char = getattr(key, "char", None)
        return char.lower() if char else None

    def _start_stdin_listener(self):
        if os.environ.get("HERMES_KEYBOARD_MODE", "auto").lower() == "pynput":
            return False
        if not sys.stdin.isatty():
            return False
        if os.name == "nt":
            try:
                import msvcrt
            except Exception:
                return False

            def worker():
                while not self._stdin_stop.is_set():
                    if msvcrt.kbhit():
                        char = msvcrt.getwch()
                        self._handle_stdin_char(char)
                    else:
                        time.sleep(0.02)

            self._stdin_thread = threading.Thread(target=worker, daemon=True)
            self._stdin_thread.start()
            return True

        try:
            import select
            import termios
            import tty
        except Exception:
            return False

        fd = sys.stdin.fileno()
        try:
            original = termios.tcgetattr(fd)
        except Exception:
            return False

        def restore():
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, original)
            except Exception:
                pass

        def worker():
            try:
                tty.setraw(fd)
                while not self._stdin_stop.is_set():
                    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if ready:
                        char = sys.stdin.read(1)
                        if char:
                            self._handle_stdin_char(char)
            finally:
                restore()

        self._stdin_restore = restore
        self._stdin_thread = threading.Thread(target=worker, daemon=True)
        self._stdin_thread.start()
        return True

    def _handle_stdin_char(self, char):
        if not char:
            return
        mapped = " " if char in (" ", "\r", "\n") else char.lower()
        if mapped == KEY_STOP:
            logger.info("[MANUAL] STOP (terminal)")
            self._pressed_keys.clear()
            return
        if mapped in (KEY_TOGGLE_MANUAL, KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT):
            class _Key:
                char = mapped
            self._on_press(_Key())

    # ---------------------------------------------------------- remote override
    def set_remote_manual(self, active):
        """Enable/disable manual mode from the dashboard (no keyboard needed)."""
        with self._lock:
            self._manual_mode_active = bool(active)
            if not active:
                self._pressed_keys.clear()
                self._key_counts.clear()
                self._remote_target = None
        logger.warning(f"Manual mode (remote): {'ENABLED' if active else 'DISABLED'}")
        hub.mode(bool(active))

    def set_remote_target(self, speed, steer, action):
        """Push a manual target from the dashboard's on-screen controls."""
        with self._lock:
            self._remote_target = (int(speed), int(steer), str(action).upper())
        logger.info(f"[MANUAL][remote] {action} speed={speed} steer={steer}")

    def clear_remote_target(self):
        """Release the dashboard override so keyboard state drives again."""
        with self._lock:
            self._remote_target = None

    # ------------------------------------------------------------------ queries
    def is_manual_mode_active(self) -> bool:
        with self._lock:
            if self._remote_target is not None:
                return True
            has_drive_input = bool(self._pressed_keys & {KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT})
            return self._manual_mode_active or has_drive_input

    def get_manual_target(self) -> tuple:
        """Resolve current keyboard (or remote) state into a drive target.

        Returns (speed, steer, action). Steering and drive are independent, so
        you can hold 'w'+'a' to arc forward-left, matching how a real RC car
        behaves.
        """
        with self._lock:
            if self._remote_target is not None:
                return self._remote_target

            w = KEY_FORWARD in self._pressed_keys
            s = KEY_BACKWARD in self._pressed_keys
            a = KEY_LEFT in self._pressed_keys
            d = KEY_RIGHT in self._pressed_keys
            seq = dict(self._press_seq)  # stable snapshot for the checks below

            # "Last key wins" for opposing keys, so a new command takes over the
            # instant it's pressed instead of both cancelling to a centred stop.
            # This is what makes a->d (and w->s) flip smoothly with no dead-zone,
            # in every input path: a terminal (where keys latch with no release)
            # and a desktop hold-to-steer session alike. When only one of a pair
            # is held it wins outright; the recency check only matters when both
            # are held at once.
            def _wins(k1, k2):
                return seq.get(k1, 0) >= seq.get(k2, 0)

        steer = 0
        if a and d:
            steer = -STEER_MANUAL_DEGREE if _wins(KEY_LEFT, KEY_RIGHT) else STEER_MANUAL_DEGREE
        elif a:
            steer = -STEER_MANUAL_DEGREE
        elif d:
            steer = STEER_MANUAL_DEGREE

        if w and s:
            if _wins(KEY_FORWARD, KEY_BACKWARD):
                return SPEED_DEFAULT_FORWARD, steer, "FORWARD"
            return SPEED_DEFAULT_BACKWARD, steer, "BACKWARD"
        if w:
            return SPEED_DEFAULT_FORWARD, steer, "FORWARD"
        if s:
            return SPEED_DEFAULT_BACKWARD, steer, "BACKWARD"
        # No drive key held: hold position but still allow the wheels to steer
        # so the operator can pre-aim before moving.
        return SPEED_STOP, steer, "STOP"
