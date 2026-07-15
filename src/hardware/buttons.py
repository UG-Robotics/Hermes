"""
Manual driving via the laptop keyboard.

Keys (hold to act, release to stop):
    w  - forward
    s  - backward
    a  - steer left
    d  - steer right
    space - immediate stop / straighten wheels
    m  - toggle MANUAL <-> AUTONOMOUS

The listener runs in a background thread using pynput so it never blocks the
20 Hz control loop. If pynput is unavailable (e.g. headless Pi with no X, or a
permissions issue) it degrades to a no-op with manual mode permanently off,
instead of crashing — the dashboard can still drive manually over HTTP.
"""

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
        self._listener = None
        # Remote (dashboard) manual override, merged with keyboard state.
        self._remote_target = None

    def start(self):
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
                logger.warning(f"Manual mode: {'ENABLED' if self._manual_mode_active else 'DISABLED'}")
                hub.mode(self._manual_mode_active)
            elif char == KEY_STOP:
                self._pressed_keys.clear()
                logger.info("[MANUAL] STOP (spacebar)")
            elif char in (KEY_FORWARD, KEY_BACKWARD, KEY_LEFT, KEY_RIGHT):
                if char not in self._pressed_keys:
                    self._pressed_keys.add(char)
                    logger.info(f"[MANUAL] key down: '{char}'")

    def _on_release(self, key):
        char = self._key_char(key)
        if char is None:
            return
        with self._lock:
            if char in self._pressed_keys:
                self._pressed_keys.discard(char)
                logger.info(f"[MANUAL] key up: '{char}'")

    @staticmethod
    def _key_char(key):
        """Normalise a pynput key to a lowercase character we recognise."""
        if keyboard is not None and key == keyboard.Key.space:
            return KEY_STOP
        char = getattr(key, "char", None)
        return char.lower() if char else None

    # ---------------------------------------------------------- remote override
    def set_remote_manual(self, active):
        """Enable/disable manual mode from the dashboard (no keyboard needed)."""
        with self._lock:
            self._manual_mode_active = bool(active)
            if not active:
                self._pressed_keys.clear()
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
            return self._manual_mode_active

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

        steer = 0
        if a and not d:
            steer = -STEER_MANUAL_DEGREE
        elif d and not a:
            steer = STEER_MANUAL_DEGREE

        if w and not s:
            return SPEED_DEFAULT_FORWARD, steer, "FORWARD"
        if s and not w:
            return SPEED_DEFAULT_BACKWARD, steer, "BACKWARD"
        # No drive key held: hold position but still allow the wheels to steer
        # so the operator can pre-aim before moving.
        return SPEED_STOP, steer, "STOP"
