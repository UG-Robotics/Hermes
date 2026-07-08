import threading
from utils.logger import get_logger
from config.robot_config import (
    KEY_TOGGLE_MANUAL,
    KEY_FORWARD,
    KEY_BACKWARD,
    SPEED_DEFAULT_FORWARD,
    SPEED_DEFAULT_BACKWARD,
    SPEED_STOP
)

logger = get_logger(__name__)

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except Exception as e:
    keyboard = None
    PYNPUT_AVAILABLE = False
    logger.warning(f"pynput unavailable at import time: {e}. Manual override disabled.")


class KeyboardOverrideListener:
    """Background listener using pynput to capture manual override keys.

    Degrades to a no-op (manual mode permanently off) if pynput can't be
    imported or its hook fails to start, instead of crashing the program.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._manual_mode_active = False
        self._pressed_keys = set()
        self._listener = None

    def start(self):
        if not PYNPUT_AVAILABLE:
            logger.warning("Keyboard override listener skipped (pynput unavailable).")
            return
        try:
            self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
            self._listener.daemon = True
            self._listener.start()
            logger.info("Keyboard override listener active.")
        except Exception as e:
            logger.warning(f"Keyboard override listener failed to start: {e}")
            self._listener = None

    def stop(self):
        if self._listener:
            self._listener.stop()

    def _on_press(self, key):
        char = getattr(key, 'char', None)
        if not char:
            return
        char = char.lower()

        with self._lock:
            if char == KEY_TOGGLE_MANUAL:
                self._manual_mode_active = not self._manual_mode_active
                logger.warning(f"Manual mode: {'ENABLED' if self._manual_mode_active else 'DISABLED'}")
            elif char in (KEY_FORWARD, KEY_BACKWARD):
                self._pressed_keys.add(char)

    def _on_release(self, key):
        char = getattr(key, 'char', None)
        if not char:
            return
        char = char.lower()

        with self._lock:
            if char in (KEY_FORWARD, KEY_BACKWARD):
                self._pressed_keys.discard(char)

    def is_manual_mode_active(self) -> bool:
        with self._lock:
            return self._manual_mode_active

    def get_manual_target(self) -> tuple:
        with self._lock:
            w_pressed = KEY_FORWARD in self._pressed_keys
            s_pressed = KEY_BACKWARD in self._pressed_keys

            if w_pressed and not s_pressed:
                return SPEED_DEFAULT_FORWARD, 0, "FORWARD"
            elif s_pressed and not w_pressed:
                return SPEED_DEFAULT_BACKWARD, 0, "BACKWARD"
            return SPEED_STOP, 0, "STOP"