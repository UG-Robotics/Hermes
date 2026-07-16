import sys
import pathlib
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from hardware.buttons import KeyboardOverrideListener


class _KeyStub:
    def __init__(self, char):
        self.char = char


class TestKeyboardOverrideListener(unittest.TestCase):
    def test_repeated_press_events_require_matching_releases(self):
        listener = KeyboardOverrideListener()

        listener._on_press(_KeyStub("w"))
        listener._on_press(_KeyStub("w"))
        self.assertTrue(listener.is_manual_mode_active() is False)
        self.assertIn("w", listener._pressed_keys)
        self.assertEqual(listener._key_counts.get("w", 0), 2)

        listener._on_release(_KeyStub("w"))
        self.assertIn("w", listener._pressed_keys)

        listener._on_release(_KeyStub("w"))
        self.assertNotIn("w", listener._pressed_keys)


if __name__ == "__main__":
    unittest.main()
