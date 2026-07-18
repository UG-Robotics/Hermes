import unittest
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from communication.protocol import (
    serialize_command,
    get_emergency_packet,
    build_ping_packet,
    build_ack_packet,
)

class TestProtocolSerialization(unittest.TestCase):
    def test_valid_serialization(self):
        # Format: SPEED,STEER,ACTION,MODE\n
        self.assertEqual(
            serialize_command(150, 0, "FORWARD", 1),
            "CMD,150,0,FORWARD,1\n"
        )
        self.assertEqual(
            serialize_command(100, -32, "BACKWARD", 0),
            "CMD,100,-32,BACKWARD,0\n"
        )
        self.assertEqual(
            serialize_command(0, 90, "STOP", 0),
            "CMD,0,90,STOP,0\n"
        )

    def test_out_of_bounds_speed(self):
        # Speed above 255 should clamp to 255
        self.assertEqual(
            serialize_command(300, 0, "FORWARD", 1),
            "CMD,255,0,FORWARD,1\n"
        )
        # Speed below 0 should clamp to 0
        self.assertEqual(
            serialize_command(-50, 0, "FORWARD", 1),
            "CMD,0,0,FORWARD,1\n"
        )

    def test_out_of_bounds_steer(self):
        # Steer angle above 90 should clamp to 90
        self.assertEqual(
            serialize_command(150, 100, "FORWARD", 1),
            "CMD,150,90,FORWARD,1\n"
        )
        # Steer angle below -90 should clamp to -90
        self.assertEqual(
            serialize_command(150, -110, "FORWARD", 1),
            "CMD,150,-90,FORWARD,1\n"
        )

    def test_invalid_types_and_defaults(self):
        # Invalid speed/steer strings should fall back to 0
        self.assertEqual(
            serialize_command("invalid_speed", "invalid_steer", "FORWARD", 1),
            "CMD,0,0,FORWARD,1\n"
        )
        # Invalid action should default to STOP (and force speed = 0)
        self.assertEqual(
            serialize_command(150, 0, "FLY_UP", 1),
            "CMD,0,0,STOP,1\n"
        )
        # Invalid mode should fall back to 1
        self.assertEqual(
            serialize_command(150, 0, "FORWARD", 5),
            "CMD,150,0,FORWARD,1\n"
        )

    def test_emergency_packet(self):
        self.assertEqual(get_emergency_packet(1), "EMG,1\n")
        self.assertEqual(get_emergency_packet(0), "EMG,0\n")

    def test_ping_and_ack_packets(self):
        self.assertEqual(build_ping_packet(), "PING\n")
        self.assertEqual(build_ack_packet(), "ACK\n")
        self.assertEqual(build_ack_packet("PING"), "ACK,PING\n")

if __name__ == "__main__":
    unittest.main()
