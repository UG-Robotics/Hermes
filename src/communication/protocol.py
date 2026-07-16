"""
Serial protocol definitions for Pi -> ESP32 communication.

Every packet is self-identifying: the first comma-separated field is a
short type tag, so the receiver (and any future packet types) can be told
apart without guessing based on field count.

Packet types (Pi -> ESP32), built here:
    CMD,<speed>,<steer>,<action>,<mode>\n    - normal motion command
    EMG,<mode>\n                             - emergency stop (always speed=0, steer=0, action=STOP)
    PING\n                                   - liveness check
    ACK,<tag>\n                              - acknowledge a packet received from the ESP32

Packet types (ESP32 -> Pi), parsed by packet_parser.py:
    STATUS,<message>
    TEL,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<tof1_mm>,<tof2_mm>
    ACK,<tag>
    PING
"""

from utils.logger import get_logger
from config.robot_config import SPEED_MIN, SPEED_MAX, STEER_MIN, STEER_MAX

logger = get_logger(__name__)

VALID_ACTIONS = {"FORWARD", "BACKWARD", "STOP"}


def _clamp(val, min_val, max_val, label):
    """Casts val to integer and clamps it within the boundaries [min_val, max_val]."""
    try:
        val = int(val)
    except (TypeError, ValueError):
        logger.error(f"Invalid type for {label}: {val}. Falling back to 0.")
        return 0
    if val < min_val or val > max_val:
        logger.warning(f"{label.capitalize()} {val} out of range. Clamping to [{min_val}, {max_val}].")
        return max(min_val, min(val, max_val))
    return val


def serialize_command(speed: int, steer: int, action: str, mode: int) -> str:
    """Builds a CMD packet: CMD,speed,steer,ACTION,mode\\n

    This is the renamed/updated version of the old serialize_packet(). The
    tag ("CMD") is new — everything else (clamping, action validation) is
    unchanged from your original.
    """
    speed_val = _clamp(speed, SPEED_MIN, SPEED_MAX, "speed")
    steer_val = _clamp(steer, STEER_MIN, STEER_MAX, "steer")

    action_val = str(action).upper().strip()
    if action_val not in VALID_ACTIONS:
        logger.error(f"Invalid action: '{action}'. Defaulting to 'STOP'.")
        action_val = "STOP"
        speed_val = 0

    mode_val = int(mode) if mode in (0, 1) else 1
    return f"CMD,{speed_val},{steer_val},{action_val},{mode_val}\n"


def get_emergency_packet(mode: int = 1) -> str:
    """Builds an EMG packet.

    Previously this returned a CMD packet with action=STOP baked in. It's now
    its own packet type so the ESP32 firmware can recognise and prioritise it
    immediately instead of running it through normal CMD parsing/validation.
    """
    mode_val = int(mode) if mode in (0, 1) else 1
    return f"EMG,{mode_val}\n"


def build_ping_packet() -> str:
    """Builds a PING packet, used to check the link is alive in both directions."""
    return "PING\n"


def build_ack_packet(tag: str = "") -> str:
    """Builds an ACK packet. tag is optional and lets the other side match the
    ACK to a specific outgoing packet if you start numbering packets later."""
    return f"ACK,{tag}\n" if tag else "ACK\n"


serialize_packet = serialize_command