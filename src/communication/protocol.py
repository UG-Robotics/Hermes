import logging
from config.robot_config import SPEED_MIN, SPEED_MAX, STEER_MIN, STEER_MAX

logger = logging.getLogger(__name__)

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

def serialize_packet(speed: int, steer: int, action: str, mode: int) -> str:
    """Serializes motion targets into SPEED,STEER,ACTION,MODE text packet."""
    speed_val = _clamp(speed, SPEED_MIN, SPEED_MAX, "speed")
    steer_val = _clamp(steer, STEER_MIN, STEER_MAX, "steer")
    
    action_val = str(action).upper().strip()
    if action_val not in VALID_ACTIONS:
        logger.error(f"Invalid action: '{action}'. Defaulting to 'STOP'.")
        action_val = "STOP"
        speed_val = 0

    mode_val = int(mode) if mode in (0, 1) else 1
    return f"{speed_val},{steer_val},{action_val},{mode_val}\n"

def get_emergency_packet(mode: int = 1) -> str:
    """Generates a safe emergency stop packet."""
    return f"0,0,STOP,{mode}\n"
