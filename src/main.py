from utils.logger import get_logger

logger = get_logger(__name__)


def parse_incoming(line: str):
    """
    Parses a single line received from the MCU over serial.

    Currently the Arduino only sends plain status strings (e.g. on boot:
    "System initialized: RUNNING state activated."). This function classifies
    and logs those, and gives you a single place to extend parsing if the
    firmware ever starts sending structured telemetry (e.g. "ACK,1" or
    "ERR,WATCHDOG").

    Returns a dict like {"type": "status", "raw": line} or
    {"type": "unknown", "raw": line}. Never raises.
    """
    if not line:
        return None

    text = line.strip()
    if not text:
        return None

    if text.startswith("System initialized"):
        logger.info(f"[MCU] {text}")
        return {"type": "status", "raw": text}

    # Extend here if firmware starts sending comma-delimited telemetry, e.g.:
    # if "," in text:
    #     parts = text.split(",")
    #     ...

    logger.debug(f"[MCU][unrecognized] {text}")
    return {"type": "unknown", "raw": text}