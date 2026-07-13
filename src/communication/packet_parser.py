# from utils.logger import get_logger

# logger = get_logger(__name__)


# def parse_incoming(line: str):
#     """
#     Parses a single line received from the MCU over serial.

#     Currently the Arduino only sends plain status strings (e.g. on boot:
#     "System initialized: RUNNING state activated."). This function classifies
#     and logs those, and gives you a single place to extend parsing if the
#     firmware ever starts sending structured telemetry (e.g. "ACK,1" or
#     "ERR,WATCHDOG").

#     Returns a dict like {"type": "status", "raw": line} or
#     {"type": "unknown", "raw": line}. Never raises.
#     """
#     if not line:
#         return None

#     text = line.strip()
#     if not text:
#         return None

#     if text.startswith("System initialized"):
#         logger.info(f"[MCU] {text}")
#         return {"type": "status", "raw": text}

#     # Extend here if firmware starts sending comma-delimited telemetry, e.g.:
#     # if "," in text:
#     #     parts = text.split(",")
#     #     ...

#     logger.debug(f"[MCU][unrecognized] {text}")
#     return {"type": "unknown", "raw": text}

from utils.logger import get_logger

logger = get_logger(__name__)

# Fields expected after "TEL," — ax, ay, az, gx, gy, gz (LSM6DSOX 6-axis IMU)
# plus tof1_mm, tof2_mm.
#
# ASSUMPTION: 2 ToF sensors. 
# if it's different, change TEL_FIELD_NAMES here to match, and update
# sendTelemetry() in serial_protocol.cpp to match the same order/count.
TEL_FIELD_NAMES = ["ax", "ay", "az", "gx", "gy", "gz", "tof1_mm", "tof2_mm"]


def parse_incoming(line: str):
    """
    Parses a single self-identifying line received from the ESP32 over serial.

    Returns a dict shaped by packet type, or None for blank input:
        {"type": "status",    "raw": line, "message": str}
        {"type": "telemetry", "raw": line, "ax":..., "ay":..., ..., "tof2_mm":...}
        {"type": "ack",       "raw": line, "tag": str}
        {"type": "ping",      "raw": line}
        {"type": "unknown",   "raw": line}

    Never raises.
    """
    if not line:
        return None

    text = line.strip()
    if not text:
        return None

    parts = text.split(",")
    tag = parts[0].strip().upper()

    if tag == "STATUS":
        message = ",".join(parts[1:]).strip() if len(parts) > 1 else ""
        logger.info(f"[MCU][STATUS] {message}")
        return {"type": "status", "raw": text, "message": message}

    if tag == "TEL":
        values = parts[1:]
        if len(values) != len(TEL_FIELD_NAMES):
            logger.warning(
                f"[MCU][TEL] expected {len(TEL_FIELD_NAMES)} fields, got {len(values)}: {text}"
            )
            return {"type": "unknown", "raw": text}
        try:
            floats = [float(v) for v in values]
        except ValueError:
            logger.warning(f"[MCU][TEL] non-numeric field in: {text}")
            return {"type": "unknown", "raw": text}
        telemetry = dict(zip(TEL_FIELD_NAMES, floats))
        telemetry["type"] = "telemetry"
        telemetry["raw"] = text
        logger.debug(f"[MCU][TEL] {telemetry}")
        return telemetry

    if tag == "ACK":
        ack_tag = parts[1].strip() if len(parts) > 1 else ""
        logger.debug(f"[MCU][ACK] {ack_tag or '(untagged)'}")
        return {"type": "ack", "raw": text, "tag": ack_tag}

    if tag == "PING":
        logger.debug("[MCU][PING]")
        return {"type": "ping", "raw": text}

    # Legacy boot line from the old test firmware, kept so you don't lose
    # visibility during the transition to the new firmware below.
    if text.startswith("System initialized"):
        logger.info(f"[MCU] {text}")
        return {"type": "status", "raw": text, "message": text}

    logger.debug(f"[MCU][unrecognized] {text}")
    return {"type": "unknown", "raw": text}