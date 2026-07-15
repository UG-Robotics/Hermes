"""
Startup diagnostics — a single place that reports the health of every hardware
component and, crucially, whether each one is REAL or SIMULATED.

This is what makes "run as if all the hardware works" honest: the boot log
plainly states which subsystems are backed by real hardware and which are
substituted by software, so nobody is fooled into thinking a simulated run
exercised real motors.
"""

from __future__ import annotations

from typing import Dict

from utils.logger import get_logger
from utils.telemetry_hub import get_hub

logger = get_logger(__name__)
hub = get_hub()


def run_startup_diagnostics(simulated: bool, serial_ok: bool, camera_source: str) -> Dict[str, str]:
    """Log and return a component -> status map.

    Status is one of: "REAL", "SIMULATED", or "OFFLINE".
    """
    # The camera is independent of the serial link; classify it on its own.
    if camera_source == "synthetic":
        camera_status = "SIMULATED"
    elif camera_source in ("none", ""):
        camera_status = "OFFLINE"
    else:  # picamera2 / opencv
        camera_status = "REAL"

    if simulated:
        report = {
            "serial_link": "SIMULATED",
            "motor": "SIMULATED",
            "servo": "SIMULATED",
            "imu": "SIMULATED",
            "tof_left": "SIMULATED",
            "tof_right": "SIMULATED",
            "camera": camera_status,
        }
    else:
        link = "REAL" if serial_ok else "OFFLINE"
        # Sensors/actuators are only reachable if the serial link came up.
        sensor = "REAL" if serial_ok else "OFFLINE"
        report = {
            "serial_link": link,
            "motor": sensor,
            "servo": sensor,
            "imu": sensor,
            "tof_left": sensor,
            "tof_right": sensor,
            "camera": camera_status,
        }

    logger.info("---------------- HARDWARE DIAGNOSTICS ----------------")
    for name, status in report.items():
        level = logger.info if status in ("REAL", "SIMULATED") else logger.warning
        level(f"  {name:<12} : {status}")
    logger.info("------------------------------------------------------")

    hub.publish("diagnostics", {"report": report, "simulated": simulated})
    return report
