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


def run_startup_diagnostics(simulated: bool, serial_ok: bool, camera_source: str,
                             telemetry_confirmed: bool = False) -> Dict[str, str]:
    """Log and return a component -> status map.

    Status is one of: "REAL", "SIMULATED", "UNCONFIRMED", or "OFFLINE".

    IMPORTANT ownership note: the IMU and both ToF sensors physically live on
    the ESP32, never the Pi. The Pi cannot know they're healthy just because
    the serial link came up -- the ESP32 itself could be alive while its I2C
    sensors failed to init. `telemetry_confirmed` must come from Runtime
    actually having seen a real TEL packet (see Runtime._await_first_telemetry)
    -- it is the only honest signal of sensor health available on this side
    of the wire. Reporting "REAL" without that confirmation would be the Pi
    asserting something about hardware it doesn't own and can't verify.
    """
    # The camera is independent of the serial link; classify it on its own.
    if camera_source == "synthetic":
        camera_status = "SIMULATED"
    elif camera_source in ("none", ""):
        camera_status = "OFFLINE"
    else:  # picamera2 / opencv
        camera_status = "REAL"

    link_status = "SIMULATED" if simulated else ("REAL" if serial_ok else "OFFLINE")
    # motor/servo are outputs the Pi commands -- their status reflects
    # whether commands CAN be sent, which is a fair thing for the Pi to
    # assert. Sensors are inputs the Pi cannot self-certify; see note above.
    actuator_status = "SIMULATED" if simulated else ("REAL" if serial_ok else "OFFLINE")

    if not serial_ok:
        sensor_status = "OFFLINE"
    elif telemetry_confirmed:
        sensor_status = "SIMULATED" if simulated else "REAL"
    else:
        sensor_status = "UNCONFIRMED"  # link is up, but no TEL packet seen yet

    report = {
        "serial_link": link_status,
        "motor": actuator_status,
        "servo": actuator_status,
        "imu": sensor_status,
        "tof_left": sensor_status,
        "tof_right": sensor_status,
        "camera": camera_status,
    }

    logger.info("---------------- HARDWARE DIAGNOSTICS ----------------")
    for name, status in report.items():
        level = logger.info if status in ("REAL", "SIMULATED") else logger.warning
        level(f"  {name:<12} : {status}")
    logger.info("------------------------------------------------------")

    hub.publish("diagnostics", {"report": report, "simulated": simulated})
    return report
