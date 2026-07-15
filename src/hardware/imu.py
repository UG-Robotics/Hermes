"""
IMU interface.

The IMU physically lives on the ESP32; its readings reach the Pi inside TEL
telemetry packets. This class is the clean, higher-level view of that data: the
rest of the software asks the IMU for acceleration / angular rate / heading and
never has to know the numbers arrived over serial (real or simulated).

It reads the latest values from the TelemetryHub, so it behaves identically
whether a real ESP32 or the virtual one produced them. Heading is integrated
from the gz (yaw-rate) channel, which is exactly what the wall-follower needs
and mirrors the gyro-free fusion approach used on the sibling robot.
"""

from __future__ import annotations

import time
from typing import Optional

from utils.logger import get_logger
from utils.telemetry_hub import get_hub, CH_TELEMETRY

logger = get_logger(__name__)


class IMU:
    def __init__(self, simulated: bool = False):
        self._hub = get_hub()
        self.simulated = simulated
        self._heading_deg = 0.0
        self._last_ts: Optional[float] = None
        source = "SIMULATED" if simulated else "ESP32 telemetry"
        logger.info(f"IMU interface ready (source: {source}).")

    def _latest(self) -> Optional[dict]:
        rec = self._hub.snapshot().get(CH_TELEMETRY)
        return rec["values"] if rec else None

    def read(self) -> dict:
        """Return the latest IMU reading, integrating heading from yaw rate.

        Returns a dict with ax, ay, az, gx, gy, gz and a derived heading_deg.
        Falls back to zeros (level, stationary) before any telemetry arrives.
        """
        values = self._latest()
        if not values:
            return {"ax": 0.0, "ay": 0.0, "az": 9.81,
                    "gx": 0.0, "gy": 0.0, "gz": 0.0, "heading_deg": self._heading_deg}

        now = time.time()
        if self._last_ts is not None:
            dt = now - self._last_ts
            # gz is deg/s; integrate to a heading estimate.
            self._heading_deg += values.get("gz", 0.0) * dt
            self._heading_deg = (self._heading_deg + 180.0) % 360.0 - 180.0
        self._last_ts = now

        return {**values, "heading_deg": self._heading_deg}

    @property
    def heading(self) -> float:
        return self._heading_deg

    def reset_heading(self) -> None:
        self._heading_deg = 0.0
        self._last_ts = None
        logger.info("IMU heading reset to 0.")

    def healthy(self) -> bool:
        """True once at least one telemetry packet has been seen."""
        return self._latest() is not None
