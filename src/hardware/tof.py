"""
Time-of-Flight (ToF) distance sensor interface.

Two ToF sensors (left and right) sit on the ESP32 and report inside the TEL
telemetry packet as tof1_mm / tof2_mm. This class exposes them as named
left/right distances for the wall-following logic, reading from the same
TelemetryHub as everything else so real and simulated data are interchangeable.
"""

from __future__ import annotations

from typing import Optional

from utils.logger import get_logger
from utils.telemetry_hub import get_hub, CH_TELEMETRY

logger = get_logger(__name__)


class ToFArray:
    # Convention: tof1 = left, tof2 = right. If the physical wiring differs,
    # flip it here in one place.
    OUT_OF_RANGE_MM = 2000.0

    def __init__(self, simulated: bool = False):
        self._hub = get_hub()
        self.simulated = simulated
        source = "SIMULATED" if simulated else "ESP32 telemetry"
        logger.info(f"ToF array ready (left+right, source: {source}).")

    def _latest(self) -> Optional[dict]:
        rec = self._hub.snapshot().get(CH_TELEMETRY)
        return rec["values"] if rec else None

    def read(self) -> dict:
        """Return {'left_mm', 'right_mm'}. Out-of-range before any telemetry."""
        values = self._latest()
        if not values:
            return {"left_mm": self.OUT_OF_RANGE_MM, "right_mm": self.OUT_OF_RANGE_MM}
        return {
            "left_mm": values.get("tof1_mm", self.OUT_OF_RANGE_MM),
            "right_mm": values.get("tof2_mm", self.OUT_OF_RANGE_MM),
        }

    @property
    def left(self) -> float:
        return self.read()["left_mm"]

    @property
    def right(self) -> float:
        return self.read()["right_mm"]

    def healthy(self) -> bool:
        return self._latest() is not None
