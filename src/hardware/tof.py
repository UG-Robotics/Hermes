"""
Time-of-Flight (ToF) distance sensor interface.

OWNERSHIP: both ToF sensors are physical hardware wired to the ESP32, not the
Pi. As with hardware/imu.py, the Pi does no I2C/GPIO of its own here -- it
only reads the tof1_mm/tof2_mm fields out of the latest TEL packet the ESP32
published to the TelemetryHub (see communication/packet_parser.py). There is
no Pi-side computation on this data at all (unlike the IMU's heading
integration) -- it's a pure pass-through, named left/right for readability.
If a ToF driver ever needs writing, it belongs in firmware/esp_controller/,
next to imu.cpp, not here.
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
