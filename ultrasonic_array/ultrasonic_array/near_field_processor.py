from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, List


@dataclass(frozen=True)
class ProcessResult:
    filtered_mm: List[float]
    nearest_mm: float
    emergency_stop: bool
    compliance_distance_mm: float


class NearFieldProcessor:
    """Pure processing logic for ultrasonic near-field hazard detection."""

    def __init__(
        self,
        sensor_count: int = 4,
        emergency_stop_mm: float = 150.0,
        stopping_margin_mm: float = 50.0,
        min_valid_mm: float = 20.0,
        max_valid_mm: float = 4000.0,
    ) -> None:
        if sensor_count <= 0:
            raise ValueError("sensor_count must be > 0")
        self._sensor_count = sensor_count
        self._emergency_stop_mm = emergency_stop_mm
        self._stopping_margin_mm = stopping_margin_mm
        self._min_valid_mm = min_valid_mm
        self._max_valid_mm = max_valid_mm

    @property
    def sensor_count(self) -> int:
        return self._sensor_count

    def process(self, raw_mm: Iterable[float]) -> ProcessResult:
        raw = list(raw_mm)
        if len(raw) != self._sensor_count:
            raise ValueError(
                f"expected {self._sensor_count} sensor readings, got {len(raw)}"
            )

        filtered = [self._normalize_range_mm(value) for value in raw]
        nearest_mm = min(filtered)
        emergency_stop = nearest_mm <= self._emergency_stop_mm
        compliance_distance = nearest_mm + self._stopping_margin_mm

        return ProcessResult(
            filtered_mm=filtered,
            nearest_mm=nearest_mm,
            emergency_stop=emergency_stop,
            compliance_distance_mm=compliance_distance,
        )

    def _normalize_range_mm(self, value: float) -> float:
        if not isfinite(value):
            return self._max_valid_mm

        if value < self._min_valid_mm:
            return self._min_valid_mm
        if value > self._max_valid_mm:
            return self._max_valid_mm
        return value
