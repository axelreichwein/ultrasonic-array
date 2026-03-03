from math import inf, nan

import pytest

from ultrasonic_array.near_field_processor import NearFieldProcessor


def test_emergency_stop_triggers_at_or_below_150_mm() -> None:
    processor = NearFieldProcessor(sensor_count=4, emergency_stop_mm=150.0)
    result = processor.process([340.0, 151.0, 150.0, 900.0])
    assert result.nearest_mm == 150.0
    assert result.emergency_stop


def test_invalid_ranges_are_safely_clamped() -> None:
    processor = NearFieldProcessor(sensor_count=4, min_valid_mm=20.0, max_valid_mm=4000.0)
    result = processor.process([nan, inf, 10.0, 4500.0])
    assert result.filtered_mm == [4000.0, 4000.0, 20.0, 4000.0]


def test_sensor_count_mismatch_raises_value_error() -> None:
    processor = NearFieldProcessor(sensor_count=4)
    with pytest.raises(ValueError):
        processor.process([1.0, 2.0, 3.0])

