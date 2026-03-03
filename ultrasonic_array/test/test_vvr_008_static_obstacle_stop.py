"""
VVR-008: Static obstacle stop test.

Requirement interpretation:
- The stop trigger threshold is 150 mm.
- A 50 mm safety margin is applied for compliance accounting.
- compliance_distance_mm must stay <= 200 mm for 20 trials.
"""

from ultrasonic_array.near_field_processor import NearFieldProcessor


def test_vvr_008_static_obstacle_stop_20_trials() -> None:
    processor = NearFieldProcessor(
        sensor_count=4,
        emergency_stop_mm=150.0,
        stopping_margin_mm=50.0,
    )

    # 20 deterministic obstacle placements inside threshold across four sensors.
    obstacle_distances_mm = [
        149.0,
        148.5,
        147.0,
        145.0,
        143.0,
        142.5,
        140.0,
        139.0,
        138.0,
        137.5,
        136.0,
        135.0,
        134.0,
        133.5,
        132.0,
        131.0,
        130.0,
        129.0,
        128.0,
        127.0,
    ]

    for idx, obstacle_mm in enumerate(obstacle_distances_mm):
        readings = [650.0, 700.0, 720.0, 680.0]
        readings[idx % 4] = obstacle_mm
        result = processor.process(readings)

        assert result.emergency_stop, f"Trial {idx + 1}: emergency stop did not trigger"
        assert (
            result.compliance_distance_mm <= 200.0
        ), f"Trial {idx + 1}: compliance distance exceeded 200 mm"

