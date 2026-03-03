"""
Corridor tracking + obstacle response integration logic.

This test emulates the near-field stream generated during a corridor run:
- initially clear path in all transducers
- then one transducer detects a close obstacle
- system must transition into emergency-stop state
"""

from ultrasonic_array.near_field_processor import NearFieldProcessor


def test_corridor_obstacle_transition() -> None:
    processor = NearFieldProcessor(sensor_count=4, emergency_stop_mm=150.0)

    safe_frames = [[600.0, 610.0, 590.0, 605.0] for _ in range(30)]
    obstacle_frames = [[600.0, 120.0, 590.0, 605.0] for _ in range(10)]
    frames = safe_frames + obstacle_frames

    estop_flags = [processor.process(frame).emergency_stop for frame in frames]

    assert not any(estop_flags[:30]), "False-positive stop while corridor is clear"
    assert all(estop_flags[30:]), "Stop was not sustained for obstacle frames"

