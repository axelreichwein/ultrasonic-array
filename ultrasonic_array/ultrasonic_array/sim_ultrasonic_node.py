from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimUltrasonicNode(Node):
    """Publishes deterministic ultrasonic data to support integration testing."""

    def __init__(self) -> None:
        super().__init__("sim_ultrasonic")

        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("topic", "/ultrasonic/raw_mm")
        self.declare_parameter("sensor_count", 4)
        self.declare_parameter("obstacle_after_frames", 30)
        self.declare_parameter("safe_distance_mm", 650.0)
        self.declare_parameter("obstacle_distance_mm", 120.0)

        publish_hz = float(self.get_parameter("publish_hz").value)
        topic = str(self.get_parameter("topic").value)
        self._sensor_count = int(self.get_parameter("sensor_count").value)
        self._obstacle_after_frames = int(
            self.get_parameter("obstacle_after_frames").value
        )
        self._safe_distance_mm = float(self.get_parameter("safe_distance_mm").value)
        self._obstacle_distance_mm = float(
            self.get_parameter("obstacle_distance_mm").value
        )
        self._frame = 0

        self._pub = self.create_publisher(Float32MultiArray, topic, 10)
        period_sec = 1.0 / max(publish_hz, 0.1)
        self._timer = self.create_timer(period_sec, self._publish)

        self.get_logger().info("sim_ultrasonic started for integration testing")

    def _publish(self) -> None:
        frame_data = [self._safe_distance_mm] * self._sensor_count
        if self._frame >= self._obstacle_after_frames:
            frame_data[1] = self._obstacle_distance_mm
        self._frame += 1
        self._pub.publish(Float32MultiArray(data=frame_data))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
