from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray

from .near_field_processor import NearFieldProcessor


class LidarProcNode(Node):
    """ROS 2 node that converts ultrasonic transducer data into near-field hazards."""

    def __init__(self) -> None:
        super().__init__("lidar_proc")

        self.declare_parameter("sensor_count", 4)
        self.declare_parameter("process_hz", 10.0)
        self.declare_parameter("emergency_stop_mm", 150.0)
        self.declare_parameter("stopping_margin_mm", 50.0)
        self.declare_parameter("min_valid_mm", 20.0)
        self.declare_parameter("max_valid_mm", 4000.0)
        self.declare_parameter("raw_topic", "/ultrasonic/raw_mm")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("emergency_stop_topic", "/an/emergency_stop")
        self.declare_parameter("nearest_topic", "/an/nearest_obstacle_mm")
        self.declare_parameter("plc_topic", "/an/plc/estop_req")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_stop_cmd", True)
        self.declare_parameter("fov_deg", 90.0)

        sensor_count = int(self.get_parameter("sensor_count").value)
        process_hz = float(self.get_parameter("process_hz").value)
        emergency_stop_mm = float(self.get_parameter("emergency_stop_mm").value)
        stopping_margin_mm = float(self.get_parameter("stopping_margin_mm").value)
        min_valid_mm = float(self.get_parameter("min_valid_mm").value)
        max_valid_mm = float(self.get_parameter("max_valid_mm").value)

        self._processor = NearFieldProcessor(
            sensor_count=sensor_count,
            emergency_stop_mm=emergency_stop_mm,
            stopping_margin_mm=stopping_margin_mm,
            min_valid_mm=min_valid_mm,
            max_valid_mm=max_valid_mm,
        )

        raw_topic = str(self.get_parameter("raw_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        emergency_stop_topic = str(self.get_parameter("emergency_stop_topic").value)
        nearest_topic = str(self.get_parameter("nearest_topic").value)
        plc_topic = str(self.get_parameter("plc_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_stop_cmd = bool(self.get_parameter("publish_stop_cmd").value)
        self._fov_deg = float(self.get_parameter("fov_deg").value)
        self._min_valid_m = min_valid_mm / 1000.0
        self._max_valid_m = max_valid_mm / 1000.0

        self._latest_raw_mm: Optional[List[float]] = None
        self._last_emergency_stop = False

        self._raw_sub = self.create_subscription(
            Float32MultiArray, raw_topic, self._raw_cb, 10
        )
        self._scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self._estop_pub = self.create_publisher(Bool, emergency_stop_topic, 10)
        self._nearest_pub = self.create_publisher(Float32, nearest_topic, 10)
        self._plc_pub = self.create_publisher(Bool, plc_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        period_sec = 1.0 / max(process_hz, 0.1)
        self._scan_time = period_sec
        self._timer = self.create_timer(period_sec, self._process_cycle)
        self.get_logger().info(
            f"lidar_proc active at {process_hz:.1f} Hz with {sensor_count} sensors"
        )

    def _raw_cb(self, msg: Float32MultiArray) -> None:
        values = list(msg.data)
        expected = self._processor.sensor_count
        if len(values) != expected:
            self.get_logger().warning(
                f"Discarded raw frame with {len(values)} values (expected {expected})"
            )
            return
        self._latest_raw_mm = values

    def _process_cycle(self) -> None:
        if self._latest_raw_mm is None:
            return

        result = self._processor.process(self._latest_raw_mm)
        self._publish_scan(result.filtered_mm)
        self._nearest_pub.publish(Float32(data=float(result.nearest_mm)))

        estop_msg = Bool(data=bool(result.emergency_stop))
        self._estop_pub.publish(estop_msg)
        self._plc_pub.publish(estop_msg)

        if result.emergency_stop and self._publish_stop_cmd:
            self._cmd_vel_pub.publish(Twist())

        if result.emergency_stop and not self._last_emergency_stop:
            self.get_logger().error(
                "Emergency stop triggered: nearest obstacle %.1f mm"
                % result.nearest_mm
            )
        self._last_emergency_stop = result.emergency_stop

    def _publish_scan(self, filtered_mm: List[float]) -> None:
        count = len(filtered_mm)
        fov_rad = self._fov_deg * 3.141592653589793 / 180.0

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._frame_id
        scan.angle_min = -0.5 * fov_rad
        scan.angle_max = 0.5 * fov_rad
        scan.angle_increment = fov_rad / max(count - 1, 1)
        scan.time_increment = 0.0
        scan.scan_time = self._scan_time
        scan.range_min = self._min_valid_m
        scan.range_max = self._max_valid_m
        scan.ranges = [value / 1000.0 for value in filtered_mm]
        self._scan_pub.publish(scan)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarProcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
