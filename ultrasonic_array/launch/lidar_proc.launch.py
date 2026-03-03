from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ultrasonic_array"))
    params_file = package_share / "config" / "lidar_proc.yaml"

    lidar_proc = Node(
        package="ultrasonic_array",
        executable="lidar_proc",
        name="lidar_proc",
        output="screen",
        parameters=[str(params_file)],
    )

    return LaunchDescription([lidar_proc])
