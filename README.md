# Ultrasonic Array (AN / ROS 2)

This workspace contains a ROS 2 package implementing near-field obstacle handling for
a 4-sensor ultrasonic array (HC-SR04 industrial equivalent transducers) in the
Autonomous Navigation (AN) subsystem.

## Implemented behavior

- `lidar_proc` node processes raw ultrasonic ranges at **10 Hz**.
- Emergency stop is triggered if any sensor reports **<= 150 mm**.
- A **50 mm compliance margin** is tracked against the 200 mm stopping requirement.
- Processed distances are published on **`/scan`** (`sensor_msgs/LaserScan`).
- Safety stop output is published to:
  - `/an/emergency_stop`
  - `/an/plc/estop_req` (for wiring into PLC I/O bridge)
- Optional software stop command publishes zero velocity to `/cmd_vel`.

## Safety architecture note

The software layer in this package is part of the mission computer ROS 2 stack.
The Safety PLC (Siemens S7-1200F) remains an independent hardwired protection layer
and should execute emergency stop regardless of mission-computer state.

## Package layout

- `ultrasonic_array/` ROS 2 package (`ament_python`)
- `ultrasonic_array/ultrasonic_array/lidar_proc_node.py` main node
- `ultrasonic_array/ultrasonic_array/near_field_processor.py` core logic
- `.github/workflows/nav_integration.yml` CI with Gazebo smoke test + verification
- `ultrasonic_array/test/test_vvr_008_static_obstacle_stop.py` VVR-008 test

## Local build and test

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test --packages-select ultrasonic_array
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ultrasonic_array lidar_proc --ros-args --params-file ultrasonic_array/config/lidar_proc.yaml
```
