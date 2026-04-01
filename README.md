# gps_speed

A ROS2 Jazzy node that estimates robot ground speed (m/s) from successive GPS fixes using the Haversine formula. Designed for use with Clearpath Warthog robots but applicable to any ROS2 platform with a u-blox GPS and Microstrain IMU.

## Overview

Ground speed is computed by differencing successive `NavSatFix` messages. A rotation gate (using IMU angular velocity) suppresses false speed readings when the robot is spinning in place. If the GPS displacement is large enough, the rotation gate is overridden — allowing accurate speed reporting during high-speed turns.

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribes | `/{robot_name}/sensors/ublox/fix` | `sensor_msgs/NavSatFix` | GPS fix |
| Subscribes | `/{robot_name}/sensors/microstrain/ekf/imu/data` | `sensor_msgs/Imu` | IMU for rotation gate |
| Publishes | `/{robot_name}/gps_speed` | `std_msgs/Float64` | Ground speed in m/s |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_name` | string | **required** | Robot namespace, e.g. `warthog1` |
| `min_time_delta_s` | double | `0.05` | Min seconds between GPS samples |
| `min_distance_m` | double | `0.05` | Min displacement (m) before reporting non-zero speed |
| `max_speed_m_s` | double | `20.0` | Sanity cap — discard GPS jumps above this |
| `max_rotation_rate_rad_s` | double | `0.2` | IMU yaw rate (rad/s) above which speed is forced to 0.0 |
| `rotation_gate_override_distance_m` | double | `0.3` | GPS displacement (m) above which the rotation gate is bypassed |

## Installation

```bash
cd ~/your_ws/src
git clone <this-repo-url>
cd ~/your_ws
colcon build --packages-select gps_speed
source install/setup.bash
```

## Usage

```bash
# Single robot
ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1

# Override rotation gate threshold
ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 max_rotation_rate_rad_s:=0.3

# Verify output
ros2 topic echo /warthog1/gps_speed
```

## Design Notes

- Uses `SensorDataQoS` (best-effort) on all topics to match u-blox and Microstrain driver defaults
- The rotation gate resets the GPS anchor during a spin so the first post-rotation speed sample diffs against a clean pre-rotation reference
- The override threshold allows speed reporting during high-speed arcs where displacement is clearly real translational motion
- GPS jumps (multipath, atmospheric) are discarded without advancing the anchor

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
