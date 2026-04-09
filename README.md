# gps_speed

A ROS2 Jazzy node that estimates robot ground speed (m/s) from successive GPS fixes using the Haversine formula, with an IMU-based rotation gate to suppress false speed readings when the robot is spinning in place.

## PETAAR26 Integration

This node is part of the PETAAR stack and its sensor topic defaults are driven by `petaar26/config/hardware.yaml`. You do not need to pass topic names manually — the launch file reads them from the shared config.

**To change the GPS or IMU topic** (e.g., when deploying on new hardware), edit:
```
petaar26/config/hardware.yaml
  → hardware.gps_topic_suffix
  → hardware.imu_topic_suffix
```

Then rebuild `petaar26` and relaunch. No changes to this package's source or launch files are needed.

> **Bug fix (applied):** The GPS and IMU subscriber topics were previously hardcoded in the C++ constructor, making them unreachable by launch file parameters. They are now read from ROS parameters at startup. The startup log confirms which topics are actually subscribed to.

---

## Overview

Ground speed is computed by differencing successive `NavSatFix` messages using the Haversine formula. A rotation gate (using IMU angular velocity) suppresses false speed readings caused by the GPS antenna tracing a small arc when the robot spins in place. If GPS displacement is large enough to indicate genuine translational motion, the gate is bypassed.

---

## Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribes | `/{robot_name}/{gps_topic_suffix}` | `sensor_msgs/NavSatFix` | GPS fix. Suffix from `hardware.yaml` (e.g. `sensors/geofog/gps/fix` or `sensors/ublox/fix`) |
| Subscribes | `/{robot_name}/{imu_topic_suffix}` | `sensor_msgs/Imu` | IMU angular velocity for rotation gate. Suffix from `hardware.yaml` |
| Publishes | `/{robot_name}/sensors/gps_speed` | `std_msgs/Float64` | Ground speed in m/s |

Verify at startup — the node logs the actual resolved topics:
```
[INFO] Subscribed to GPS:   /warthog1/sensors/geofog/gps/fix
[INFO] Subscribed to IMU:   /warthog1/sensors/microstrain/ekf/imu/data
[INFO] Publishing speed to: /warthog1/sensors/gps_speed
```
If these don't match your hardware, update `hardware.yaml`.

---

## Parameters

| Parameter | Type | Default | Source | Description |
|---|---|---|---|---|
| `robot_name` | string | **required** | launch arg | Robot namespace, e.g. `warthog1` |
| `gps_topic_suffix` | string | from `hardware.yaml` | `hardware.gps_topic_suffix` | GPS topic path after `/{robot_name}/` |
| `imu_topic_suffix` | string | from `hardware.yaml` | `hardware.imu_topic_suffix` | IMU topic path after `/{robot_name}/` |
| `min_time_delta_s` | double | `0.05` | launch arg | Min seconds between GPS samples |
| `min_distance_m` | double | `0.05` | launch arg | Min displacement (m) before reporting non-zero speed |
| `max_speed_m_s` | double | `20.0` | launch arg | Sanity cap — discard GPS jumps above this |
| `max_rotation_rate_rad_s` | double | `0.2` | launch arg | IMU yaw rate (rad/s) above which speed is forced to 0.0 (~11°/s) |
| `rotation_gate_override_distance_m` | double | `0.3` | launch arg | GPS displacement (m) above which the rotation gate is bypassed |

---

## Launch

```bash
# Default parameters (topics from hardware.yaml)
ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1

# Override rotation gate for high-speed turns
ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \
    max_rotation_rate_rad_s:=0.3

# Override GPS topic directly (overrides hardware.yaml default)
ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \
    gps_topic_suffix:=sensors/ublox/fix

# Verify output
ros2 topic echo /warthog1/sensors/gps_speed
```

---

## Design Notes

- Uses `SensorDataQoS` (best-effort) on subscriptions to match typical GPS and IMU driver QoS
- Speed output uses `SensorDataQoS` so downstream nodes (e.g., `imu_compass_node`) must also use best-effort or sensor QoS to receive it
- The rotation gate resets the GPS anchor during a spin so the first post-rotation fix diffs against a clean pre-rotation reference — prevents an inflated "snap back" speed reading
- GPS jumps (multipath, atmospheric) above `max_speed_m_s` are discarded without advancing the anchor, so the next sample still diffs against a trustworthy position

---

## Dependencies

- `rclcpp`, `sensor_msgs`, `std_msgs`
- `petaar26` package (for shared config defaults at launch time)

---

## Package Structure

```
gps_speed/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/gps_speed/
│   └── gps_speed_node.hpp
├── launch/
│   └── gps_speed.launch.py
└── src/
    └── gps_speed_node.cpp
```
