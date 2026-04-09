"""
gps_speed.launch.py — Launch the gps_speed_node for one or more robots.

USAGE:
    # Single robot (default parameters)
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1

    # Looser rotation gate (allow up to ~17 deg/s yaw before suppressing speed)
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \
        max_rotation_rate_rad_s:=0.3

ARGUMENTS:
    robot_name                — Robot namespace (required). e.g. warthog1
    min_time_delta_s          — Min seconds between GPS samples (default: 0.05)
    min_distance_m            — Min displacement (m) before non-zero speed is
                                reported (default: 0.05)
    max_speed_m_s             — Sanity cap; samples implying higher speeds are
                                discarded as GPS jumps (default: 20.0)
    max_rotation_rate_rad_s   — IMU yaw rate threshold (rad/s). Speed is forced
                                to 0.0 above this to suppress rotation-in-place
                                false readings (default: 0.2 rad/s ≈ 11 deg/s)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def _petaar26(filename: str) -> dict:
    """Load a petaar26 config YAML from the installed share directory."""
    path = os.path.join(
        get_package_share_directory('petaar26'),
        'config', filename
    )
    with open(path) as f:
        return yaml.safe_load(f)


_topics = _petaar26('topics.yaml')['topics']
_hw     = _petaar26('hardware.yaml')['hardware']


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Declare launch arguments
    # -------------------------------------------------------------------------
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        description="Robot namespace (e.g. warthog1). Sets subscribe/publish topics."
    )

    # GPS topic suffix — selects between GeoFog and u-blox hardware.
    # The node subscribes to /{robot_name}/{gps_topic_suffix}.
    # Default matches the historical hardcoded value so existing configs work.
    # sim_control.py passes the active profile's gps_topic_suffix here.
    gps_topic_suffix_arg = DeclareLaunchArgument(
        "gps_topic_suffix",
        default_value=_hw['gps_topic_suffix'],
        description=(
            "GPS topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{gps_topic_suffix}. "
            "Options: sensors/geofog/gps/fix (GeoFog, NAI_2) "
            "or sensors/ublox/fix (u-blox, NAI_3/NAI_4). "
            "Driven by gps_topic_suffix in the active profile (profiles.json)."
        )
    )

    # IMU topic suffix — deployment-specific driver path.
    # Default matches the historical hardcoded value so existing configs work.
    imu_topic_suffix_arg = DeclareLaunchArgument(
        "imu_topic_suffix",
        default_value=_hw['imu_topic_suffix'],
        description=(
            "IMU topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{imu_topic_suffix}. "
            "Default from config/topics.yaml → topics.imu_raw."
        )
    )

    min_time_delta_arg = DeclareLaunchArgument(
        "min_time_delta_s",
        default_value="0.05",
        description="Minimum seconds between GPS samples to compute speed."
    )

    min_distance_arg = DeclareLaunchArgument(
        "min_distance_m",
        default_value="0.05",
        description="Minimum displacement (m) before reporting non-zero speed."
    )

    max_speed_arg = DeclareLaunchArgument(
        "max_speed_m_s",
        default_value="20.0",
        description="Sanity cap (m/s). Samples above this are discarded as GPS jumps."
    )

    max_rotation_rate_arg = DeclareLaunchArgument(
        "max_rotation_rate_rad_s",
        default_value="0.2",
        description=(
            "IMU yaw rate (rad/s) above which speed is forced to 0.0. "
            "Suppresses false speed readings during rotation-in-place. "
            "Default 0.2 rad/s ≈ 11 deg/s."
        )
    )

    rotation_gate_override_arg = DeclareLaunchArgument(
        "rotation_gate_override_distance_m",
        default_value="0.3",
        description=(
            "GPS displacement (m) above which the rotation gate is bypassed. "
            "If the robot is genuinely translating while turning (large displacement), "
            "the speed is reported even if yaw rate exceeds max_rotation_rate_rad_s. "
            "Default 0.3m ≈ 2.7 m/s at 9 Hz GPS."
        )
    )

    # -------------------------------------------------------------------------
    # Node
    # -------------------------------------------------------------------------
    gps_speed_node = Node(
        package="gps_speed",
        executable="gps_speed_node",
        name="gps_speed_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "robot_name":                        LaunchConfiguration("robot_name"),
            "gps_topic_suffix":                  LaunchConfiguration("gps_topic_suffix"),
            "imu_topic_suffix":                  LaunchConfiguration("imu_topic_suffix"),
            "min_time_delta_s":                  LaunchConfiguration("min_time_delta_s"),
            "min_distance_m":                    LaunchConfiguration("min_distance_m"),
            "max_speed_m_s":                     LaunchConfiguration("max_speed_m_s"),
            "max_rotation_rate_rad_s":           LaunchConfiguration("max_rotation_rate_rad_s"),
            "rotation_gate_override_distance_m": LaunchConfiguration("rotation_gate_override_distance_m"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_name_arg,
        gps_topic_suffix_arg,
        imu_topic_suffix_arg,
        min_time_delta_arg,
        min_distance_arg,
        max_speed_arg,
        max_rotation_rate_arg,
        rotation_gate_override_arg,
        gps_speed_node,
    ])
