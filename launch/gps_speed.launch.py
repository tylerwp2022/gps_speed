"""
gps_speed.launch.py — Launch the gps_speed_node for one or more robots.

USAGE:
    # Default — standstill gate enabled, uses status_speed if available
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1

    # Disable standstill gate (legacy behaviour)
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \\
        use_status_speed:=false

    # Tighter standstill threshold (very low encoder noise)
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \\
        standstill_threshold_m_s:=0.02

    # Override status_speed topic suffix
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \\
        status_speed_topic_suffix:=platform/speed

    # Looser rotation gate (allow up to ~17 deg/s yaw before suppressing speed)
    ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \\
        max_rotation_rate_rad_s:=0.3

STANDSTILL GATE NOTE:
    When use_status_speed=true (default), the node subscribes to
    /{robot_name}/{status_speed_topic_suffix} and suppresses GPS speed output
    whenever status_speed < standstill_threshold_m_s. The GPS anchor is also
    reset at standstill so the first post-standstill sample doesn't produce
    a spurious speed spike from accumulated GPS drift.

    If no status_speed message arrives within the first GPS cycles, the node
    falls back to normal computation (no standstill gate) and emits a throttled
    warning. Set use_status_speed:=false to silence the warning and lock to
    legacy behaviour permanently.

ARGUMENTS:
    robot_name                    — Robot namespace (required). e.g. warthog1
    use_status_speed              — Enable standstill gate via hardware odometry.
                                    Falls back gracefully if topic is absent.
                                    (default: true)
    status_speed_topic_suffix     — Topic suffix for hardware speed, under
                                    /{robot_name}/. (default: "status_speed")
    standstill_threshold_m_s      — status_speed below this → suppress GPS output
                                    and reset anchor. (default: 0.05 m/s)
    min_time_delta_s              — Min seconds between GPS samples (default: 0.05)
    min_distance_m                — Min displacement (m) before non-zero speed is
                                    reported (default: 0.05)
    max_speed_m_s                 — Sanity cap; samples above this are discarded
                                    as GPS jumps (default: 20.0)
    max_rotation_rate_rad_s       — IMU yaw rate threshold (rad/s). Speed is forced
                                    to 0.0 above this to suppress rotation-in-place
                                    false readings (default: 0.2 rad/s ≈ 11 deg/s)
    rotation_gate_override_distance_m — GPS displacement (m) above which the rotation
                                    gate is bypassed (genuine translation while turning).
                                    (default: 0.3m ≈ 2.7 m/s at 9 Hz GPS)
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


_hw = _petaar26('hardware.yaml')['hardware']


def generate_launch_description():

    # -------------------------------------------------------------------------
    # Declare launch arguments
    # -------------------------------------------------------------------------
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        description="Robot namespace (e.g. warthog1). Sets subscribe/publish topics."
    )

    # use_status_speed — controls whether the standstill gate is active.
    # Default true: node subscribes to status_speed and suppresses GPS output
    # when the platform is stationary. Fallback to no gate if topic is absent.
    use_status_speed_arg = DeclareLaunchArgument(
        "use_status_speed",
        default_value="true",
        description=(
            "When true, subscribe to /{robot_name}/{status_speed_topic_suffix} "
            "and suppress GPS speed output when below standstill_threshold_m_s. "
            "Falls back to no standstill gate if the topic hasn't published yet. "
            "Set false for legacy behaviour (no standstill gate, only IMU rotation gate)."
        )
    )

    # status_speed topic suffix — platform-specific path segment after /{robot_name}/
    status_speed_topic_suffix_arg = DeclareLaunchArgument(
        "status_speed_topic_suffix",
        default_value="status_speed",
        description=(
            "Topic path suffix for hardware speed, under /{robot_name}/. "
            "Only used when use_status_speed=true. "
            "Node subscribes to /{robot_name}/{status_speed_topic_suffix}. "
            "Default: 'status_speed'."
        )
    )

    # standstill_threshold_m_s — the odometry speed below which the robot is
    # considered stationary. 0.05 m/s = 5 cm/s, well below any intentional motion.
    standstill_threshold_arg = DeclareLaunchArgument(
        "standstill_threshold_m_s",
        default_value="0.05",
        description=(
            "status_speed (m/s) below which the robot is considered stationary. "
            "GPS output is forced to 0.0 and the GPS anchor is reset. "
            "Default 0.05 m/s = 5 cm/s. Increase if your platform's encoder "
            "has quantisation noise above this level."
        )
    )

    # GPS topic suffix — selects between GeoFog and u-blox hardware.
    gps_topic_suffix_arg = DeclareLaunchArgument(
        "gps_topic_suffix",
        default_value=_hw['gps_topic_suffix'],
        description=(
            "GPS topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{gps_topic_suffix}. "
            "Options: sensors/geofog/gps/fix (GeoFog, NAI_2) "
            "or sensors/ublox/fix (u-blox, NAI_3/NAI_4)."
        )
    )

    # IMU topic suffix — deployment-specific driver path.
    imu_topic_suffix_arg = DeclareLaunchArgument(
        "imu_topic_suffix",
        default_value=_hw['imu_topic_suffix'],
        description=(
            "IMU topic path suffix within the robot namespace. "
            "Node subscribes to /{robot_name}/{imu_topic_suffix}."
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
        description=(
            "Minimum displacement (m) before reporting non-zero speed. "
            "With use_status_speed=true, this can be set lower (e.g. 0.02m) "
            "since standstill noise is suppressed by the odometry gate. "
            "Without the gate, set above the 99.9th percentile of standstill "
            "GPS noise from gps_noise_characterizer.py."
        )
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
            "Robot is genuinely translating while turning at this displacement. "
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
            "use_status_speed":                  LaunchConfiguration("use_status_speed"),
            "status_speed_topic_suffix":         LaunchConfiguration("status_speed_topic_suffix"),
            "standstill_threshold_m_s":          LaunchConfiguration("standstill_threshold_m_s"),
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
        use_status_speed_arg,
        status_speed_topic_suffix_arg,
        standstill_threshold_arg,
        gps_topic_suffix_arg,
        imu_topic_suffix_arg,
        min_time_delta_arg,
        min_distance_arg,
        max_speed_arg,
        max_rotation_rate_arg,
        rotation_gate_override_arg,
        gps_speed_node,
    ])
