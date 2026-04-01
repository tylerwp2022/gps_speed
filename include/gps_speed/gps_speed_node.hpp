#ifndef GPS_SPEED__GPS_SPEED_NODE_HPP
#define GPS_SPEED__GPS_SPEED_NODE_HPP

//==============================================================================
// gps_speed_node.hpp — ROS2 Jazzy Node
//==============================================================================
//
// PURPOSE:
// --------
// Estimates the robot's ground speed (m/s) by computing the Haversine distance
// between successive GPS fixes and dividing by the elapsed time between them.
//
// Published as a scalar Float64 to /{robot_name}/gps_speed.
//
// WHY GPS-DERIVED SPEED:
//   Wheel odometry accumulates drift, IMU integration drifts, but GPS gives
//   absolute position. Differencing two GPS fixes gives a noisy but drift-free
//   speed estimate — exactly what's needed to gate heading calibration
//   (only calibrate IMU compass when the robot is moving in a straight-ish
//   line, not spinning in place).
//
// ROTATION GATE (the key feature):
//   When the robot rotates in place, the GPS antenna traces a small circle
//   (antenna offset from center of rotation) plus normal GPS jitter, making
//   it look like the robot is translating. This produces false speed readings.
//
//   Fix: subscribe to the IMU and cache angular_velocity.z. If |yaw rate|
//   exceeds max_rotation_rate_rad_s, publish 0.0 and skip the GPS calculation.
//   Pure rotation is suppressed; linear motion (which has low yaw rate in
//   steady-state driving) passes through normally.
//
// SUBSCRIBES:
//   /{robot_name}/sensors/ublox/fix                  [sensor_msgs/msg/NavSatFix]
//       Standard u-blox GPS fix. Must have status >= STATUS_FIX.
//
//   /{robot_name}/sensors/microstrain/ekf/imu/data   [sensor_msgs/msg/Imu]
//       EKF-filtered IMU. angular_velocity.z used to detect rotation-in-place.
//
// PUBLISHES:
//   /{robot_name}/gps_speed  [std_msgs/msg/Float64]
//       Ground speed in m/s.
//       Publishes 0.0 on first fix, invalid fix, or when rotating in place.
//       Only publishes non-zero when displacement >= min_distance_m AND
//       |angular_velocity.z| <= max_rotation_rate_rad_s.
//
// PARAMETERS:
//   robot_name                (string, required) — Robot namespace, e.g. "warthog1"
//   min_time_delta_s          (double, 0.05)     — Minimum seconds between GPS
//                                                  samples; guards divide-by-zero
//   min_distance_m            (double, 0.05)     — Minimum displacement in meters
//                                                  before reporting non-zero speed
//   max_speed_m_s             (double, 20.0)     — Sanity cap; samples implying
//                                                  higher speed are discarded as
//                                                  GPS jumps
//   max_rotation_rate_rad_s   (double, 0.2)      — IMU yaw rate (rad/s) above which
//                                                  the robot is considered to be
//                                                  rotating in place. Speed output
//                                                  is forced to 0.0 above this.
//                                                  ~0.2 rad/s ≈ 11°/s — well below
//                                                  a deliberate spin but above steady
//                                                  straight-line yaw corrections.
//   rotation_gate_override_distance_m (double, 0.3) — If the GPS displacement between
//                                                  fixes exceeds this, the rotation gate
//                                                  is bypassed and speed is reported
//                                                  regardless of yaw rate. Handles the
//                                                  case where the robot is genuinely
//                                                  translating while turning (e.g. 3 m/s
//                                                  arc). Default 0.3m ≈ 2.7 m/s at 9 Hz.
//
// HAVERSINE:
//   Uses the standard Haversine formula (spherical Earth, R=6371000 m).
//   Accurate to ~0.5% for distances under a few km.
//
// LAUNCH EXAMPLE:
//   ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1
//
//   # Override rotation gate threshold:
//   ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1
//       max_rotation_rate_rad_s:=0.3
//
// TOPIC EXAMPLE:
//   $ ros2 topic echo /warthog1/gps_speed
//   data: 1.42
//   ---
//
//==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include <atomic>
#include <cmath>
#include <optional>
#include <string>

class GpsSpeedNode : public rclcpp::Node
{
public:
    explicit GpsSpeedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    //==========================================================================
    // CALLBACKS
    //==========================================================================

    /// Called on every incoming GPS fix. Computes and publishes speed.
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /// Called on every incoming IMU message. Caches yaw rate for rotation gate.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    //==========================================================================
    // MATH HELPERS
    //==========================================================================

    /// Haversine distance in meters between two WGS-84 lat/lon points.
    static double haversine_distance_m(double lat1_deg, double lon1_deg,
                                       double lat2_deg, double lon2_deg);

    /// Degrees → radians.
    static constexpr double deg2rad(double d) { return d * M_PI / 180.0; }

    //==========================================================================
    // INTERNAL STATE
    //==========================================================================

    // --- ROS interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           speed_pub_;

    // --- Parameters ---
    std::string robot_name_;
    double min_time_delta_s_;                   ///< Skip GPS samples closer together than this
    double min_distance_m_;                     ///< Report 0.0 below this displacement
    double max_speed_m_s_;                      ///< Sanity cap — discard GPS jumps above this
    double max_rotation_rate_rad_s_;            ///< Force 0.0 speed when |yaw rate| exceeds this
    double rotation_gate_override_distance_m_;  ///< Override rotation gate if displacement exceeds this

    // --- Latest IMU yaw rate (updated by imu_callback) ---
    // WHY atomic: gps_callback and imu_callback run on the same single-threaded
    // executor by default, but a multi-threaded executor would interleave them.
    // atomic<double> is a cheap, safe way to share a single scalar without a mutex.
    std::atomic<double> latest_yaw_rate_rad_s_{0.0};

    // --- Previous GPS sample (empty until first valid fix arrives) ---
    struct GpsSample {
        double lat_deg;
        double lon_deg;
        rclcpp::Time stamp;
    };
    std::optional<GpsSample> prev_sample_;
};

#endif  // GPS_SPEED__GPS_SPEED_NODE_HPP
