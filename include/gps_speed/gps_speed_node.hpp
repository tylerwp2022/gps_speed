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
// Published as a scalar Float64 to /{robot_name}/sensors/gps_speed.
//
// WHY GPS-DERIVED SPEED:
//   Wheel odometry accumulates drift, IMU integration drifts, but GPS gives
//   absolute position. Differencing two GPS fixes gives a noisy but drift-free
//   speed estimate — useful for gating heading calibration in imu_compass_node.
//
// GATES (applied in order inside gps_callback):
// -----------------------------------------------
//
//   Gate 1 — Invalid fix rejection:
//     STATUS_NO_FIX → publish 0.0, don't advance anchor.
//
//   Gate 2 — Standstill gate (NEW):
//     When use_status_speed=true and status_speed < standstill_threshold_m_s,
//     the platform odometry confirms the robot is not moving. Any GPS
//     displacement at this point is pure noise (multipath, atmospheric
//     scintillation, antenna vibration). We publish 0.0 and reset prev_sample_
//     so the anchor is always "last position while genuinely moving".
//
//     WHY RESET THE ANCHOR HERE:
//       If we hold prev_sample_ across a standstill, the first post-standstill
//       fix diffs against the pre-stop position. The resulting displacement
//       (robot moved + GPS drift during the stop) produces an inflated speed
//       spike on the first moving sample. Resetting the anchor forces a clean
//       re-anchor on the first post-standstill fix (which publishes 0.0) so
//       speed is computed correctly from the second moving fix onward.
//
//     WHY THIS IS BETTER THAN TUNING min_distance_m:
//       min_distance_m is a static threshold that must be set at or above the
//       worst-case GPS noise, which wastes real low-speed motion readings.
//       The standstill gate is dynamic — it only suppresses when the platform
//       actually confirms the robot is stopped, so low-speed motion (e.g.
//       0.1 m/s creep) still produces valid output.
//
//   Gate 3 — Rotation gate:
//     |IMU yaw rate| > max_rotation_rate_rad_s → publish 0.0, don't advance
//     anchor. Suppresses false speed from the GPS antenna tracing an arc
//     when rotating in place. Bypassed if displacement is large enough to
//     indicate genuine translational motion (rotation_gate_override_distance_m).
//
//   Gate 4 — Min distance:
//     displacement < min_distance_m → publish 0.0. Catches residual GPS noise
//     below the standstill threshold (e.g. very slow creep, or first fix after
//     a standstill before the anchor is freshly set).
//
//   Gate 5 — Max speed sanity cap:
//     implied speed > max_speed_m_s → discard sample (don't advance anchor).
//     Catches GPS multipath jumps that survived all other gates.
//
// SUBSCRIBES:
//   /{robot_name}/{gps_topic_suffix}                 [sensor_msgs/msg/NavSatFix]
//       GPS fix. Suffix driven by 'gps_topic_suffix' parameter.
//       Default: "sensors/geofog/gps/fix".
//
//   /{robot_name}/{imu_topic_suffix}                 [sensor_msgs/msg/Imu]
//       EKF-filtered IMU. angular_velocity.z used for rotation gate.
//       Default: "sensors/microstrain/ekf/imu/data".
//
//   /{robot_name}/{status_speed_topic_suffix}         [std_msgs/msg/Float32]
//       Hardware odometry speed in m/s. Subscribed when use_status_speed=true.
//       Used as the standstill gate: if value < standstill_threshold_m_s,
//       output is forced to 0.0 and the GPS anchor is reset.
//       Default suffix: "status_speed".
//
// PUBLISHES:
//   /{robot_name}/sensors/gps_speed  [std_msgs/msg/Float64]
//       Ground speed in m/s. 0.0 when stationary (any gate), first fix,
//       or invalid fix. Non-zero only when all gates pass.
//
// PARAMETERS:
//   robot_name                (string, required)
//       Robot namespace, e.g. "warthog1".
//
//   use_status_speed          (bool, true)
//       When true, subscribe to status_speed and use it as a standstill gate.
//       Falls back to no standstill gate if the topic hasn't published yet.
//       Set false to disable entirely (legacy behaviour, Gates 1/3/4/5 only).
//
//   status_speed_topic_suffix (string, "status_speed")
//       Topic path after /{robot_name}/ for the hardware speed signal.
//       Only used when use_status_speed=true.
//
//   standstill_threshold_m_s  (double, 0.05)
//       status_speed below this value → robot is stationary → publish 0.0
//       and reset GPS anchor. 0.05 m/s = 5 cm/s, well below any intentional
//       motion. Increase if your platform has encoder quantisation noise above
//       this level.
//
//   gps_topic_suffix          (string, "sensors/geofog/gps/fix")
//       GPS topic path after /{robot_name}/.
//
//   imu_topic_suffix          (string, "sensors/microstrain/ekf/imu/data")
//       IMU topic path after /{robot_name}/.
//
//   min_time_delta_s          (double, 0.05)
//       Minimum seconds between GPS samples; guards divide-by-zero.
//
//   min_distance_m            (double, 0.05)
//       Minimum displacement (m) before reporting non-zero speed.
//       With status_speed gate active, this can be set lower (e.g. 0.02m)
//       since standstill noise is suppressed at the source. Without the
//       gate, set this to the 99.9th percentile of standstill GPS noise
//       from gps_noise_characterizer.py.
//
//   max_speed_m_s             (double, 20.0)
//       Sanity cap. Samples implying higher speed are discarded.
//
//   max_rotation_rate_rad_s   (double, 0.2)
//       IMU yaw rate (rad/s) above which speed is forced to 0.0.
//       ~0.2 rad/s ≈ 11°/s.
//
//   rotation_gate_override_distance_m (double, 0.3)
//       GPS displacement above which the rotation gate is bypassed
//       (robot is genuinely translating while turning). Default 0.3m ≈ 2.7 m/s
//       at 9 Hz GPS.
//
// LAUNCH EXAMPLES:
//   # Default — standstill gate enabled, uses status_speed if available
//   ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1
//
//   # Disable standstill gate (legacy behaviour)
//   ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \
//       use_status_speed:=false
//
//   # Tighter standstill threshold (encoder noise is very low)
//   ros2 launch gps_speed gps_speed.launch.py robot_name:=warthog1 \
//       standstill_threshold_m_s:=0.02
//
// HAVERSINE:
//   Standard Haversine formula (spherical Earth, R=6371000 m).
//   Accurate to ~0.5% for distances under a few km.
//
//==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
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

    /// Called on every incoming status_speed message. Caches hardware speed
    /// for the standstill gate. Only subscribed when use_status_speed_=true.
    /// NOTE: status_speed publishes std_msgs/Float32, not Float64.
    void status_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);

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
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        status_speed_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           speed_pub_;

    // --- Parameters ---
    std::string robot_name_;
    bool        use_status_speed_;              ///< Enable standstill gate via status_speed
    bool        use_status_speed_as_primary_;   ///< Bypass GPS pipeline; republish status_speed directly
    double standstill_threshold_m_s_;           ///< status_speed below this → standstill
    double min_time_delta_s_;                   ///< Skip GPS samples closer together than this
    double min_distance_m_;                     ///< Report 0.0 below this displacement
    double max_speed_m_s_;                      ///< Sanity cap — discard GPS jumps above this
    double max_rotation_rate_rad_s_;            ///< Force 0.0 speed when |yaw rate| exceeds this
    double rotation_gate_override_distance_m_;  ///< Override rotation gate if displacement exceeds this

    // --- Hardware odometry speed (standstill gate) ---
    // WHY atomic: written by status_speed_callback, read by gps_callback.
    // With a single-threaded executor they don't truly race, but atomic is
    // cheap and makes the cross-callback access explicit and safe.
    std::atomic<double> latest_status_speed_m_s_{0.0};

    /// Set to true on the first incoming status_speed message.
    /// Until true, the standstill gate is skipped (startup fallback).
    std::atomic<bool> status_speed_received_{false};

    // --- Latest IMU yaw rate (updated by imu_callback) ---
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
