//==============================================================================
// gps_speed_node.cpp — ROS2 Jazzy Node
//==============================================================================
// See include/gps_speed/gps_speed_node.hpp for full documentation.
//==============================================================================

#include "gps_speed/gps_speed_node.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>

static constexpr double EARTH_RADIUS_M = 6'371'000.0;

//==============================================================================
// CONSTRUCTOR
//==============================================================================

GpsSpeedNode::GpsSpeedNode(const rclcpp::NodeOptions & options)
: Node("gps_speed_node", options)
{
    //--------------------------------------------------------------------------
    // Declare + retrieve parameters
    //--------------------------------------------------------------------------
    this->declare_parameter<std::string>("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    if (robot_name_.empty())
    {
        RCLCPP_FATAL(this->get_logger(),
            "Parameter 'robot_name' is required. "
            "Set via: --ros-args -p robot_name:=warthog1");
        throw std::runtime_error("Missing required parameter: robot_name");
    }

    // use_status_speed: when true (default), subscribe to the hardware odometry
    // speed topic and use it as a standstill gate. If status_speed < standstill_
    // threshold_m_s, the GPS computation is skipped entirely and 0.0 is published.
    // Falls back to no standstill gate if the topic hasn't published yet, with a
    // throttled warning so the operator knows.
    this->declare_parameter<bool>("use_status_speed", true);
    use_status_speed_ = this->get_parameter("use_status_speed").as_bool();

    this->declare_parameter<std::string>("status_speed_topic_suffix", "status_speed");
    const std::string status_speed_suffix =
        this->get_parameter("status_speed_topic_suffix").as_string();

    this->declare_parameter<double>("standstill_threshold_m_s", 0.05);
    standstill_threshold_m_s_ = this->get_parameter("standstill_threshold_m_s").as_double();

    this->declare_parameter<double>("min_time_delta_s",                0.05);
    this->declare_parameter<double>("min_distance_m",                  0.05);
    this->declare_parameter<double>("max_speed_m_s",                  20.0);
    this->declare_parameter<double>("max_rotation_rate_rad_s",         0.2);
    this->declare_parameter<double>("rotation_gate_override_distance_m", 0.3);

    // Topic suffix parameters — see header for rationale.
    this->declare_parameter<std::string>("gps_topic_suffix",
        "sensors/geofog/gps/fix");
    this->declare_parameter<std::string>("imu_topic_suffix",
        "sensors/microstrain/ekf/imu/data");

    min_time_delta_s_                  = this->get_parameter("min_time_delta_s").as_double();
    min_distance_m_                    = this->get_parameter("min_distance_m").as_double();
    max_speed_m_s_                     = this->get_parameter("max_speed_m_s").as_double();
    max_rotation_rate_rad_s_           = this->get_parameter("max_rotation_rate_rad_s").as_double();
    rotation_gate_override_distance_m_ = this->get_parameter("rotation_gate_override_distance_m").as_double();
    const std::string gps_topic_suffix = this->get_parameter("gps_topic_suffix").as_string();
    const std::string imu_topic_suffix = this->get_parameter("imu_topic_suffix").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Parameters: robot_name=%s  use_status_speed=%s  "
        "standstill_threshold=%.3fm/s  min_time_delta=%.3fs  "
        "min_distance=%.3fm  max_speed=%.1fm/s  "
        "max_rotation_rate=%.3frad/s (%.1fdeg/s)  "
        "rotation_gate_override=%.3fm",
        robot_name_.c_str(),
        use_status_speed_ ? "true" : "false",
        standstill_threshold_m_s_,
        min_time_delta_s_,
        min_distance_m_,
        max_speed_m_s_,
        max_rotation_rate_rad_s_,
        max_rotation_rate_rad_s_ * 180.0 / M_PI,
        rotation_gate_override_distance_m_);

    //--------------------------------------------------------------------------
    // Build topic names from parameters
    //--------------------------------------------------------------------------
    const std::string gps_topic          = "/" + robot_name_ + "/" + gps_topic_suffix;
    const std::string imu_topic          = "/" + robot_name_ + "/" + imu_topic_suffix;
    const std::string status_speed_topic = "/" + robot_name_ + "/" + status_speed_suffix;
    const std::string speed_topic        = "/" + robot_name_ + "/gps_speed";

    //--------------------------------------------------------------------------
    // Subscribers
    //--------------------------------------------------------------------------

    // GPS: SensorDataQoS (best-effort) to match the u-blox / GeoFog driver.
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->gps_callback(msg);
        });

    // IMU: SensorDataQoS (best-effort) to match the Microstrain driver.
    // Updates at ~100-500 Hz so latest_yaw_rate_rad_s_ is always current
    // by the time a GPS callback fires at ~9 Hz.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            this->imu_callback(msg);
        });

    // status_speed: only subscribed when use_status_speed=true.
    // WHY CONDITIONAL: if false the user has explicitly opted out of the
    // standstill gate. Creating the subscription anyway would add confusion
    // to ros2 topic info output and unnecessary callbacks on platforms that
    // don't publish this topic.
    if (use_status_speed_)
    {
        status_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            status_speed_topic,
            rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                this->status_speed_callback(msg);
            });
    }

    //--------------------------------------------------------------------------
    // Publisher
    //--------------------------------------------------------------------------
    speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        speed_topic,
        rclcpp::SensorDataQoS());

    //--------------------------------------------------------------------------
    // Startup log
    //--------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "Subscribed to GPS:   %s", gps_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU:   %s", imu_topic.c_str());

    if (use_status_speed_)
    {
        RCLCPP_INFO(this->get_logger(),
            "Subscribed to status_speed: %s  "
            "(standstill gate: will suppress GPS noise when < %.3f m/s; "
            "waiting for first message before gate is active)",
            status_speed_topic.c_str(),
            standstill_threshold_m_s_);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
            "use_status_speed=false — standstill gate disabled. "
            "GPS noise at standstill will propagate to output. "
            "Tune min_distance_m to suppress noise manually, or set "
            "use_status_speed:=true if the platform publishes %s.",
            status_speed_topic.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Publishing speed to:  %s", speed_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Waiting for first GPS fix...");
}

//==============================================================================
// STATUS SPEED CALLBACK — cache hardware odometry speed for standstill gate
//==============================================================================

void GpsSpeedNode::status_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_status_speed_m_s_.store(msg->data);

    // Flip the received flag on the first message so gps_callback knows the
    // gate is now active. Log once for operator confirmation.
    if (!status_speed_received_.load())
    {
        status_speed_received_.store(true);
        RCLCPP_INFO(this->get_logger(),
            "status_speed topic is live (first message: %.4f m/s). "
            "Standstill gate is now active — GPS noise below %.3f m/s "
            "will be suppressed.",
            msg->data, standstill_threshold_m_s_);
    }
}

//==============================================================================
// IMU CALLBACK — cache yaw rate for rotation gate
//==============================================================================

void GpsSpeedNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // angular_velocity.z is yaw rate in rad/s (positive = CCW).
    latest_yaw_rate_rad_s_.store(msg->angular_velocity.z);
}

//==============================================================================
// GPS CALLBACK — compute and publish speed
//
// Gates applied in order:
//   1. Invalid fix rejection
//   2. Standstill gate (status_speed, if enabled and received)
//   3. First-fix anchor
//   4. Time delta guard
//   5. Haversine distance
//   6. Rotation gate
//   7. Min distance
//   8. Max speed sanity
//   9. Publish + advance anchor
//==============================================================================

void GpsSpeedNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    //--------------------------------------------------------------------------
    // Gate 1: Reject invalid fixes
    //--------------------------------------------------------------------------
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No GPS fix — publishing 0.0.");

        auto out = std_msgs::msg::Float64();
        out.data = 0.0;
        speed_pub_->publish(out);
        // Don't update prev_sample_ — bad fix shouldn't anchor the next delta.
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 2: Standstill gate
    //
    // If use_status_speed=true AND the status_speed topic has published at
    // least once, check whether the platform odometry confirms standstill.
    //
    // WHY "at least once" before activating:
    //   At node startup the status_speed topic may not have published yet.
    //   We don't want to permanently suppress output because of a cold-start
    //   race. Once the first message arrives (status_speed_received_ = true),
    //   the gate is active for the lifetime of the node.
    //
    // WHY RESET prev_sample_ HERE:
    //   If we hold the anchor across a standstill, the first post-standstill
    //   fix diffs against the pre-stop position, producing a spurious speed
    //   spike. Resetting the anchor forces a clean re-anchor on the first
    //   post-standstill fix (which publishes 0.0 via the first-fix path),
    //   so speed is computed correctly from the second moving fix onward.
    //
    // WHY NOT JUST SUPPRESS OUTPUT WITHOUT RESETTING:
    //   Suppressing without resetting is tempting but wrong — the GPS antenna
    //   can drift by 0.5–2.0 m during a long standstill. Keeping the stale
    //   anchor means the first moving sample measures that drift as speed.
    //--------------------------------------------------------------------------
    if (use_status_speed_)
    {
        if (!status_speed_received_.load())
        {
            // Topic not yet live — warn periodically and fall through to
            // normal computation. This is the startup fallback window.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "use_status_speed=true but no status_speed message received yet — "
                "standstill gate inactive. Check that %s is publishing. "
                "This warning disappears once the first message arrives.",
                ("/" + robot_name_ + "/" +
                 this->get_parameter("status_speed_topic_suffix").as_string()).c_str());
        }
        else if (latest_status_speed_m_s_.load() < standstill_threshold_m_s_)
        {
            // STATE: STANDSTILL — robot is not moving.
            // Suppress GPS noise and invalidate the anchor.
            if (prev_sample_.has_value())
            {
                RCLCPP_DEBUG(this->get_logger(),
                    "Standstill gate: status_speed=%.4f m/s < %.4f m/s threshold — "
                    "resetting GPS anchor and publishing 0.0.",
                    latest_status_speed_m_s_.load(), standstill_threshold_m_s_);
                prev_sample_.reset();
            }

            auto out = std_msgs::msg::Float64();
            out.data = 0.0;
            speed_pub_->publish(out);
            return;
        }
        // else: status_speed >= threshold → robot is moving, fall through.
    }

    //--------------------------------------------------------------------------
    // Gate 3: First fix — nothing to diff against yet
    //--------------------------------------------------------------------------
    const rclcpp::Time current_stamp = msg->header.stamp;

    if (!prev_sample_.has_value())
    {
        RCLCPP_INFO(this->get_logger(),
            "First GPS fix (lat=%.6f, lon=%.6f). Publishing 0.0; "
            "speed available after second fix.",
            msg->latitude, msg->longitude);

        prev_sample_ = GpsSample{msg->latitude, msg->longitude, current_stamp};

        auto out = std_msgs::msg::Float64();
        out.data = 0.0;
        speed_pub_->publish(out);
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 4: Time delta guard
    //--------------------------------------------------------------------------
    const double dt_s = (current_stamp - prev_sample_->stamp).seconds();

    if (dt_s < min_time_delta_s_)
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Sample too close in time (dt=%.4fs < %.4fs), skipping.",
            dt_s, min_time_delta_s_);
        return;
    }

    //--------------------------------------------------------------------------
    // Gate 5 (setup): Haversine distance
    // Computed before the rotation gate so the displacement can override it.
    //--------------------------------------------------------------------------
    const double distance_m = haversine_distance_m(
        prev_sample_->lat_deg, prev_sample_->lon_deg,
        msg->latitude,         msg->longitude);

    //--------------------------------------------------------------------------
    // Gate 5: Rotation gate
    //
    // Suppress false speed caused by the GPS antenna tracing an arc when
    // the robot spins in place. Bypassed if displacement is large enough to
    // indicate the robot is genuinely translating while turning.
    //
    // Note: the standstill gate (Gate 2) already handles the zero-speed case.
    // The rotation gate handles the separate case where status_speed > 0 but
    // the motion is rotational — e.g. pivoting on one track — where GPS
    // displacement is non-zero but represents antenna arc, not translation.
    //--------------------------------------------------------------------------
    const double yaw_rate = std::abs(latest_yaw_rate_rad_s_.load());

    if (yaw_rate > max_rotation_rate_rad_s_)
    {
        if (distance_m >= rotation_gate_override_distance_m_)
        {
            // Large displacement — genuine translation while turning.
            RCLCPP_DEBUG(this->get_logger(),
                "Rotation gate OVERRIDDEN: |yaw_rate|=%.3f rad/s but "
                "distance=%.4fm >= override=%.4fm — reporting speed.",
                yaw_rate, distance_m, rotation_gate_override_distance_m_);
        }
        else
        {
            // Small displacement — rotation-in-place noise. Suppress.
            RCLCPP_DEBUG(this->get_logger(),
                "Rotation gate: |yaw_rate|=%.3f rad/s (%.1f deg/s), "
                "distance=%.4fm < override=%.4fm — publishing 0.0.",
                yaw_rate, yaw_rate * 180.0 / M_PI,
                distance_m, rotation_gate_override_distance_m_);

            auto out = std_msgs::msg::Float64();
            out.data = 0.0;
            speed_pub_->publish(out);
            // Don't advance anchor — keep the pre-rotation reference intact.
            return;
        }
    }

    //--------------------------------------------------------------------------
    // Gate 6: Min distance — robot is still or fix noise is below threshold
    //--------------------------------------------------------------------------
    double speed_m_s = 0.0;

    if (distance_m >= min_distance_m_)
    {
        speed_m_s = distance_m / dt_s;

        // Gate 7: Max speed sanity — discard GPS jumps (multipath, atmospheric).
        // Don't advance anchor so the next good fix diffs against a trustworthy
        // reference position.
        if (speed_m_s > max_speed_m_s_)
        {
            RCLCPP_WARN(this->get_logger(),
                "Implausible speed %.2f m/s (distance=%.3fm, dt=%.3fs) — "
                "likely GPS jump. Discarding sample.",
                speed_m_s, distance_m, dt_s);
            return;
        }
    }

    //--------------------------------------------------------------------------
    // Publish and advance sliding window
    //--------------------------------------------------------------------------
    auto out = std_msgs::msg::Float64();
    out.data = speed_m_s;
    speed_pub_->publish(out);

    RCLCPP_DEBUG(this->get_logger(),
        "Speed: %.3f m/s  (distance=%.4fm, dt=%.4fs, "
        "yaw_rate=%.3f rad/s, status_speed=%.3f m/s)",
        speed_m_s, distance_m, dt_s,
        yaw_rate,
        use_status_speed_ ? latest_status_speed_m_s_.load() : -1.0);

    prev_sample_ = GpsSample{msg->latitude, msg->longitude, current_stamp};
}

//==============================================================================
// HAVERSINE DISTANCE
//==============================================================================

double GpsSpeedNode::haversine_distance_m(
    double lat1_deg, double lon1_deg,
    double lat2_deg, double lon2_deg)
{
    const double lat1 = deg2rad(lat1_deg);
    const double lat2 = deg2rad(lat2_deg);
    const double dlat = deg2rad(lat2_deg - lat1_deg);
    const double dlon = deg2rad(lon2_deg - lon1_deg);

    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0)
                   + std::cos(lat1) * std::cos(lat2)
                   * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return EARTH_RADIUS_M * c;
}

//==============================================================================
// MAIN
//==============================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<GpsSpeedNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("gps_speed_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
