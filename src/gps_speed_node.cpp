//==============================================================================
// gps_speed_node.cpp — ROS2 Jazzy Node
//==============================================================================
// See include/gps_speed/gps_speed_node.hpp for full documentation.
//==============================================================================

#include "gps_speed/gps_speed_node.hpp"

#include <sensor_msgs/msg/nav_sat_status.hpp>

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

    this->declare_parameter<double>("min_time_delta_s",                0.05);
    this->declare_parameter<double>("min_distance_m",                  0.05);
    this->declare_parameter<double>("max_speed_m_s",                  20.0);
    this->declare_parameter<double>("max_rotation_rate_rad_s",         0.2);
    this->declare_parameter<double>("rotation_gate_override_distance_m", 0.3);

    min_time_delta_s_                  = this->get_parameter("min_time_delta_s").as_double();
    min_distance_m_                    = this->get_parameter("min_distance_m").as_double();
    max_speed_m_s_                     = this->get_parameter("max_speed_m_s").as_double();
    max_rotation_rate_rad_s_           = this->get_parameter("max_rotation_rate_rad_s").as_double();
    rotation_gate_override_distance_m_ = this->get_parameter("rotation_gate_override_distance_m").as_double();

    RCLCPP_INFO(this->get_logger(),
        "Parameters: robot_name=%s  min_time_delta=%.3fs  min_distance=%.3fm  "
        "max_speed=%.1fm/s  max_rotation_rate=%.3frad/s (%.1f deg/s)  "
        "rotation_gate_override=%.3fm",
        robot_name_.c_str(),
        min_time_delta_s_,
        min_distance_m_,
        max_speed_m_s_,
        max_rotation_rate_rad_s_,
        max_rotation_rate_rad_s_ * 180.0 / M_PI,
        rotation_gate_override_distance_m_);

    //--------------------------------------------------------------------------
    // Build topic names
    //--------------------------------------------------------------------------
    const std::string gps_topic   = "/" + robot_name_ + "/sensors/ublox/fix";
    const std::string imu_topic   = "/" + robot_name_ + "/sensors/microstrain/ekf/imu/data";
    const std::string speed_topic = "/" + robot_name_ + "/gps_speed";

    //--------------------------------------------------------------------------
    // Subscribers
    //--------------------------------------------------------------------------

    // GPS: SensorDataQoS (best-effort) to match the u-blox driver.
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->gps_callback(msg);
        });

    // IMU: SensorDataQoS (best-effort) to match the Microstrain driver.
    // The IMU updates much faster than GPS (~100-500 Hz vs ~9 Hz), so by the
    // time a GPS callback fires, latest_yaw_rate_rad_s_ is always current.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            this->imu_callback(msg);
        });

    //--------------------------------------------------------------------------
    // Publisher
    //--------------------------------------------------------------------------
    speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        speed_topic,
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Subscribed to GPS:   %s", gps_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU:   %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing speed to: %s", speed_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Waiting for first GPS fix...");
}

//==============================================================================
// IMU CALLBACK — just cache the yaw rate
//==============================================================================

void GpsSpeedNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // angular_velocity.z is the yaw rate in rad/s (positive = counter-clockwise).
    // Store atomically so gps_callback can read it safely.
    latest_yaw_rate_rad_s_.store(msg->angular_velocity.z);
}

//==============================================================================
// GPS CALLBACK — compute and publish speed
//==============================================================================

void GpsSpeedNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    //--------------------------------------------------------------------------
    // Reject invalid fixes
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
    // First fix: nothing to diff against yet
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
    // Time delta guard
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
    // Haversine distance — computed before the rotation gate so we can use
    // the displacement to decide whether to override it.
    //--------------------------------------------------------------------------
    const double distance_m = haversine_distance_m(
        prev_sample_->lat_deg, prev_sample_->lon_deg,
        msg->latitude,         msg->longitude);

    //--------------------------------------------------------------------------
    // Rotation gate — suppress false speed caused by spinning in place, BUT
    // override if the displacement is large enough to be real translational
    // motion rather than antenna-circle + GPS jitter.
    //
    // Example: robot moving at 3 m/s while turning — displacement per fix
    // is ~0.33m which exceeds the default override threshold of 0.3m, so the
    // speed is reported even though yaw rate is high.
    //
    // Example: robot spinning in place — displacement per fix is <0.1m
    // (antenna offset noise), well below the override threshold, so 0.0 is
    // published and prev_sample_ is NOT advanced.
    //--------------------------------------------------------------------------
    const double yaw_rate = std::abs(latest_yaw_rate_rad_s_.load());

    if (yaw_rate > max_rotation_rate_rad_s_)
    {
        if (distance_m >= rotation_gate_override_distance_m_)
        {
            // Large displacement — robot is genuinely translating even while
            // turning. Let the speed through and log it.
            RCLCPP_DEBUG(this->get_logger(),
                "Rotation gate OVERRIDDEN: |yaw_rate|=%.3f rad/s but "
                "distance=%.4fm >= override threshold=%.4fm — reporting speed.",
                yaw_rate, distance_m, rotation_gate_override_distance_m_);
        }
        else
        {
            // Small displacement — this is rotation-in-place noise. Suppress.
            RCLCPP_DEBUG(this->get_logger(),
                "Rotation gate: |yaw_rate|=%.3f rad/s (%.1f deg/s), "
                "distance=%.4fm < override=%.4fm — publishing 0.0.",
                yaw_rate, yaw_rate * 180.0 / M_PI,
                distance_m, rotation_gate_override_distance_m_);

            auto out = std_msgs::msg::Float64();
            out.data = 0.0;
            speed_pub_->publish(out);
            // Don't advance prev_sample_ — keep the pre-rotation anchor intact
            // so the next post-rotation fix diffs against a clean reference.
            return;
        }
    }

    //--------------------------------------------------------------------------
    // Below minimum distance → robot is still (or fix noise)
    //--------------------------------------------------------------------------
    double speed_m_s = 0.0;

    if (distance_m >= min_distance_m_)
    {
        speed_m_s = distance_m / dt_s;

        // Sanity check: discard GPS jumps. Don't advance prev_sample_ so the
        // next good fix diffs against a trustworthy anchor.
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
        "Speed: %.3f m/s  (distance=%.4fm, dt=%.4fs, yaw_rate=%.3f rad/s)",
        speed_m_s, distance_m, dt_s, yaw_rate);

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
