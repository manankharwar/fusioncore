#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/gnss.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <compass_msgs/msg/azimuth.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <optional>
#include <proj.h>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FusionNode()
  : rclcpp_lifecycle::LifecycleNode("fusioncore")
  {
    RCLCPP_INFO(get_logger(), "FusionCore node created");
    // Two separate mutually-exclusive groups so the publish timer never blocks
    // waiting for a sensor callback (and vice-versa). MultiThreadedExecutor
    // assigns each group its own thread, giving the publish timer its own lane.
    sensor_cb_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  // ─── Lifecycle: Configure ──────────────────────────────────────────────────

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring FusionCore...");

    declare_parameter("base_frame",   "base_link");
    declare_parameter("odom_frame",   "odom");
    declare_parameter("publish_rate", 100.0);

    declare_parameter("imu.gyro_noise",  0.005);
    // Set to true if IMU has a magnetometer (9-axis: BNO08x, VectorNav, Xsens)
    // Set to false for 6-axis IMUs: yaw from gyro integration drifts
    declare_parameter("imu.has_magnetometer", false);
    declare_parameter("imu.accel_noise", 0.1);

    declare_parameter("encoder.vel_noise", 0.05);
    declare_parameter("encoder.yaw_noise", 0.02);

    declare_parameter("gnss.base_noise_xy",  1.0);
    declare_parameter("gnss.base_noise_z",   2.0);
    declare_parameter("gnss.heading_noise",  0.02);
    declare_parameter("gnss.max_hdop",       4.0);
    declare_parameter("gnss.min_satellites", 4);
    // Minimum fix type for GNSS fusion: 1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED
    // Note: NavSatFix status only goes up to 2 (GBAS) which maps to RTK_FIXED.
    // RTK_FLOAT (3) is unreachable via NavSatFix alone.
    declare_parameter("gnss.min_fix_type",  1);

    // Topic for dual antenna heading: sensor_msgs/Imu used as heading carrier.
    // The yaw component of orientation is the heading.
    // Set to empty string to disable dual antenna heading.
    declare_parameter("gnss.heading_topic", "/gnss/heading");

    // Optional second GNSS receiver topic: set to empty string to disable
    declare_parameter("gnss.fix2_topic", "");

    // compass_msgs/Azimuth heading topic: peci1 standard
    // Set to empty string to disable (use sensor_msgs/Imu heading instead)
    declare_parameter("gnss.azimuth_topic", "");

    // Antenna lever arm params: primary receiver
    declare_parameter("gnss.lever_arm_x", 0.0);
    declare_parameter("gnss.lever_arm_y", 0.0);
    declare_parameter("gnss.lever_arm_z", 0.0);

    // Antenna lever arm params: secondary receiver (gnss.fix2_topic)
    // Leave at 0.0 if second antenna is at the same position as the first,
    // or if fix2_topic is not used.
    declare_parameter("gnss.lever_arm2_x", 0.0);
    declare_parameter("gnss.lever_arm2_y", 0.0);
    declare_parameter("gnss.lever_arm2_z", 0.0);

    // PROJ coordinate reference system parameters
    // input.gnss_crs: CRS of incoming NavSatFix messages (default: WGS84 lat/lon)
    // output.crs: intermediate CRS for internal computations (default: ECEF)
    // output.convert_to_enu_at_reference: when true, convert ECEF output to local
    //   ENU frame centered at the GPS reference point (required for ECEF output).
    //   Set false only when output.crs is already a projected local CRS (e.g. UTM).
    // reference.use_first_fix: use first GPS fix as the local origin (default: true)
    // reference.x/y/z: fixed reference in output.crs units (used when use_first_fix=false)
    declare_parameter("input.gnss_crs",                    std::string("EPSG:4326"));
    declare_parameter("output.crs",                        std::string("EPSG:4978"));
    declare_parameter("output.convert_to_enu_at_reference", true);
    declare_parameter("reference.use_first_fix",           true);
    declare_parameter("reference.x",                       0.0);
    declare_parameter("reference.y",                       0.0);
    declare_parameter("reference.z",                       0.0);

    declare_parameter("outlier_rejection",      true);
    declare_parameter("outlier_threshold_gnss", 16.27);
    declare_parameter("outlier_threshold_imu",  15.09);
    declare_parameter("outlier_threshold_enc",  11.34);
    declare_parameter("outlier_threshold_hdg",  10.83);

    declare_parameter("adaptive.imu",     true);
    declare_parameter("adaptive.encoder", true);
    declare_parameter("adaptive.gnss",    true);
    declare_parameter("adaptive.window",  50);
    declare_parameter("adaptive.alpha",   0.01);

    // Zero-velocity update (ZUPT)
    // When encoder velocity and IMU angular rate are both below threshold,
    // the robot is considered stationary and a zero-velocity measurement is fused.
    declare_parameter("zupt.enabled",            true);
    declare_parameter("zupt.velocity_threshold", 0.05);  // m/s
    declare_parameter("zupt.angular_threshold",  0.05);  // rad/s
    declare_parameter("zupt.noise_sigma",        0.01);  // m/s: tight

    declare_parameter("ukf.q_position",     0.01);
    declare_parameter("ukf.q_orientation",  0.01);
    declare_parameter("ukf.q_velocity",     0.1);
    declare_parameter("ukf.q_angular_vel",  0.1);
    declare_parameter("ukf.q_acceleration", 1.0);
    declare_parameter("ukf.q_gyro_bias",    1e-5);
    declare_parameter("ukf.q_accel_bias",   1e-5);

    base_frame_   = get_parameter("base_frame").as_string();
    odom_frame_   = get_parameter("odom_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    heading_topic_ = get_parameter("gnss.heading_topic").as_string();
    gnss2_topic_    = get_parameter("gnss.fix2_topic").as_string();
    azimuth_topic_  = get_parameter("gnss.azimuth_topic").as_string();

    fusioncore::FusionCoreConfig config;

    config.imu.gyro_noise_x  = get_parameter("imu.gyro_noise").as_double();
    config.imu.gyro_noise_y  = config.imu.gyro_noise_x;
    config.imu.gyro_noise_z  = config.imu.gyro_noise_x;
    config.imu.accel_noise_x    = get_parameter("imu.accel_noise").as_double();
    config.imu_has_magnetometer = get_parameter("imu.has_magnetometer").as_bool();
    config.imu.accel_noise_y = config.imu.accel_noise_x;
    config.imu.accel_noise_z = config.imu.accel_noise_x;

    config.encoder.vel_noise_x  = get_parameter("encoder.vel_noise").as_double();
    config.encoder.vel_noise_y  = config.encoder.vel_noise_x;
    config.encoder.vel_noise_wz = get_parameter("encoder.yaw_noise").as_double();

    config.gnss.base_noise_xy  = get_parameter("gnss.base_noise_xy").as_double();
    config.gnss.base_noise_z   = get_parameter("gnss.base_noise_z").as_double();
    config.gnss.heading_noise  = get_parameter("gnss.heading_noise").as_double();
    config.gnss.max_hdop       = get_parameter("gnss.max_hdop").as_double();
    config.gnss.min_satellites = get_parameter("gnss.min_satellites").as_int();
    min_fix_type_ = static_cast<fusioncore::sensors::GnssFixType>(
        get_parameter("gnss.min_fix_type").as_int());
    config.gnss.min_fix_type = min_fix_type_;
    RCLCPP_INFO(get_logger(),
                "GNSS min_fix_type: %d (1=GPS, 2=DGPS, 3=RTK_FLOAT, 4=RTK_FIXED)",
                static_cast<int>(min_fix_type_));
    gnss_lever_arm_.x = get_parameter("gnss.lever_arm_x").as_double();
    gnss_lever_arm_.y = get_parameter("gnss.lever_arm_y").as_double();
    gnss_lever_arm_.z = get_parameter("gnss.lever_arm_z").as_double();

    gnss_lever_arm2_.x = get_parameter("gnss.lever_arm2_x").as_double();
    gnss_lever_arm2_.y = get_parameter("gnss.lever_arm2_y").as_double();
    gnss_lever_arm2_.z = get_parameter("gnss.lever_arm2_z").as_double();

    if (!gnss_lever_arm_.is_zero()) {
      RCLCPP_INFO(get_logger(),
        "GNSS lever arm (primary) set: x=%.3f y=%.3f z=%.3f m",
        gnss_lever_arm_.x, gnss_lever_arm_.y, gnss_lever_arm_.z);
    }
    if (!gnss_lever_arm2_.is_zero()) {
      RCLCPP_INFO(get_logger(),
        "GNSS lever arm (secondary) set: x=%.3f y=%.3f z=%.3f m",
        gnss_lever_arm2_.x, gnss_lever_arm2_.y, gnss_lever_arm2_.z);
    }

    // Wire PROJ parameters
    input_gnss_crs_              = get_parameter("input.gnss_crs").as_string();
    output_crs_                  = get_parameter("output.crs").as_string();
    convert_to_enu_at_reference_ = get_parameter("output.convert_to_enu_at_reference").as_bool();
    reference_use_first_fix_     = get_parameter("reference.use_first_fix").as_bool();

    init_proj();

    if (!reference_use_first_fix_) {
      gnss_ref_ecef_.x = get_parameter("reference.x").as_double();
      gnss_ref_ecef_.y = get_parameter("reference.y").as_double();
      gnss_ref_ecef_.z = get_parameter("reference.z").as_double();
      output_to_gnss(gnss_ref_ecef_, gnss_ref_lla_);
      gnss_ref_set_ = true;
      RCLCPP_INFO(get_logger(),
        "PROJ: fixed reference origin (%.3f, %.3f, %.3f) → lat=%.6f lon=%.6f alt=%.2f",
        gnss_ref_ecef_.x, gnss_ref_ecef_.y, gnss_ref_ecef_.z,
        gnss_ref_lla_.lat_rad * 180.0 / M_PI,
        gnss_ref_lla_.lon_rad * 180.0 / M_PI,
        gnss_ref_lla_.alt_m);
    } else {
      RCLCPP_INFO(get_logger(), "PROJ: using first GPS fix as local reference origin");
    }

    config.outlier_rejection      = get_parameter("outlier_rejection").as_bool();
    config.outlier_threshold_gnss = get_parameter("outlier_threshold_gnss").as_double();
    config.outlier_threshold_imu  = get_parameter("outlier_threshold_imu").as_double();
    config.outlier_threshold_enc  = get_parameter("outlier_threshold_enc").as_double();
    config.outlier_threshold_hdg  = get_parameter("outlier_threshold_hdg").as_double();

    config.adaptive_imu     = get_parameter("adaptive.imu").as_bool();
    config.adaptive_encoder = get_parameter("adaptive.encoder").as_bool();
    config.adaptive_gnss    = get_parameter("adaptive.gnss").as_bool();
    config.adaptive_window  = get_parameter("adaptive.window").as_int();
    config.adaptive_alpha   = get_parameter("adaptive.alpha").as_double();

    config.ukf.q_position   = get_parameter("ukf.q_position").as_double();
    config.ukf.q_orientation  = get_parameter("ukf.q_orientation").as_double();
    config.ukf.q_velocity     = get_parameter("ukf.q_velocity").as_double();
    config.ukf.q_angular_vel  = get_parameter("ukf.q_angular_vel").as_double();
    config.ukf.q_acceleration = get_parameter("ukf.q_acceleration").as_double();
    config.ukf.q_gyro_bias    = get_parameter("ukf.q_gyro_bias").as_double();
    config.ukf.q_accel_bias   = get_parameter("ukf.q_accel_bias").as_double();

    zupt_enabled_            = get_parameter("zupt.enabled").as_bool();
    zupt_velocity_threshold_ = get_parameter("zupt.velocity_threshold").as_double();
    zupt_angular_threshold_  = get_parameter("zupt.angular_threshold").as_double();
    zupt_noise_sigma_        = get_parameter("zupt.noise_sigma").as_double();

    fc_ = std::make_unique<fusioncore::FusionCore>(config);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!heading_topic_.empty()) {
      RCLCPP_INFO(get_logger(),
        "Dual antenna heading enabled on topic: %s", heading_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Dual antenna heading disabled.");
    }

    RCLCPP_INFO(get_logger(),
      "FusionCore configured. base_frame=%s odom_frame=%s rate=%.0fHz",
      base_frame_.c_str(), odom_frame_.c_str(), publish_rate_);

    // TF validation: check transforms exist before starting
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    validate_transforms();

    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Activate ───────────────────────────────────────────────────

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating FusionCore...");

    // Do NOT initialize the filter here with now().seconds().
    // With use_sim_time:true, now() may return 0 if /clock hasn't started yet.
    // Initializing at t=0 then receiving the first IMU at sim t=T causes a
    // T-second dead prediction step that can blow up the state covariance.
    // Instead, initialize lazily on the first IMU message using its timestamp.
    pending_init_ = true;

    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_cb_group_;

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        imu_callback(msg);
      }, sensor_opts);

    encoder_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/wheels", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        encoder_callback(msg);
      }, sensor_opts);

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix", 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        gnss_callback(msg, 0);
      }, sensor_opts);

    // compass_msgs/Azimuth heading: optional, preferred over sensor_msgs/Imu
    if (!azimuth_topic_.empty()) {
      azimuth_sub_ = create_subscription<compass_msgs::msg::Azimuth>(
        azimuth_topic_, 10,
        [this](const compass_msgs::msg::Azimuth::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          azimuth_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "compass_msgs/Azimuth heading enabled on topic: %s", azimuth_topic_.c_str());
    }

    // Second GNSS receiver: optional
    if (!gnss2_topic_.empty()) {
      gnss2_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss2_topic_, 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gnss_callback(msg, 1);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Second GNSS receiver enabled on topic: %s", gnss2_topic_.c_str());
    }

    // Dual antenna heading subscriber: only if topic is configured
    // Expects sensor_msgs/Imu where orientation.z/w gives the yaw heading.
    // This is the standard way dual antenna GPS receivers report heading in ROS.
    if (!heading_topic_.empty()) {
      gnss_heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        heading_topic_, 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gnss_heading_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Subscribed to dual antenna heading: %s", heading_topic_.c_str());
    }

    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("/fusion/odom", 100);
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fusion/pose", 100);
    diag_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); },
      publish_cb_group_);

    // Diagnostics at 1 Hz: standard ROS convention
    diag_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish_diagnostics(); },
      publish_cb_group_);

    // Reset service: re-initializes the filter without restarting the node.
    // Useful after GPS jumps, teleportation in simulation, or catastrophic drift.
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        fusioncore::State initial;
        initial.x = fusioncore::StateVector::Zero();
        initial.P = fusioncore::StateMatrix::Identity() * 0.1;
        initial.P(0,0) = 1000.0;
        initial.P(1,1) = 1000.0;
        initial.P(2,2) = 1000.0;
        fc_->init(initial, last_imu_time_);
        gnss_ref_set_ = false;  // re-anchor GPS reference on next fix
        response->success = true;
        response->message = "FusionCore filter reset. GPS reference cleared.";
        RCLCPP_INFO(get_logger(), "Filter reset via ~/reset service.");
      });

    RCLCPP_INFO(get_logger(), "FusionCore active. Listening for sensors.");
    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Deactivate ─────────────────────────────────────────────────

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    imu_sub_.reset();
    encoder_sub_.reset();
    gnss_sub_.reset();
    gnss2_sub_.reset();
    gnss_heading_sub_.reset();
    azimuth_sub_.reset();
    publish_timer_.reset();
    diag_timer_.reset();
    reset_srv_.reset();
    odom_pub_.reset();
    pose_pub_.reset();
    diag_pub_.reset();
    deinit_proj();
    RCLCPP_INFO(get_logger(), "FusionCore deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    fc_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

private:

  // ─── TF validation ────────────────────────────────────────────────────────
  // Called during on_configure. Checks all required transforms exist.
  // Prints [OK] or [MISSING] + exact fix command for each.
  // Returns true only if all critical transforms are found.

  bool validate_transforms()
  {
    bool all_ok = true;
    RCLCPP_INFO(get_logger(), "--- TF Validation ---");

    // Check common sensor transforms
    std::vector<std::pair<std::string,std::string>> to_check = {
      {"imu_link", base_frame_},
      {base_frame_, odom_frame_},
    };

    for (const auto& [from, to] : to_check) {
      if (check_transform(from, to)) {
        RCLCPP_INFO(get_logger(), "  [OK]      %s -> %s", from.c_str(), to.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "  [MISSING] %s -> %s  Fix: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 %s %s",
          from.c_str(), to.c_str(), to.c_str(), from.c_str());
        all_ok = false;  // Fix 1: was never set: function always returned true
      }
    }

    // Check GNSS frame if primary lever arm is configured
    if (!gnss_lever_arm_.is_zero()) {
      if (check_transform("gnss_link", base_frame_)) {
        RCLCPP_INFO(get_logger(), "  [OK]      gnss_link -> %s", base_frame_.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
          "  [MISSING] gnss_link -> %s  Fix: ros2 run tf2_ros static_transform_publisher %.3f %.3f %.3f 0 0 0 %s gnss_link",
          base_frame_.c_str(),
          gnss_lever_arm_.x, gnss_lever_arm_.y, gnss_lever_arm_.z,
          base_frame_.c_str());
        all_ok = false;
      }
    }

    // Check GNSS2 frame if secondary lever arm is configured
    if (!gnss_lever_arm2_.is_zero()) {
      if (check_transform("gnss2_link", base_frame_)) {
        RCLCPP_INFO(get_logger(), "  [OK]      gnss2_link -> %s", base_frame_.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
          "  [MISSING] gnss2_link -> %s  Fix: ros2 run tf2_ros static_transform_publisher %.3f %.3f %.3f 0 0 0 %s gnss2_link",
          base_frame_.c_str(),
          gnss_lever_arm2_.x, gnss_lever_arm2_.y, gnss_lever_arm2_.z,
          base_frame_.c_str());
        all_ok = false;
      }
    }

    RCLCPP_INFO(get_logger(), "---------------------");
    return all_ok;
  }


  bool check_transform(
    const std::string& from_frame,
    const std::string& to_frame,
    double timeout_seconds = 1.0)
  {
    try {
      tf_buffer_->lookupTransform(
        to_frame, from_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout_seconds));
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  // ─── IMU callback: with frame transform ──────────────────────────────────

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Lazy init: initialize the filter on the first IMU message using the
    // message timestamp. This avoids a large dead prediction step when
    // use_sim_time:true and /clock hasn't started before on_activate().
    last_imu_time_ = t;

    if (pending_init_) {
      fusioncore::State initial;
      initial.x = fusioncore::StateVector::Zero();
      initial.P = fusioncore::StateMatrix::Identity() * 0.1;
      initial.P(0,0) = 1000.0;  // large position uncertainty: accept first GPS
      initial.P(1,1) = 1000.0;
      initial.P(2,2) = 1000.0;
      fc_->init(initial, t);
      pending_init_ = false;
      RCLCPP_INFO(get_logger(), "Filter initialized at t=%.3f (first IMU)", t);
    }

    if (!fc_->is_initialized()) return;

    std::string imu_frame = msg->header.frame_id;
    if (imu_frame.empty()) imu_frame = "imu_link";

    if (imu_frame == base_frame_) {
      fc_->update_imu(t,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z,
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
      // No frame rotation needed: IMU is already in base_frame
      fuse_imu_orientation_if_valid(t, msg, std::nullopt);
      return;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, imu_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot transform IMU from %s to %s: %s"
        " -- Fix: ros2 run tf2_ros static_transform_publisher"
        " 0 0 0 0 0 0 %s %s",
        imu_frame.c_str(), base_frame_.c_str(), ex.what(),
        base_frame_.c_str(), imu_frame.c_str());
      fc_->update_imu(t,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z,
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
      return;
    }

    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);
    tf2::Matrix3x3 R(q);

    tf2::Vector3 w(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
    tf2::Vector3 w_base = R * w;

    tf2::Vector3 a(msg->linear_acceleration.x,
                   msg->linear_acceleration.y,
                   msg->linear_acceleration.z);
    tf2::Vector3 a_base = R * a;

    fc_->update_imu(t,
      w_base.x(), w_base.y(), w_base.z(),
      a_base.x(), a_base.y(), a_base.z());
    // Fix 11: pass the rotation quaternion so orientation is also transformed
    fuse_imu_orientation_if_valid(t, msg, q);
  }

  // ─── IMU orientation helper ───────────────────────────────────────────────
  // Fix 11: now accepts optional imu_to_base rotation to transform orientation
  // into base_frame before fusing. Previously used raw IMU-frame orientation
  // even when IMU was mounted at an angle relative to base_frame.

  void fuse_imu_orientation_if_valid(
    double t,
    const sensor_msgs::msg::Imu::SharedPtr& msg,
    const std::optional<tf2::Quaternion>& imu_to_base)
  {
    // orientation_covariance[0] == -1 means "no orientation data"
    if (msg->orientation_covariance[0] < 0.0) return;

    // All zeros also means unknown: skip
    bool has_orientation = false;
    for (int i = 0; i < 9; ++i) {
      if (msg->orientation_covariance[i] != 0.0) {
        has_orientation = true;
        break;
      }
    }
    if (!has_orientation) return;

    tf2::Quaternion q_imu(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    // Fix 11: rotate orientation from IMU frame to base_frame.
    // q_base = q_imu_to_base * q_imu  (apply mount rotation first)
    tf2::Quaternion q_base = imu_to_base.has_value()
      ? (imu_to_base.value() * q_imu).normalized()
      : q_imu;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_base).getRPY(roll, pitch, yaw);

    fc_->update_imu_orientation(
      t, roll, pitch, yaw,
      msg->orientation_covariance.data());
  }

  // ─── Encoder callback ─────────────────────────────────────────────────────

  void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Extract per-axis variances from the Odometry twist covariance (6x6, row-major).
    // Indices: vx=0, vy=7, wz=35 (diagonal elements for linear.x, linear.y, angular.z).
    // Pass -1.0 for any axis where the message reports zero or negative variance,
    // so update_encoder falls back to adaptive/config noise for that axis.
    const auto& cov = msg->twist.covariance;
    double var_vx = (cov[0]  > 0.0) ? cov[0]  : -1.0;
    double var_vy = (cov[7]  > 0.0) ? cov[7]  : -1.0;
    double var_wz = (cov[35] > 0.0) ? cov[35] : -1.0;

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double wz = msg->twist.twist.angular.z;

    fc_->update_encoder(t, vx, vy, wz, var_vx, var_vy, var_wz);

    // Non-holonomic ground constraint: wheeled robots cannot move vertically.
    // Fuses VZ=0 as a pseudo-measurement to prevent altitude drift.
    fc_->update_ground_constraint(t);

    // Zero-velocity update (ZUPT): when the robot is stationary, assert
    // [VX=0, VY=0, WZ=0] with tight noise to suppress IMU drift.
    // Uses encoder velocity AND current UKF angular rate for detection.
    if (zupt_enabled_) {
      double speed = std::sqrt(vx*vx + vy*vy);
      double yaw_rate = std::abs(fc_->get_state().x[fusioncore::WZ]);
      if (speed < zupt_velocity_threshold_ && yaw_rate < zupt_angular_threshold_) {
        fc_->update_zupt(t, zupt_noise_sigma_);
      }
    }
  }

  // ─── GNSS position callback ────────────────────────────────────────────────

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg, int source_id = 0)
  {
    if (!fc_->is_initialized()) return;

    if (msg->status.status < 0) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    fusioncore::sensors::LLAPoint lla;
    lla.lat_rad = msg->latitude  * M_PI / 180.0;
    lla.lon_rad = msg->longitude * M_PI / 180.0;
    lla.alt_m   = msg->altitude;

    // Convert from input CRS (e.g. EPSG:4326 WGS84) to output CRS (e.g. EPSG:4978 ECEF)
    // using PROJ. Default behavior is identical to the hand-coded WGS84→ECEF math.
    fusioncore::sensors::ECEFPoint ecef;
    gnss_to_output(lla, ecef);

    if (!gnss_ref_set_) {
      gnss_ref_lla_ = lla;
      gnss_ref_ecef_ = ecef;
      gnss_ref_set_ = true;
      RCLCPP_INFO(get_logger(), "GNSS reference set: lat=%.6f lon=%.6f",
        msg->latitude, msg->longitude);
      // Do NOT return: fall through and fuse ENU [0,0,0] as first fix.
    }

    // Pre-filter: drop fixes more than 10km from the reference origin.
    // Handles Gazebo NavSat bug (gz-sim #2163) and catastrophic hardware glitches.
    // Mahalanobis handles normal outliers (1-100m); this handles physically impossible jumps.
    {
      double dx = ecef.x - gnss_ref_ecef_.x;
      double dy = ecef.y - gnss_ref_ecef_.y;
      double dz = ecef.z - gnss_ref_ecef_.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist > 10000.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "GPS fix dropped: %.0fm from reference (Gazebo NavSat bug or hardware glitch)", dist);
        return;
      }
    }

    // Bug 3 fix: only convert ECEF→ENU when output.convert_to_enu_at_reference is true.
    // When output.crs is already a local projected CRS (e.g. UTM), use XY directly.
    Eigen::Vector3d enu;
    if (convert_to_enu_at_reference_) {
      enu = fusioncore::sensors::ecef_to_enu(ecef, gnss_ref_ecef_, gnss_ref_lla_);
    } else {
      enu = Eigen::Vector3d(ecef.x - gnss_ref_ecef_.x,
                            ecef.y - gnss_ref_ecef_.y,
                            ecef.z - gnss_ref_ecef_.z);
    }

    fusioncore::sensors::GnssFix fix;
    fix.x = enu[0];
    fix.y = enu[1];
    fix.z = enu[2];
    // Map NavSatFix status to GnssFixType:
    //   -1 = STATUS_NO_FIX  (already rejected above)
    //    0 = STATUS_FIX      → GPS_FIX
    //    1 = STATUS_SBAS_FIX → DGPS_FIX
    //    2 = STATUS_GBAS_FIX → RTK_FIXED (RTK/GBAS augmented)
    switch (msg->status.status) {
      case 2:  fix.fix_type = fusioncore::sensors::GnssFixType::RTK_FIXED; break;
      case 1:  fix.fix_type = fusioncore::sensors::GnssFixType::DGPS_FIX; break;
      default: fix.fix_type = fusioncore::sensors::GnssFixType::GPS_FIX;  break;
    }
    fix.source_id = source_id;
    fix.lever_arm = (source_id == 0) ? gnss_lever_arm_ : gnss_lever_arm2_;

    // Use message covariance when meaningful (peci1 fix)
    // position_covariance_type:
    //   0 = unknown
    //   1 = approximated (diagonal only)
    //   2 = diagonal known
    //   3 = full matrix known: use off-diagonal elements too
    if (msg->position_covariance_type == 3) {
      // Full 3x3 covariance available: use it directly including off-diagonals
      Eigen::Matrix3d cov;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          cov(i, j) = msg->position_covariance[i*3 + j];

      // Validate diagonal is positive
      if (cov(0,0) > 0.0 && cov(1,1) > 0.0 && cov(2,2) > 0.0) {
        fix.has_full_covariance = true;
        fix.full_covariance = cov;
        fix.hdop = std::sqrt(cov(0,0));  // for validity check
        fix.vdop = std::sqrt(cov(2,2));
        fix.satellites = 4;  // Fix 10: honest minimum: was hardcoded 6, always passed quality gate
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
        fix.satellites = 4;  // Fix 10
      }
    } else if (msg->position_covariance_type >= 1) {
      // Diagonal covariance available
      double var_xy = (msg->position_covariance[0] + msg->position_covariance[4]) / 2.0;
      double var_z  = msg->position_covariance[8];
      if (var_xy > 0.0 && var_z > 0.0) {
        fix.hdop = std::sqrt(var_xy);
        fix.vdop = std::sqrt(var_z);
        fix.satellites = 4;  // Fix 10
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
        fix.satellites = 4;  // Fix 10
      }
    } else {
      // Unknown covariance: use config defaults
      fix.hdop = 1.5;
      fix.vdop = 2.0;
      fix.satellites = 4;  // Fix 10
    }

    bool accepted = fc_->update_gnss(t, fix);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GNSS fix rejected (fix_type=%d, min=%d, hdop=%.2f, "
        "quality check or Mahalanobis gate)",
        static_cast<int>(fix.fix_type),
        static_cast<int>(min_fix_type_),
        fix.hdop);
    }

    // Log heading observability status
    auto fc_status = fc_->get_status();
    if (!fc_status.heading_validated) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Heading not yet validated: lever arm inactive. "
        "Distance traveled: %.1fm (need %.1fm), or provide dual antenna / IMU orientation.",
        fc_status.distance_traveled,
        5.0);
    }
  }

  // ─── Dual antenna heading callback ────────────────────────────────────────
  // Fixes peci1 issue: dual antenna heading was in core C++ but not wired
  // to any ROS topic. Now subscribes to gnss.heading_topic.
  //
  // Expected message: sensor_msgs/Imu
  // The orientation quaternion gives the robot heading in ENU frame.
  // We extract yaw from it and pass to update_gnss_heading().
  //
  // Most dual antenna GPS receivers (u-blox, Septentrio, Trimble) publish
  // heading as a quaternion in a sensor_msgs/Imu message. This is the
  // de facto standard in ROS even though it is slightly awkward.

  void gnss_heading_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Check orientation covariance: if all zeros the orientation is invalid
    bool orientation_valid = false;
    for (int i = 0; i < 9; ++i) {
      if (msg->orientation_covariance[i] != 0.0) {
        orientation_valid = true;
        break;
      }
    }

    // Some drivers set covariance[0] = -1 to signal "no orientation"
    if (msg->orientation_covariance[0] < 0.0) {
      orientation_valid = false;
    }

    if (!orientation_valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Dual antenna heading message has invalid orientation covariance."
        " Check your GPS driver configuration.");
      return;
    }

    // Extract yaw from quaternion
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Extract heading accuracy from orientation covariance
    // covariance[8] is the yaw variance (3rd diagonal element)
    double yaw_variance = msg->orientation_covariance[8];
    double yaw_sigma = (yaw_variance > 0.0) ? std::sqrt(yaw_variance) : 0.02;

    fusioncore::sensors::GnssHeading heading;
    heading.heading_rad  = yaw;
    heading.accuracy_rad = yaw_sigma;
    heading.valid        = true;

    bool accepted = fc_->update_gnss_heading(t, heading);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GNSS heading update rejected");
    }
  }

  // ─── compass_msgs/Azimuth heading callback ───────────────────────────────
  // Handles the compass_msgs standard suggested by peci1.
  // Supports ENU/NED orientation and RAD/DEG units.
  // Converts to ENU radians before passing to the filter.

  void azimuth_callback(const compass_msgs::msg::Azimuth::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Convert to radians if needed
    double azimuth_rad = msg->azimuth;
    if (msg->unit == compass_msgs::msg::Azimuth::UNIT_DEG) {
      azimuth_rad = azimuth_rad * M_PI / 180.0;
    }

    // Convert to ENU yaw if needed
    // ENU: 0 = east, increases CCW: matches ROS REP-103
    // NED: 0 = north, increases CW: needs conversion
    double yaw_enu;
    if (msg->orientation == compass_msgs::msg::Azimuth::ORIENTATION_NED) {
      // NED azimuth to ENU yaw: yaw_enu = pi/2 - azimuth_ned
      yaw_enu = M_PI / 2.0 - azimuth_rad;
    } else {
      // Already ENU
      yaw_enu = azimuth_rad;
    }

    // Normalize to [-pi, pi]
    while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
    while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

    // Build heading struct
    fusioncore::sensors::GnssHeading heading;
    heading.heading_rad  = yaw_enu;
    heading.accuracy_rad = (msg->variance > 0.0) ? std::sqrt(msg->variance) : 0.02;
    heading.valid        = true;

    // Note magnetic vs geographic north
    // Geographic is preferred: magnetic has declination error
    if (msg->reference == compass_msgs::msg::Azimuth::REFERENCE_MAGNETIC) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "compass_msgs/Azimuth uses MAGNETIC north reference. "
        "Consider using GEOGRAPHIC for better accuracy.");
    }

    bool accepted = fc_->update_gnss_heading(t, heading);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Azimuth heading update rejected");
    }
  }

  // ─── Publish state ────────────────────────────────────────────────────────

  void publish_state()
  {
    std::lock_guard<std::mutex> lock(fc_mutex_);
    if (!fc_->is_initialized()) return;

    const fusioncore::State& s = fc_->get_state();
    auto stamp = now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x = s.x[fusioncore::X];
    odom.pose.pose.position.y = s.x[fusioncore::Y];
    odom.pose.pose.position.z = s.x[fusioncore::Z];

    tf2::Quaternion q;
    q.setRPY(s.x[fusioncore::ROLL],
             s.x[fusioncore::PITCH],
             s.x[fusioncore::YAW]);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = s.x[fusioncore::VX];
    odom.twist.twist.linear.y  = s.x[fusioncore::VY];
    odom.twist.twist.linear.z  = s.x[fusioncore::VZ];
    odom.twist.twist.angular.x = s.x[fusioncore::WX];
    odom.twist.twist.angular.y = s.x[fusioncore::WY];
    odom.twist.twist.angular.z = s.x[fusioncore::WZ];

    // Publish UKF covariance so Nav2 and other consumers see real uncertainty.
    // pose.covariance is 6x6 row-major for [x, y, z, roll, pitch, yaw].
    // twist.covariance is 6x6 row-major for [vx, vy, vz, wx, wy, wz].
    // Extract the relevant 6x6 sub-blocks from the 21x21 P matrix.
    const fusioncore::StateMatrix& P = s.P;
    // Pose state indices: X=0,Y=1,Z=2,ROLL=3,PITCH=4,YAW=5
    static constexpr int pose_idx[6] = {
      fusioncore::X, fusioncore::Y, fusioncore::Z,
      fusioncore::ROLL, fusioncore::PITCH, fusioncore::YAW
    };
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        odom.pose.covariance[i * 6 + j] = P(pose_idx[i], pose_idx[j]);

    // Twist state indices: VX=6,VY=7,VZ=8,WX=9,WY=10,WZ=11
    static constexpr int twist_idx[6] = {
      fusioncore::VX, fusioncore::VY, fusioncore::VZ,
      fusioncore::WX, fusioncore::WY, fusioncore::WZ
    };
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        odom.twist.covariance[i * 6 + j] = P(twist_idx[i], twist_idx[j]);

    odom_pub_->publish(odom);

    // Also publish PoseWithCovarianceStamped: expected by AMCL, slam_toolbox,
    // Nav2 pose initializer, and many visualization tools.
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose   = odom.pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id  = base_frame_;

    tf.transform.translation.x = s.x[fusioncore::X];
    tf.transform.translation.y = s.x[fusioncore::Y];
    tf.transform.translation.z = s.x[fusioncore::Z];
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  // ─── Diagnostics ─────────────────────────────────────────────────────────
  // Published at 1 Hz on /diagnostics (standard ROS convention).
  // Consumed by rqt_robot_monitor, Nav2 bringup, and production monitoring.

  void publish_diagnostics()
  {
    std::lock_guard<std::mutex> lock(fc_mutex_);
    if (!fc_->is_initialized()) return;

    auto status = fc_->get_status();
    auto stamp  = now();

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = stamp;

    auto make_status = [&](
      const std::string& name,
      uint8_t level,
      const std::string& message,
      const std::vector<std::pair<std::string,std::string>>& kv)
    {
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.name        = "fusioncore: " + name;
      s.hardware_id = "fusioncore";
      s.level       = level;
      s.message     = message;
      for (const auto& [k, v] : kv) {
        diagnostic_msgs::msg::KeyValue kv_msg;
        kv_msg.key   = k;
        kv_msg.value = v;
        s.values.push_back(kv_msg);
      }
      return s;
    };

    auto health_to_level = [](fusioncore::SensorHealth h) -> uint8_t {
      switch (h) {
        case fusioncore::SensorHealth::OK:       return diagnostic_msgs::msg::DiagnosticStatus::OK;
        case fusioncore::SensorHealth::STALE:    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
        case fusioncore::SensorHealth::NOT_INIT: return diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    };

    auto health_to_str = [](fusioncore::SensorHealth h) -> std::string {
      switch (h) {
        case fusioncore::SensorHealth::OK:       return "OK";
        case fusioncore::SensorHealth::STALE:    return "STALE: no recent data";
        case fusioncore::SensorHealth::NOT_INIT: return "Not yet initialized";
      }
      return "Unknown";
    };

    // IMU
    diag_array.status.push_back(make_status("IMU",
      health_to_level(status.imu_health),
      health_to_str(status.imu_health),
      {{"outlier_count", std::to_string(status.imu_outliers)}}));

    // Encoder
    diag_array.status.push_back(make_status("Encoder",
      health_to_level(status.encoder_health),
      health_to_str(status.encoder_health),
      {{"outlier_count", std::to_string(status.enc_outliers)}}));

    // GNSS
    diag_array.status.push_back(make_status("GNSS",
      health_to_level(status.gnss_health),
      health_to_str(status.gnss_health),
      {{"outlier_count",     std::to_string(status.gnss_outliers)},
       {"heading_outliers",  std::to_string(status.hdg_outliers)}}));

    // Filter
    auto heading_src_str = [](fusioncore::HeadingSource src) -> std::string {
      switch (src) {
        case fusioncore::HeadingSource::NONE:            return "NONE: lever arm inactive";
        case fusioncore::HeadingSource::DUAL_ANTENNA:    return "DUAL_ANTENNA";
        case fusioncore::HeadingSource::IMU_ORIENTATION: return "IMU_ORIENTATION (9-axis)";
        case fusioncore::HeadingSource::GPS_TRACK:       return "GPS_TRACK";
      }
      return "Unknown";
    };

    uint8_t filter_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string filter_msg = "Running";
    if (!status.heading_validated) {
      filter_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      filter_msg   = "Heading not validated: lever arm inactive";
    }

    diag_array.status.push_back(make_status("Filter",
      filter_level, filter_msg,
      {{"heading_source",        heading_src_str(status.heading_source)},
       {"heading_validated",     status.heading_validated ? "true" : "false"},
       {"distance_traveled_m",   std::to_string(status.distance_traveled)},
       {"position_uncertainty_m", std::to_string(std::sqrt(status.position_uncertainty))},
       {"update_count",          std::to_string(status.update_count)}}));

    diag_pub_->publish(diag_array);
  }

  // ─── PROJ coordinate transforms ───────────────────────────────────────────
  // Replaces hand-coded WGS84→ECEF math with the PROJ library.
  // Default config (EPSG:4326 → EPSG:4978) is numerically equivalent to the
  // old lla_to_ecef() but supports any input/output CRS for agricultural RTK
  // and other projected coordinate systems.

  void init_proj()
  {
    std::lock_guard<std::mutex> lock(proj_mutex_);
    if (proj_initialized_) return;

    proj_ctx_ = proj_context_create();

    // proj_create_crs_to_crs automatically selects the best pipeline between CRS.
    // proj_normalize_for_visualization ensures consistent axis order:
    // input: (latitude°, longitude°, altitude_m) for geographic CRS
    // output: (x, y, z) in the output CRS native units
    PJ* raw = proj_create_crs_to_crs(proj_ctx_,
      input_gnss_crs_.c_str(), output_crs_.c_str(), nullptr);

    if (!raw) {
      RCLCPP_ERROR(get_logger(), "PROJ: failed to create transform %s → %s. "
        "Check that both CRS strings are valid PROJ identifiers.",
        input_gnss_crs_.c_str(), output_crs_.c_str());
      return;
    }

    proj_ = proj_normalize_for_visualization(proj_ctx_, raw);
    proj_destroy(raw);

    if (!proj_) {
      RCLCPP_ERROR(get_logger(), "PROJ: failed to normalize transform axis order");
      return;
    }

    proj_initialized_ = true;
    RCLCPP_INFO(get_logger(), "PROJ: transform ready (%s → %s)",
      input_gnss_crs_.c_str(), output_crs_.c_str());
  }

  void deinit_proj()
  {
    std::lock_guard<std::mutex> lock(proj_mutex_);
    if (proj_)    { proj_destroy(proj_);          proj_    = nullptr; }
    if (proj_ctx_){ proj_context_destroy(proj_ctx_); proj_ctx_ = nullptr; }
    proj_initialized_ = false;
  }

  // gnss_to_output: LLA (radians) → output CRS (x, y, z).
  // Bug 1 fix: proj_normalize_for_visualization makes EPSG:4326 expect degrees,
  // so we convert from the internal radians to degrees before passing to PROJ.
  void gnss_to_output(
    const fusioncore::sensors::LLAPoint& lla,
    fusioncore::sensors::ECEFPoint& out)
  {
    if (!proj_initialized_) {
      RCLCPP_ERROR_ONCE(get_logger(), "PROJ transform not initialized");
      return;
    }
    std::lock_guard<std::mutex> lock(proj_mutex_);
    PJ_COORD c = {{
      lla.lat_rad * 180.0 / M_PI,   // degrees (after normalize_for_visualization)
      lla.lon_rad * 180.0 / M_PI,
      lla.alt_m,
      HUGE_VAL
    }};
    PJ_COORD r = proj_trans(proj_, PJ_FWD, c);
    out.x = r.xyz.x;
    out.y = r.xyz.y;
    out.z = r.xyz.z;
  }

  // output_to_gnss: output CRS (x, y, z) → LLA (radians).
  // Bug 2 fix: phi and lam come back in degrees after normalization,
  // so we convert to radians before storing in LLAPoint.
  void output_to_gnss(
    const fusioncore::sensors::ECEFPoint& in,
    fusioncore::sensors::LLAPoint& lla)
  {
    if (!proj_initialized_) {
      RCLCPP_ERROR_ONCE(get_logger(), "PROJ transform not initialized");
      return;
    }
    std::lock_guard<std::mutex> lock(proj_mutex_);
    PJ_COORD c = {{ in.x, in.y, in.z, HUGE_VAL }};
    PJ_COORD r = proj_trans(proj_, PJ_INV, c);
    lla.lat_rad = r.lpzt.phi * M_PI / 180.0;   // degrees → radians
    lla.lon_rad = r.lpzt.lam * M_PI / 180.0;
    lla.alt_m   = r.lpzt.z;
  }

  // ─── Members ──────────────────────────────────────────────────────────────

  std::unique_ptr<fusioncore::FusionCore>        fc_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub_;
  rclcpp::Subscription<compass_msgs::msg::Azimuth>::SharedPtr     azimuth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          gnss_heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                gnss2_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                       odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr         diag_pub_;
  rclcpp::TimerBase::SharedPtr                                                publish_timer_;
  rclcpp::TimerBase::SharedPtr                                                diag_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                          reset_srv_;

  std::string base_frame_;
  std::string odom_frame_;
  double      publish_rate_;
  std::string heading_topic_;
  std::string gnss2_topic_;
  std::string azimuth_topic_;

  bool   pending_init_   = false;
  bool   gnss_ref_set_   = false;
  double last_imu_time_  = 0.0;   // timestamp of most recent IMU message
  fusioncore::sensors::LLAPoint  gnss_ref_lla_;
  fusioncore::sensors::ECEFPoint gnss_ref_ecef_;

  fusioncore::sensors::GnssFixType  min_fix_type_   = fusioncore::sensors::GnssFixType::GPS_FIX;
  fusioncore::sensors::GnssLeverArm gnss_lever_arm_;    // primary receiver
  fusioncore::sensors::GnssLeverArm gnss_lever_arm2_;   // secondary receiver (fix2_topic)

  // ZUPT parameters
  bool   zupt_enabled_            = true;
  double zupt_velocity_threshold_ = 0.05;
  double zupt_angular_threshold_  = 0.05;
  double zupt_noise_sigma_        = 0.01;

  // Callback groups: sensor callbacks are mutually exclusive (protect UKF state);
  // publish timer runs in its own group so it never waits on a sensor callback.
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr publish_cb_group_;
  std::mutex fc_mutex_;

  // PROJ coordinate transform members
  std::string input_gnss_crs_;
  std::string output_crs_;
  bool convert_to_enu_at_reference_ = true;
  bool reference_use_first_fix_     = true;
  std::mutex proj_mutex_;
  bool       proj_initialized_ = false;
  PJ        *proj_     = nullptr;
  PJ_CONTEXT*proj_ctx_ = nullptr;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNode>();
  // Two threads: one serves the sensor callback group (IMU/GPS/encoder),
  // the other serves the publish timer group: so the 100 Hz publish never
  // stalls waiting for a sensor update to finish.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
