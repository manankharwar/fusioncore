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
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  FusionNode()
  : rclcpp_lifecycle::LifecycleNode("fusioncore")
  {
    RCLCPP_INFO(get_logger(), "FusionCore node created");
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
    // Set to false for 6-axis IMUs — yaw from gyro integration drifts
    declare_parameter("imu.has_magnetometer", false);
    declare_parameter("imu.accel_noise", 0.1);

    declare_parameter("encoder.vel_noise", 0.05);
    declare_parameter("encoder.yaw_noise", 0.02);

    declare_parameter("gnss.base_noise_xy",  1.0);
    declare_parameter("gnss.base_noise_z",   2.0);
    declare_parameter("gnss.heading_noise",  0.02);
    declare_parameter("gnss.max_hdop",       4.0);
    declare_parameter("gnss.min_satellites", 4);

    // Topic for dual antenna heading — sensor_msgs/Imu used as heading carrier.
    // The yaw component of orientation is the heading.
    // Set to empty string to disable dual antenna heading.
    declare_parameter("gnss.heading_topic", "/gnss/heading");

    // Optional second GNSS receiver topic — set to empty string to disable
    declare_parameter("gnss.fix2_topic", "");

    // Antenna lever arm params
    declare_parameter("gnss.lever_arm_x", 0.0);
    declare_parameter("gnss.lever_arm_y", 0.0);
    declare_parameter("gnss.lever_arm_z", 0.0);

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
    gnss2_topic_   = get_parameter("gnss.fix2_topic").as_string();

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
    config.gnss.lever_arm.x    = get_parameter("gnss.lever_arm_x").as_double();
    config.gnss.lever_arm.y    = get_parameter("gnss.lever_arm_y").as_double();
    config.gnss.lever_arm.z    = get_parameter("gnss.lever_arm_z").as_double();

    if (!config.gnss.lever_arm.is_zero()) {
      RCLCPP_INFO(get_logger(),
        "GNSS lever arm set: x=%.3f y=%.3f z=%.3f m",
        config.gnss.lever_arm.x,
        config.gnss.lever_arm.y,
        config.gnss.lever_arm.z);
    }

    config.ukf.q_position     = get_parameter("ukf.q_position").as_double();
    config.ukf.q_orientation  = get_parameter("ukf.q_orientation").as_double();
    config.ukf.q_velocity     = get_parameter("ukf.q_velocity").as_double();
    config.ukf.q_angular_vel  = get_parameter("ukf.q_angular_vel").as_double();
    config.ukf.q_acceleration = get_parameter("ukf.q_acceleration").as_double();
    config.ukf.q_gyro_bias    = get_parameter("ukf.q_gyro_bias").as_double();
    config.ukf.q_accel_bias   = get_parameter("ukf.q_accel_bias").as_double();

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

    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Activate ───────────────────────────────────────────────────

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating FusionCore...");

    fusioncore::State initial;
    initial.x = fusioncore::StateVector::Zero();
    initial.P = fusioncore::StateMatrix::Identity() * 0.1;
    fc_->init(initial, now().seconds());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

    encoder_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/wheels", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { encoder_callback(msg); });

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix", 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gnss_callback(msg, 0);
      });

    // Second GNSS receiver — optional
    if (!gnss2_topic_.empty()) {
      gnss2_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss2_topic_, 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          gnss_callback(msg, 1);
        });
      RCLCPP_INFO(get_logger(),
        "Second GNSS receiver enabled on topic: %s", gnss2_topic_.c_str());
    }

    // Dual antenna heading subscriber — only if topic is configured
    // Expects sensor_msgs/Imu where orientation.z/w gives the yaw heading.
    // This is the standard way dual antenna GPS receivers report heading in ROS.
    if (!heading_topic_.empty()) {
      gnss_heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        heading_topic_, 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          gnss_heading_callback(msg);
        });
      RCLCPP_INFO(get_logger(),
        "Subscribed to dual antenna heading: %s", heading_topic_.c_str());
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fusion/odom", 100);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); });

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
    publish_timer_.reset();
    odom_pub_.reset();
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

  // ─── IMU callback — with frame transform ──────────────────────────────────

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

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
      fuse_imu_orientation_if_valid(t, msg);
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
    fuse_imu_orientation_if_valid(t, msg);
  }

  // ─── IMU orientation helper ───────────────────────────────────────────────
  // Called after every IMU update if the message contains a valid orientation.
  // Handles IMUs like BNO08x, VectorNav, Xsens that publish full orientation.
  // Uses message covariance when available (peci1 fix).

  void fuse_imu_orientation_if_valid(
    double t,
    const sensor_msgs::msg::Imu::SharedPtr& msg)
  {
    // orientation_covariance[0] == -1 means "no orientation data"
    if (msg->orientation_covariance[0] < 0.0) return;

    // All zeros also means unknown — skip
    bool has_orientation = false;
    for (int i = 0; i < 9; ++i) {
      if (msg->orientation_covariance[i] != 0.0) {
        has_orientation = true;
        break;
      }
    }
    if (!has_orientation) return;

    // Extract roll, pitch, yaw from quaternion
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    fc_->update_imu_orientation(
      t, roll, pitch, yaw,
      msg->orientation_covariance.data());
  }

  // ─── Encoder callback ─────────────────────────────────────────────────────

  void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    fc_->update_encoder(t,
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.angular.z);
  }

  // ─── GNSS position callback ────────────────────────────────────────────────

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg, int source_id = 0)
  {
    if (!fc_->is_initialized()) return;

    if (msg->status.status < 0) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    if (!gnss_ref_set_) {
      gnss_ref_lla_.lat_rad = msg->latitude  * M_PI / 180.0;
      gnss_ref_lla_.lon_rad = msg->longitude * M_PI / 180.0;
      gnss_ref_lla_.alt_m   = msg->altitude;
      lla_to_ecef(gnss_ref_lla_, gnss_ref_ecef_);
      gnss_ref_set_ = true;
      RCLCPP_INFO(get_logger(), "GNSS reference set: lat=%.6f lon=%.6f",
        msg->latitude, msg->longitude);
      return;
    }

    fusioncore::sensors::LLAPoint lla;
    lla.lat_rad = msg->latitude  * M_PI / 180.0;
    lla.lon_rad = msg->longitude * M_PI / 180.0;
    lla.alt_m   = msg->altitude;

    fusioncore::sensors::ECEFPoint ecef;
    lla_to_ecef(lla, ecef);

    Eigen::Vector3d enu = fusioncore::sensors::ecef_to_enu(
      ecef, gnss_ref_ecef_, gnss_ref_lla_);

    fusioncore::sensors::GnssFix fix;
    fix.x = enu[0];
    fix.y = enu[1];
    fix.z = enu[2];
    fix.fix_type  = fusioncore::sensors::GnssFixType::GPS_FIX;
    fix.source_id = source_id;

    // Use message covariance when meaningful (peci1 fix)
    // position_covariance_type:
    //   0 = unknown
    //   1 = approximated (diagonal only)
    //   2 = diagonal known
    //   3 = full matrix known — use off-diagonal elements too
    if (msg->position_covariance_type == 3) {
      // Full 3x3 covariance available — use it directly including off-diagonals
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
        fix.satellites = 6;
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
        fix.satellites = 6;
      }
    } else if (msg->position_covariance_type >= 1) {
      // Diagonal covariance available
      double var_xy = (msg->position_covariance[0] + msg->position_covariance[4]) / 2.0;
      double var_z  = msg->position_covariance[8];
      if (var_xy > 0.0 && var_z > 0.0) {
        fix.hdop = std::sqrt(var_xy);
        fix.vdop = std::sqrt(var_z);
        fix.satellites = 6;
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
        fix.satellites = 6;
      }
    } else {
      // Unknown covariance — use config defaults
      fix.hdop = 1.5;
      fix.vdop = 2.0;
      fix.satellites = 6;
    }

    bool accepted = fc_->update_gnss(t, fix);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GNSS fix rejected (poor quality)");
    }

    // Log heading observability status
    auto fc_status = fc_->get_status();
    if (!fc_status.heading_validated) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Heading not yet validated — lever arm inactive. "
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

    // Check orientation covariance — if all zeros the orientation is invalid
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

  // ─── Publish state ────────────────────────────────────────────────────────

  void publish_state()
  {
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

    odom_pub_->publish(odom);

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

  // ─── LLA to ECEF ──────────────────────────────────────────────────────────

  void lla_to_ecef(
    const fusioncore::sensors::LLAPoint& lla,
    fusioncore::sensors::ECEFPoint& ecef)
  {
    const double a  = 6378137.0;
    const double e2 = 0.00669437999014;
    double sin_lat = std::sin(lla.lat_rad);
    double cos_lat = std::cos(lla.lat_rad);
    double sin_lon = std::sin(lla.lon_rad);
    double cos_lon = std::cos(lla.lon_rad);
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    ecef.x = (N + lla.alt_m) * cos_lat * cos_lon;
    ecef.y = (N + lla.alt_m) * cos_lat * sin_lon;
    ecef.z = (N * (1.0 - e2) + lla.alt_m) * sin_lat;
  }

  // ─── Members ──────────────────────────────────────────────────────────────

  std::unique_ptr<fusioncore::FusionCore>        fc_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          gnss_heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    gnss2_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  rclcpp::TimerBase::SharedPtr                                    publish_timer_;

  std::string base_frame_;
  std::string odom_frame_;
  double      publish_rate_;
  std::string heading_topic_;
  std::string gnss2_topic_;

  bool gnss_ref_set_ = false;
  fusioncore::sensors::LLAPoint  gnss_ref_lla_;
  fusioncore::sensors::ECEFPoint gnss_ref_ecef_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
