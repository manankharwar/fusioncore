#include "fusioncore/fusioncore.hpp"
#include "fusioncore/sensors/gnss.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

    // Declare parameters
    declare_parameter("base_frame",   "base_link");
    declare_parameter("odom_frame",   "odom");
    declare_parameter("publish_rate", 100.0);

    // IMU noise
    declare_parameter("imu.gyro_noise",  0.005);
    declare_parameter("imu.accel_noise", 0.1);

    // Encoder noise
    declare_parameter("encoder.vel_noise",    0.05);
    declare_parameter("encoder.yaw_noise",    0.02);

    // GNSS params
    declare_parameter("gnss.base_noise_xy",   1.0);
    declare_parameter("gnss.base_noise_z",    2.0);
    declare_parameter("gnss.heading_noise",   0.02);
    declare_parameter("gnss.max_hdop",        4.0);
    declare_parameter("gnss.min_satellites",  4);

    // UKF process noise
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

    // Build config from parameters
    fusioncore::FusionCoreConfig config;

    config.imu.gyro_noise_x  = get_parameter("imu.gyro_noise").as_double();
    config.imu.gyro_noise_y  = config.imu.gyro_noise_x;
    config.imu.gyro_noise_z  = config.imu.gyro_noise_x;
    config.imu.accel_noise_x = get_parameter("imu.accel_noise").as_double();
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

    config.ukf.q_position     = get_parameter("ukf.q_position").as_double();
    config.ukf.q_orientation  = get_parameter("ukf.q_orientation").as_double();
    config.ukf.q_velocity     = get_parameter("ukf.q_velocity").as_double();
    config.ukf.q_angular_vel  = get_parameter("ukf.q_angular_vel").as_double();
    config.ukf.q_acceleration = get_parameter("ukf.q_acceleration").as_double();
    config.ukf.q_gyro_bias    = get_parameter("ukf.q_gyro_bias").as_double();
    config.ukf.q_accel_bias   = get_parameter("ukf.q_accel_bias").as_double();

    fc_ = std::make_unique<fusioncore::FusionCore>(config);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "FusionCore configured. base_frame=%s odom_frame=%s rate=%.0fHz",
      base_frame_.c_str(), odom_frame_.c_str(), publish_rate_);

    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Activate ───────────────────────────────────────────────────

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating FusionCore...");

    // Initialize filter at origin
    fusioncore::State initial;
    initial.x = fusioncore::StateVector::Zero();
    initial.P = fusioncore::StateMatrix::Identity() * 0.1;
    fc_->init(initial, now().seconds());

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

    encoder_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/wheels", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { encoder_callback(msg); });

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix", 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { gnss_callback(msg); });

    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fusion/odom", 100);

    // Publish timer
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
    publish_timer_.reset();
    odom_pub_.reset();
    RCLCPP_INFO(get_logger(), "FusionCore deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    fc_.reset();
    tf_broadcaster_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

private:

  // ─── IMU callback ─────────────────────────────────────────────────────────

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    fc_->update_imu(t,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z,
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z
    );
  }

  // ─── Encoder callback ─────────────────────────────────────────────────────

  void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    fc_->update_encoder(t,
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.angular.z
    );
  }

  // ─── GNSS callback ────────────────────────────────────────────────────────

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;

    // Reject no-fix
    if (msg->status.status < 0) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Convert lat/lon/alt to ENU using first fix as reference
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

    // Convert current fix to ENU
    fusioncore::sensors::LLAPoint lla;
    lla.lat_rad = msg->latitude  * M_PI / 180.0;
    lla.lon_rad = msg->longitude * M_PI / 180.0;
    lla.alt_m   = msg->altitude;

    fusioncore::sensors::ECEFPoint ecef;
    lla_to_ecef(lla, ecef);

    Eigen::Vector3d enu = fusioncore::sensors::ecef_to_enu(
      ecef, gnss_ref_ecef_, gnss_ref_lla_);

    // Build fix struct
    fusioncore::sensors::GnssFix fix;
    fix.x = enu[0];
    fix.y = enu[1];
    fix.z = enu[2];
    fix.satellites = 6;   // NavSatFix doesn't expose sat count directly
    fix.hdop = 1.5;       // NavSatFix doesn't expose HDOP directly
    fix.vdop = 2.0;
    fix.fix_type = fusioncore::sensors::GnssFixType::GPS_FIX;

    // Use position covariance if available
    if (msg->position_covariance_type > 0) {
      double sigma_xy = std::sqrt(msg->position_covariance[0]);
      double sigma_z  = std::sqrt(msg->position_covariance[8]);
      fix.hdop = sigma_xy;
      fix.vdop = sigma_z;
    }

    bool accepted = fc_->update_gnss(t, fix);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GNSS fix rejected (poor quality)");
    }
  }

  // ─── Publish state ────────────────────────────────────────────────────────

  void publish_state()
  {
    if (!fc_->is_initialized()) return;

    const fusioncore::State& s = fc_->get_state();
    auto stamp = now();

    // Publish nav_msgs/Odometry
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

    // Broadcast TF: odom -> base_link
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
    const double a   = 6378137.0;
    const double e2  = 0.00669437999014;
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

  std::unique_ptr<fusioncore::FusionCore> fc_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    gnss_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  rclcpp::TimerBase::SharedPtr                                    publish_timer_;

  std::string base_frame_;
  std::string odom_frame_;
  double      publish_rate_;

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
