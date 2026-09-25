// gnss_doppler_bridge: converts ublox NavPVT NED velocity to nav_msgs/Odometry (ENU)
// and publishes on gnss.velocity_topic so FusionCore can fuse GPS Doppler speed.
// Only published when fix_type >= 3 (3D fix or better) AND gnssFixOK flag is set.

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

class GnssDopplerBridge : public rclcpp::Node
{
public:
  GnssDopplerBridge() : Node("gnss_doppler_bridge")
  {
    declare_parameter<std::string>("navpvt_topic",  "/ublox/navpvt");
    declare_parameter<std::string>("output_topic",  "/gnss/doppler_vel");
    declare_parameter<double>("min_speed_mps",  0.05);  // ignore near-zero speeds

    const auto in  = get_parameter("navpvt_topic").as_string();
    const auto out = get_parameter("output_topic").as_string();

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out, 10);
    sub_ = create_subscription<ublox_msgs::msg::NavPVT>(
      in, 10,
      [this](const ublox_msgs::msg::NavPVT::SharedPtr msg) { on_navpvt(msg); });

    RCLCPP_INFO(get_logger(), "gnss_doppler_bridge: %s -> %s", in.c_str(), out.c_str());
  }

private:
  void on_navpvt(const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    // require 3D fix (3) or better, and gnssFixOK (bit 0 of flags)
    if (msg->fix_type < 3 || !(msg->flags & 0x01))
      return;

    // NAV-PVT NED velocities are int32 mm/s; s_acc is uint32 mm/s 1-sigma
    const double east  =  msg->vel_e / 1000.0;
    const double north =  msg->vel_n / 1000.0;
    const double up    = -msg->vel_d / 1000.0;

    const double min_spd = get_parameter("min_speed_mps").as_double();
    const double ground_speed = std::hypot(east, north);
    if (ground_speed < min_spd)
      return;

    const double sigma  = msg->s_acc / 1000.0;
    const double var    = sigma * sigma;

    nav_msgs::msg::Odometry odom;
    // NavPVT carries no std_msgs/Header, so there is nothing to copy. This used to
    // read msg->header.stamp, which does not compile at all: the whole bridge is
    // built only under find_package(ublox_msgs QUIET), so on every machine without
    // ublox_msgs installed it was silently skipped and the error never surfaced.
    //
    // Node time rather than the receiver's own clock. NavPVT does carry UTC fields
    // and i_tow, but FusionCore times its IMU replay buffer against ROS time, so a
    // receiver clock that disagrees with the system clock would shift every Doppler
    // measurement against the IMU it is being fused with. The cost of stamping on
    // arrival is the USB and driver latency, which is small and constant; the cost
    // of the other choice is a systematic and variable offset.
    odom.header.stamp    = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.twist.twist.linear.x = east;
    odom.twist.twist.linear.y = north;
    odom.twist.twist.linear.z = up;

    // diagonal: east/north variances from s_acc; rest large to signal unused
    odom.twist.covariance[0]  = var;
    odom.twist.covariance[7]  = var;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 1e6;

    pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssDopplerBridge>());
  rclcpp::shutdown();
  return 0;
}
