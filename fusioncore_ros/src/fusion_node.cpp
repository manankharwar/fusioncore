#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"
#include "fusioncore/sensors/gnss.hpp"
#include "fusioncore/sensors/vslam.hpp"
#include "fusioncore/sensors/magnetometer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <compass_msgs/msg/azimuth.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "fusioncore_ros/srv/from_ll.hpp"
#include "fusioncore_ros/msg/gnss_status.hpp"
#include "fusioncore_ros/msg/filter_health.hpp"
#include <lifecycle_msgs/msg/transition.hpp>
#include <mutex>
#include <optional>
#include <set>
#include <fstream>
#include <sstream>
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
    // Force 2D output: zero the Z position in the published odometry
    // and the odom->base TF. For ground robots where altitude is
    // irrelevant (mower, vacuum, AGV), this prevents any GPS-altitude
    // or IMU-Z drift from moving the costmap rolling window out of the
    // 2D navigation plane. Orientation (roll/pitch) is untouched.
    declare_parameter("publish.force_2d", false);
    // Set to false when another node owns the odom->base_link TF
    // (e.g. a separate sensor fusion layer). FusionCore will still
    // publish /fusion/odom; only the TF broadcast is suppressed.
    declare_parameter("publish.tf", true);

    // When true (default), the node self-transitions configure -> activate
    // automatically ~200ms after on_configure() returns. This removes the
    // need for external callers to manage lifecycle transitions manually.
    // Set to false when your launch file drives transitions itself (e.g. via
    // OnStateTransition or nav2_lifecycle_manager) to avoid a double-activate.
    declare_parameter("autostart", true);

    // Primary IMU topic. Override when your driver publishes at a non-default topic
    // (e.g. Clearpath Microstrain at /sensors/imu_0/data, Realsense at /camera/imu).
    // Using a launch-time remap is equivalent and preferred for readability.
    declare_parameter("imu.topic", std::string("/imu/data"));
    declare_parameter("imu.gyro_noise",  0.005);
    // Set to true if IMU has a magnetometer (9-axis: BNO08x, VectorNav, Xsens)
    // Set to false for 6-axis IMUs: yaw from gyro integration drifts
    declare_parameter("imu.has_magnetometer", false);
    declare_parameter("imu.accel_noise", 0.1);
    // Override frame_id for IMU messages. When non-empty, FusionCore uses this
    // frame instead of msg->header.frame_id. Useful when the IMU driver publishes
    // with an empty or wrong frame_id. Leave empty to use the message frame_id
    // (falls back to "imu_link" if the message frame_id is also empty).
    declare_parameter("imu.frame_id", std::string(""));
    // Set to true ONLY if your IMU driver has ALREADY removed gravity and
    // publishes "linear acceleration" (true body acceleration, not specific force).
    // Most IMUs publish raw specific force (gravity included): leave this false.
    // The filter measurement model always expects specific force. If your IMU
    // already subtracted gravity, enable this to add gravity back before fusing.
    declare_parameter("imu.remove_gravitational_acceleration", false);

    // IMU lever arm (offset from base_link to IMU sensing point, body frame).
    // Leave at 0 to auto-resolve from TF (base_frame -> imu_frame translation
    // via the URDF). Set non-zero to override the TF value.
    declare_parameter("imu.lever_arm_x", 0.0);
    declare_parameter("imu.lever_arm_y", 0.0);
    declare_parameter("imu.lever_arm_z", 0.0);

    // Optional second IMU source. When non-empty, FusionCore subscribes to this
    // topic and calls update_imu() for each message, treating the two sensors as
    // independent measurements of the same state. Uses the same noise model as
    // the primary IMU. Useful when your platform has two IMUs and you want
    // redundancy rather than pre-merging them externally (e.g. VESC IMU + D435i).
    // Leave empty to disable.
    declare_parameter("imu2.topic",    std::string(""));
    declare_parameter("imu2.frame_id", std::string(""));
    declare_parameter("imu2.remove_gravitational_acceleration", false);

    // Primary wheel odometry topic (nav_msgs/Odometry, twist is what gets fused).
    // The default is deliberately not the conventional /odom: FusionCore publishes
    // its own fused odometry, and subscribing to /odom would invite a feedback loop
    // with its own output. Most drivers publish somewhere else (/odom,
    // /diff_drive_controller/odom, ...), so point this at yours rather than writing
    // a launch remap. A remap still works and takes effect on whatever name is set here.
    declare_parameter("encoder.topic", std::string("/odom/wheels"));
    declare_parameter("encoder.vel_noise", 0.05);
    declare_parameter("encoder.yaw_noise", 0.02);

    // Optional second encoder-twist source (e.g. KISS-ICP LiDAR odometry).
    // When non-empty, FusionCore subscribes to this topic as nav_msgs/Odometry
    // and fuses twist.linear.x/y + twist.angular.z using the same update_encoder
    // path as the primary wheel encoder. Per-axis covariance is taken from the
    // message twist.covariance when positive; otherwise encoder2.vel_noise and
    // encoder2.yaw_noise are used as fallback. Leave empty to disable.
    declare_parameter("encoder2.topic",     std::string(""));
    declare_parameter("encoder2.vel_noise", 0.05);
    declare_parameter("encoder2.yaw_noise", 0.02);

    // GPS velocity topic: fuses horizontal speed from any receiver that outputs
    // nav_msgs/Odometry with velocity in the ENU (world) frame.
    // linear.x = east, linear.y = north. Rotated to body frame internally.
    // Leave empty to disable. Works with F9P, Septentrio, and any bridge node
    // that republishes GPS velocity as nav_msgs/Odometry.
    declare_parameter("gnss.velocity_topic", std::string(""));

    // Radar Doppler velocity topic: fuses ego-velocity from a 4D imaging radar.
    // Accepts nav_msgs/Odometry with velocity in the robot body frame:
    //   linear.x = forward (m/s), linear.y = lateral (m/s)
    // A bridge node extracts ego-velocity from raw radar point cloud Doppler and
    // publishes here. Works indoors and outdoors, all weather conditions.
    // Leave empty to disable.
    declare_parameter("radar.velocity_topic", std::string(""));
    declare_parameter("radar.vel_noise",      0.1);   // m/s fallback when msg cov <= 0

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
    declare_parameter("gnss.heading_topic", "");

    // Primary GNSS fix topic. Applies to both message types: it is read as
    // sensor_msgs/NavSatFix by default, or gps_msgs/GPSFix when gnss.use_gps_fix
    // is true. Real drivers publish on /fix, /ublox/fix, /gps/fix, and so on, so
    // set this instead of writing a launch remap (a remap still works and applies
    // to whatever name is set here).
    declare_parameter("gnss.fix_topic", std::string("/gnss/fix"));

    // Optional second GNSS receiver topic: set to empty string to disable
    declare_parameter("gnss.fix2_topic", "");

    // compass_msgs/Azimuth heading topic: peci1 standard
    // Set to empty string to disable (use sensor_msgs/Imu heading instead)
    declare_parameter("gnss.azimuth_topic", "");

    // Read gnss.fix_topic as gps_msgs/GPSFix instead of sensor_msgs/NavSatFix.
    // GPSFix carries RTK_FLOAT status (unreachable via NavSatFix), separate hdop/vdop
    // from the receiver, satellites_used, and err_horz/err_vert position bounds.
    // Required when your GNSS driver publishes gps_msgs/GPSFix (e.g. nmea_navsat_driver
    // with fix_type=RTK_FLOAT or ublox_dgnss in GPSFix mode).
    // Set false (default) to use sensor_msgs/NavSatFix: works with all receivers.
    declare_parameter("gnss.use_gps_fix", false);

    // Antenna lever arm params: primary receiver.
    // Leave at 0 to auto-resolve from TF (base_frame -> GNSS msg frame_id
    // translation, via URDF); set non-zero to override.
    declare_parameter("gnss.lever_arm_x", 0.0);
    declare_parameter("gnss.lever_arm_y", 0.0);
    declare_parameter("gnss.lever_arm_z", 0.0);

    // When true, the GNSS lever arm is applied from the very first fix, not
    // only after heading_validated_. Let RTK-grade fixes observe yaw directly
    // through the antenna-offset projection from startup rather than waiting
    // for the heading_observable_distance integration. Safe with Mahalanobis
    // gating on.
    declare_parameter("gnss.apply_lever_arm_pre_heading", false);

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
    declare_parameter("outlier_threshold_enc",   11.34);
    declare_parameter("outlier_threshold_hdg",   10.83);
    declare_parameter("outlier_threshold_vslam", 22.46);
    declare_parameter("gnss.max_speed",          0.0);
    declare_parameter("gnss.max_speed_margin",   5.0);
    // VSLAM pose input (ORB-SLAM3, RTAB-Map, Kimera, etc.)
    declare_parameter("vslam.topic",              std::string(""));
    declare_parameter("vslam.position_noise",     0.1);
    declare_parameter("vslam.orientation_noise",  0.02);
    declare_parameter("vslam.frame_id",           std::string(""));
    declare_parameter("vslam.reinit_n",           10);

    declare_parameter("gnss.coast_n",               5);
    declare_parameter("gnss.coast_min_gap_s",       1.0);
    declare_parameter("gnss.coast_q_factor",        20.0);
    declare_parameter("gnss.coast_timeout_s",       0.0);
    declare_parameter("gnss.coast_q_bias_factor",   100.0);
    declare_parameter("gnss.coast_imu_wz_scale",    1.0);
    declare_parameter("gnss.recovery_rejection_n",  0);
    declare_parameter("gnss.p_inflate_sigma",       50.0);
    declare_parameter("gnss.recovery_timeout_s",    0.0);
    declare_parameter("gnss.track_heading_enabled",   true);
    declare_parameter("gnss.track_heading_min_dist",  5.0);
    declare_parameter("gnss.track_heading_max_sigma", 0.4);
    declare_parameter("gnss.track_heading_min_speed",    0.2);
    declare_parameter("gnss.track_heading_max_yaw_rate", 0.3);
    declare_parameter("gnss.lever_arm_max_heading_sigma_deg", 20.0);

    declare_parameter("adaptive.imu",               true);
    declare_parameter("adaptive.encoder",           true);
    declare_parameter("adaptive.gnss",              true);
    declare_parameter("adaptive.ground_constraint", true);
    declare_parameter("adaptive.window",            50);
    declare_parameter("adaptive.alpha",             0.01);

    // Zero-velocity update (ZUPT)
    // When encoder velocity and IMU angular rate are both below threshold,
    // the robot is considered stationary and a zero-velocity measurement is fused.
    declare_parameter("zupt.enabled",            true);
    declare_parameter("zupt.velocity_threshold", 0.05);  // m/s
    declare_parameter("zupt.angular_threshold",  0.05);  // rad/s
    declare_parameter("zupt.noise_sigma",        0.01);  // m/s: tight

    // Raw magnetometer heading fusion.
    // Subscribe to sensor_msgs/MagneticField and fuse heading via UKF 1-DOF update.
    // Provides immediate yaw observability at startup and suppresses heading drift
    // during GPS outages. Requires hard/soft iron calibration for accurate results.
    // Set enabled: true only after calibrating with imu_calib or magneto.
    declare_parameter("magnetometer.enabled",        false);
    declare_parameter("magnetometer.topic",          std::string("/imu/mag"));
    declare_parameter("magnetometer.noise_rad",      0.05);
    declare_parameter("magnetometer.chi2_threshold", 9.21);
    declare_parameter("magnetometer.declination_rad", 0.0);
    // Hard iron bias (Tesla): [bx, by, bz]. Estimated from calibration.
    declare_parameter("magnetometer.hard_iron",
      std::vector<double>{0.0, 0.0, 0.0});
    // Soft iron scale matrix (row-major 3x3). Identity = no correction.
    declare_parameter("magnetometer.soft_iron",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    // Disturbance rejection by field magnitude. field_strength = local Earth
    // total field in the SAME units as the incoming reading (Gauss, microtesla);
    // a reading whose magnitude deviates by more than field_tolerance is treated
    // as locally disturbed (motor, steel) and rejected. 0.0 = disabled.
    declare_parameter("magnetometer.field_strength",  0.0);
    declare_parameter("magnetometer.field_tolerance", 0.2);

    // Lateral velocity NHC: how strongly to enforce VY=0 (m/s sigma).
    // 0.05 (default): standard differential drive on good surface.
    // 10.0+: effectively disabled, use for mecanum/omnidirectional robots.
    // 0.3-1.0: Ackermann robots or slippery surfaces with real lateral slip.
    declare_parameter("encoder.nhc_vy_sigma", 0.05);

    // Auto-detect holonomic (mecanum/omnidirectional) robots at runtime.
    // When true, FusionCore watches the encoder VY field. If VY is consistently
    // non-zero, the robot is identified as holonomic and the VY=0 NHC is
    // disabled automatically within ~0.1s of the first lateral motion.
    // No config change needed. Set to false only if auto-detect causes problems.
    declare_parameter("encoder.nhc_auto_detect", true);

    // Body-frame vertical velocity constraint (VZ=0) tightness (m/s sigma).
    // 0.1 (default): fine for flat floors and mild terrain.
    // 0.3-1.0: robots traversing curbs, obstacles, or rough outdoor terrain.
    declare_parameter("ground_constraint.vz_sigma", 0.1);

    // Body-frame vertical acceleration constraint (AZ=0) tightness (m/s² sigma).
    // 0.5 (default): loose enough for bumps and ramps.
    // 2.0+: aggressive terrain where real vertical accelerations occur.
    declare_parameter("ground_constraint.az_sigma", 0.5);

    // Flat-terrain Z position constraint. 0.0 = disabled (default).
    // Set to ~0.3 for campus/parking-lot/warehouse deployments where
    // GPS altitude noise would otherwise cause Z oscillations.
    declare_parameter("ground_constraint.z_position_sigma", 0.0);

    // Static bias initialization window (seconds, default 0 = disabled).
    // When > 0, the filter collects IMU data for this duration before starting.
    // Gyro and accel biases are estimated from the mean readings, eliminating
    // the ~60s startup transient caused by bias convergence from zero.
    // Only activates if the robot is stationary during the window (encoder check).
    declare_parameter("init.stationary_window", 0.0);

    // Wait for all configured sensors before starting the filter (default false).
    // When true, FusionCore holds initialization until every subscribed sensor
    // has published at least one message, so the filter starts with a full set
    // of measurements rather than drifting on IMU alone.
    // init.sensor_wait_timeout: give up and start anyway after this many seconds.
    declare_parameter("init.wait_for_all_sensors", false);
    declare_parameter("init.sensor_wait_timeout",  10.0);

    // Checkpoint path for deterministic replay (save/load filter state).
    // ~/save_checkpoint saves the current state to this file.
    // ~/load_checkpoint restores state from this file (re-run from any point in a bag).
    declare_parameter("replay.checkpoint_path",
      std::string("/tmp/fusioncore_checkpoint.txt"));

    // Motion model: controls how sigma points are propagated in the predict step.
    // "ConstantVelocityAcceleration" (default): no platform constraints.
    // "DifferentialDrive": zeros lateral velocity (VY) each predict step.
    // "Ackermann": same lateral constraint; wheelbase stored for future extensions.
    declare_parameter("motion_model", std::string("ConstantVelocityAcceleration"));
    declare_parameter("motion_model_params.wheelbase", 0.55);

    declare_parameter("ukf.q_position",     0.01);
    declare_parameter("ukf.q_orientation",  1e-9);
    declare_parameter("ukf.q_velocity",     0.1);
    declare_parameter("ukf.q_angular_vel",  0.1);
    declare_parameter("ukf.q_acceleration", 1.0);
    declare_parameter("ukf.q_gyro_bias",         1e-5);
    declare_parameter("ukf.q_accel_bias",        1e-5);
    declare_parameter("ukf.q_encoder_wz_bias",   1e-7);

    base_frame_   = get_parameter("base_frame").as_string();
    odom_frame_   = get_parameter("odom_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    force_2d_     = get_parameter("publish.force_2d").as_bool();
    publish_tf_   = get_parameter("publish.tf").as_bool();
    heading_topic_ = get_parameter("gnss.heading_topic").as_string();
    gnss_fix_topic_ = get_parameter("gnss.fix_topic").as_string();
    gnss2_topic_    = get_parameter("gnss.fix2_topic").as_string();
    azimuth_topic_  = get_parameter("gnss.azimuth_topic").as_string();
    use_gps_fix_    = get_parameter("gnss.use_gps_fix").as_bool();

    fusioncore::FusionCoreConfig config;

    config.imu.gyro_noise_x  = get_parameter("imu.gyro_noise").as_double();
    config.imu.gyro_noise_y  = config.imu.gyro_noise_x;
    config.imu.gyro_noise_z  = config.imu.gyro_noise_x;
    config.imu.accel_noise_x    = get_parameter("imu.accel_noise").as_double();
    config.imu_has_magnetometer = get_parameter("imu.has_magnetometer").as_bool();
    config.imu.accel_noise_y = config.imu.accel_noise_x;
    config.imu.accel_noise_z = config.imu.accel_noise_x;
    imu_topic_          = get_parameter("imu.topic").as_string();
    imu_remove_gravity_ = get_parameter("imu.remove_gravitational_acceleration").as_bool();
    imu_frame_override_ = get_parameter("imu.frame_id").as_string();

    // IMU lever arm: start from explicit params; if all zero, the ROS
    // wrapper will auto-resolve from TF (base_frame -> imu_frame) on the
    // first IMU message and call fc_->set_imu_lever_arm() with
    // the translation it extracts from URDF.
    config.imu.lever_arm.x = get_parameter("imu.lever_arm_x").as_double();
    config.imu.lever_arm.y = get_parameter("imu.lever_arm_y").as_double();
    config.imu.lever_arm.z = get_parameter("imu.lever_arm_z").as_double();
    imu_lever_arm_explicit_ = !config.imu.lever_arm.is_zero();
    if (imu_lever_arm_explicit_) {
      RCLCPP_INFO(get_logger(),
        "IMU lever arm (explicit): x=%.3f y=%.3f z=%.3f m",
        config.imu.lever_arm.x, config.imu.lever_arm.y, config.imu.lever_arm.z);
    } else {
      RCLCPP_INFO(get_logger(),
        "IMU lever arm: will auto-resolve from TF on first IMU message");
    }
    RCLCPP_INFO(get_logger(), "IMU gravity removal: %s",
      imu_remove_gravity_ ? "ENABLED" : "disabled");
    if (!imu_frame_override_.empty())
      RCLCPP_INFO(get_logger(), "IMU frame override: %s", imu_frame_override_.c_str());

    imu2_topic_          = get_parameter("imu2.topic").as_string();
    imu2_frame_override_ = get_parameter("imu2.frame_id").as_string();
    imu2_remove_gravity_ = get_parameter("imu2.remove_gravitational_acceleration").as_bool();

    config.encoder.vel_noise_x  = get_parameter("encoder.vel_noise").as_double();
    config.encoder.vel_noise_y  = get_parameter("encoder.nhc_vy_sigma").as_double();
    config.encoder.vel_noise_wz = get_parameter("encoder.yaw_noise").as_double();
    nhc_auto_detect_    = get_parameter("encoder.nhc_auto_detect").as_bool();
    nhc_vy_auto_noise_  = config.encoder.vel_noise_x;  // VY noise proxy for holonomic robots

    encoder_topic_      = get_parameter("encoder.topic").as_string();
    encoder2_topic_     = get_parameter("encoder2.topic").as_string();
    enc2_vel_noise_     = get_parameter("encoder2.vel_noise").as_double();
    enc2_yaw_noise_     = get_parameter("encoder2.yaw_noise").as_double();
    gnss_vel_topic_    = get_parameter("gnss.velocity_topic").as_string();
    radar_vel_topic_   = get_parameter("radar.velocity_topic").as_string();
    radar_vel_noise_   = get_parameter("radar.vel_noise").as_double();

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
    gnss_lever_arm_explicit_ = !gnss_lever_arm_.is_zero();
    config.gnss.apply_lever_arm_pre_heading =
      get_parameter("gnss.apply_lever_arm_pre_heading").as_bool();
    if (config.gnss.apply_lever_arm_pre_heading) {
      RCLCPP_INFO(get_logger(),
        "GNSS lever arm will be applied pre-heading-validation "
        "(gnss.apply_lever_arm_pre_heading=true)");
    }

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
    config.gnss_max_speed         = get_parameter("gnss.max_speed").as_double();
    config.gnss_max_speed_margin  = get_parameter("gnss.max_speed_margin").as_double();
    config.outlier_threshold_imu  = get_parameter("outlier_threshold_imu").as_double();
    config.outlier_threshold_enc   = get_parameter("outlier_threshold_enc").as_double();
    config.outlier_threshold_hdg   = get_parameter("outlier_threshold_hdg").as_double();
    config.outlier_threshold_vslam = get_parameter("outlier_threshold_vslam").as_double();

    vslam_topic_          = get_parameter("vslam.topic").as_string();
    vslam_frame_override_ = get_parameter("vslam.frame_id").as_string();
    config.vslam.position_noise    = get_parameter("vslam.position_noise").as_double();
    config.vslam.orientation_noise = get_parameter("vslam.orientation_noise").as_double();
    vslam_reinit_n_       = get_parameter("vslam.reinit_n").as_int();

    config.gnss_coast_n               = get_parameter("gnss.coast_n").as_int();
    config.gnss_coast_min_gap_s       = get_parameter("gnss.coast_min_gap_s").as_double();
    config.gnss_coast_q_factor        = get_parameter("gnss.coast_q_factor").as_double();
    config.gnss_coast_timeout_s       = get_parameter("gnss.coast_timeout_s").as_double();
    config.gnss_coast_q_bias_factor   = get_parameter("gnss.coast_q_bias_factor").as_double();
    config.gnss_coast_imu_wz_scale    = get_parameter("gnss.coast_imu_wz_scale").as_double();
    config.gnss_recovery_rejection_n  = get_parameter("gnss.recovery_rejection_n").as_int();
    config.gnss_p_inflate_sigma       = get_parameter("gnss.p_inflate_sigma").as_double();
    config.gnss_recovery_timeout_s    = get_parameter("gnss.recovery_timeout_s").as_double();
    config.gps_track_heading_enabled       = get_parameter("gnss.track_heading_enabled").as_bool();
    config.gps_track_heading_min_dist      = get_parameter("gnss.track_heading_min_dist").as_double();
    config.gps_track_heading_max_sigma     = get_parameter("gnss.track_heading_max_sigma").as_double();
    config.gps_track_heading_min_speed     = get_parameter("gnss.track_heading_min_speed").as_double();
    config.gps_track_heading_max_yaw_rate  = get_parameter("gnss.track_heading_max_yaw_rate").as_double();
    config.gnss_lever_arm_max_heading_sigma_deg =
      get_parameter("gnss.lever_arm_max_heading_sigma_deg").as_double();

    config.adaptive_imu               = get_parameter("adaptive.imu").as_bool();
    config.adaptive_encoder           = get_parameter("adaptive.encoder").as_bool();
    config.adaptive_gnss              = get_parameter("adaptive.gnss").as_bool();
    config.adaptive_ground_constraint = get_parameter("adaptive.ground_constraint").as_bool();
    config.adaptive_window            = get_parameter("adaptive.window").as_int();
    config.adaptive_alpha             = get_parameter("adaptive.alpha").as_double();

    config.ukf.q_position   = get_parameter("ukf.q_position").as_double();
    config.ukf.q_orientation  = get_parameter("ukf.q_orientation").as_double();
    if (config.ukf.q_orientation > 1e-3) {
      RCLCPP_ERROR(get_logger(),
        "ukf.q_orientation=%.2e is too large: quaternion math will corrupt at IMU rates. "
        "Set to 1.0e-9 or remove the line from your config (default is 1.0e-9).",
        config.ukf.q_orientation);
      return CallbackReturn::FAILURE;
    }
    config.ukf.q_velocity     = get_parameter("ukf.q_velocity").as_double();
    config.ukf.q_angular_vel  = get_parameter("ukf.q_angular_vel").as_double();
    config.ukf.q_acceleration = get_parameter("ukf.q_acceleration").as_double();
    config.ukf.q_gyro_bias          = get_parameter("ukf.q_gyro_bias").as_double();
    config.ukf.q_accel_bias         = get_parameter("ukf.q_accel_bias").as_double();
    config.ukf.q_encoder_wz_bias    = get_parameter("ukf.q_encoder_wz_bias").as_double();

    zupt_enabled_            = get_parameter("zupt.enabled").as_bool();
    zupt_velocity_threshold_ = get_parameter("zupt.velocity_threshold").as_double();
    zupt_angular_threshold_  = get_parameter("zupt.angular_threshold").as_double();
    zupt_noise_sigma_        = get_parameter("zupt.noise_sigma").as_double();

    mag_enabled_ = get_parameter("magnetometer.enabled").as_bool();
    mag_topic_   = get_parameter("magnetometer.topic").as_string();
    config.mag.noise_rad      = get_parameter("magnetometer.noise_rad").as_double();
    config.mag.chi2_threshold = get_parameter("magnetometer.chi2_threshold").as_double();
    config.mag.declination_rad = get_parameter("magnetometer.declination_rad").as_double();
    config.mag.field_strength  = get_parameter("magnetometer.field_strength").as_double();
    config.mag.field_tolerance = get_parameter("magnetometer.field_tolerance").as_double();

    {
      auto hi = get_parameter("magnetometer.hard_iron").as_double_array();
      if (hi.size() == 3) {
        config.mag.hard_iron = Eigen::Vector3d(hi[0], hi[1], hi[2]);
      }
      auto si = get_parameter("magnetometer.soft_iron").as_double_array();
      if (si.size() == 9) {
        config.mag.soft_iron << si[0], si[1], si[2],
                                si[3], si[4], si[5],
                                si[6], si[7], si[8];
      }
    }

    if (mag_enabled_) {
      RCLCPP_INFO(get_logger(),
        "Magnetometer heading fusion enabled on topic: %s "
        "(noise=%.3f rad, declination=%.3f rad)",
        mag_topic_.c_str(),
        config.mag.noise_rad,
        config.mag.declination_rad);
    }

    config.encoder_nhc_vy_sigma        = get_parameter("encoder.nhc_vy_sigma").as_double();
    config.ground_constraint_vz_sigma  = get_parameter("ground_constraint.vz_sigma").as_double();
    config.ground_constraint_az_sigma  = get_parameter("ground_constraint.az_sigma").as_double();
    config.ground_z_position_sigma     = get_parameter("ground_constraint.z_position_sigma").as_double();

    init_window_duration_    = get_parameter("init.stationary_window").as_double();
    wait_for_all_sensors_    = get_parameter("init.wait_for_all_sensors").as_bool();
    sensor_wait_timeout_     = get_parameter("init.sensor_wait_timeout").as_double();
    checkpoint_path_         = get_parameter("replay.checkpoint_path").as_string();

    const std::string motion_model_name =
      get_parameter("motion_model").as_string();
    const double wheelbase =
      get_parameter("motion_model_params.wheelbase").as_double();

    if (!motion_model_name.empty() &&
        motion_model_name != "ConstantVelocityAcceleration" &&
        motion_model_name != "CVA") {
      try {
        config.motion_model = fusioncore::create_motion_model(
          motion_model_name, {{"wheelbase", wheelbase}});
        RCLCPP_INFO(get_logger(), "Motion model: %s", motion_model_name.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        return CallbackReturn::FAILURE;
      }
    }

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

    autostart_ = get_parameter("autostart").as_bool();
    if (autostart_) {
      autostart_timer_ = create_wall_timer(200ms, [this]() {
        autostart_timer_->cancel();
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      });
      RCLCPP_INFO(get_logger(), "Autostart enabled: activating in 200ms.");
    }

    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Activate ───────────────────────────────────────────────────

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating FusionCore...");
    validate_transforms();

    // Do NOT initialize the filter here with now().seconds().
    // With use_sim_time:true, now() may return 0 if /clock hasn't started yet.
    // Initializing at t=0 then receiving the first IMU at sim t=T causes a
    // T-second dead prediction step that can blow up the state covariance.
    // Instead, initialize lazily on the first IMU message using its timestamp.
    pending_init_ = true;

    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_cb_group_;
    // BEST_EFFORT QoS: compatible with both BEST_EFFORT and RELIABLE publishers.
    // Most hardware drivers (ublox, Septentrio, Microstrain, etc.) publish sensor
    // data as BEST_EFFORT. Using RELIABLE here silently drops all messages from
    // those drivers because the QoS policies are incompatible in ROS 2.
    auto sensor_qos = rclcpp::SensorDataQoS();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        imu_callback(msg);
      }, sensor_opts);
    RCLCPP_INFO(get_logger(), "IMU topic: %s", imu_topic_.c_str());

    if (!imu2_topic_.empty()) {
      imu2_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu2_topic_, sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          imu2_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Second IMU enabled on topic: %s", imu2_topic_.c_str());
    }

    encoder_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      encoder_topic_, sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        encoder_callback(msg);
      }, sensor_opts);
    RCLCPP_INFO(get_logger(), "Encoder topic: %s", encoder_topic_.c_str());

    // Second encoder-twist source (e.g. KISS-ICP LiDAR odometry). Created
    // lazily only when encoder2.topic is non-empty to keep the default
    // behavior identical to a single-encoder setup.
    if (!encoder2_topic_.empty()) {
      encoder2_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        encoder2_topic_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          encoder2_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Second encoder-twist source enabled on topic: %s", encoder2_topic_.c_str());
    }

    if (!vslam_topic_.empty()) {
      vslam_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        vslam_topic_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          vslam_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "VSLAM pose fusion enabled on topic: %s", vslam_topic_.c_str());
    }

    if (!gnss_vel_topic_.empty()) {
      gnss_vel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        gnss_vel_topic_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gnss_vel_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "GPS velocity fusion enabled on topic: %s", gnss_vel_topic_.c_str());
    }

    if (!radar_vel_topic_.empty()) {
      radar_vel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        radar_vel_topic_, sensor_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          radar_vel_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Radar Doppler velocity fusion enabled on topic: %s", radar_vel_topic_.c_str());
    }

    if (use_gps_fix_) {
      gps_fix_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
        gnss_fix_topic_, sensor_qos,
        [this](const gps_msgs::msg::GPSFix::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gps_fix_callback(msg, 0);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "GNSS topic: %s (gps_msgs/GPSFix, RTK_FLOAT capable)", gnss_fix_topic_.c_str());
    } else {
      gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_fix_topic_, sensor_qos,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gnss_callback(msg, 0);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "GNSS topic: %s (sensor_msgs/NavSatFix)", gnss_fix_topic_.c_str());
    }

    // compass_msgs/Azimuth heading: optional, preferred over sensor_msgs/Imu
    if (!azimuth_topic_.empty()) {
      azimuth_sub_ = create_subscription<compass_msgs::msg::Azimuth>(
        azimuth_topic_, sensor_qos,
        [this](const compass_msgs::msg::Azimuth::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          azimuth_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "compass_msgs/Azimuth heading enabled on topic: %s", azimuth_topic_.c_str());
    }

    // Raw magnetometer heading: optional, enabled via magnetometer.enabled
    if (mag_enabled_) {
      mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        mag_topic_, sensor_qos,
        [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          mag_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Magnetometer subscribed on topic: %s", mag_topic_.c_str());
    }

    // Second GNSS receiver: optional
    if (!gnss2_topic_.empty()) {
      gnss2_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss2_topic_, sensor_qos,
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
        heading_topic_, sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(fc_mutex_);
          gnss_heading_callback(msg);
        }, sensor_opts);
      RCLCPP_INFO(get_logger(),
        "Subscribed to dual antenna heading: %s", heading_topic_.c_str());
    }

    odom_pub_          = create_publisher<nav_msgs::msg::Odometry>("/fusion/odom", 100);
    pose_pub_          = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fusion/pose", 100);
    diag_pub_          = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    gnss_status_pub_   = create_publisher<fusioncore_ros::msg::GnssStatus>("/fusion/debug/gnss_status", 10);
    filter_health_pub_ = create_publisher<fusioncore_ros::msg::FilterHealth>("/fusion/debug/filter_health", 10);

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

    // fromLL service: converts GPS lat/lon/alt to map frame x/y/z.
    // Drop-in replacement for robot_localization's /fromLL service used by
    // nav2_waypoint_follower for GPS waypoint navigation.
    from_ll_srv_ = create_service<fusioncore_ros::srv::FromLL>(
      "/fromLL",
      [this](
        const fusioncore_ros::srv::FromLL::Request::SharedPtr request,
        fusioncore_ros::srv::FromLL::Response::SharedPtr response)
      {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        if (!gnss_ref_set_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "fromLL called before GPS reference is set. "
            "Wait for a GPS fix before requesting waypoint conversion.");
          response->map_point.x = 0.0;
          response->map_point.y = 0.0;
          response->map_point.z = 0.0;
          return;
        }
        fusioncore::sensors::LLAPoint lla;
        lla.lat_rad = request->ll_point.latitude  * M_PI / 180.0;
        lla.lon_rad = request->ll_point.longitude * M_PI / 180.0;
        lla.alt_m   = request->ll_point.altitude;
        fusioncore::sensors::ECEFPoint ecef;
        gnss_to_output(lla, ecef);
        Eigen::Vector3d enu;
        if (convert_to_enu_at_reference_) {
          enu = fusioncore::sensors::ecef_to_enu(ecef, gnss_ref_ecef_, gnss_ref_lla_);
        } else {
          enu = Eigen::Vector3d(ecef.x - gnss_ref_ecef_.x,
                                ecef.y - gnss_ref_ecef_.y,
                                ecef.z - gnss_ref_ecef_.z);
        }
        response->map_point.x = enu[0];
        response->map_point.y = enu[1];
        response->map_point.z = enu[2];
      });

    // Checkpoint services: save/load full filter state for deterministic replay.
    save_checkpoint_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_checkpoint",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        if (!fc_->is_initialized()) {
          response->success = false;
          response->message = "Filter not initialized.";
          return;
        }
        std::ofstream f(checkpoint_path_);
        if (!f) {
          response->success = false;
          response->message = "Cannot open: " + checkpoint_path_;
          return;
        }
        const auto& s = fc_->get_state();
        f << "t=" << last_imu_time_ << "\n";
        f << "x=";
        for (int i = 0; i < fusioncore::STATE_DIM; ++i)
          f << s.x[i] << (i + 1 < fusioncore::STATE_DIM ? " " : "\n");
        f << "P=";
        for (int r = 0; r < fusioncore::STATE_DIM; ++r)
          for (int c = 0; c < fusioncore::STATE_DIM; ++c)
            f << s.P(r, c) <<
              (r == fusioncore::STATE_DIM - 1 && c == fusioncore::STATE_DIM - 1 ? "\n" : " ");
        response->success = true;
        response->message = "Saved to " + checkpoint_path_;
        RCLCPP_INFO(get_logger(), "State checkpoint saved to %s at t=%.3f",
          checkpoint_path_.c_str(), last_imu_time_);
      });

    load_checkpoint_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_checkpoint",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::lock_guard<std::mutex> lock(fc_mutex_);
        std::ifstream f(checkpoint_path_);
        if (!f) {
          response->success = false;
          response->message = "Cannot open: " + checkpoint_path_;
          return;
        }
        fusioncore::State restored;
        double t = 0.0;
        std::string line;
        while (std::getline(f, line)) {
          if (line.substr(0, 2) == "t=") {
            t = std::stod(line.substr(2));
          } else if (line.substr(0, 2) == "x=") {
            std::istringstream ss(line.substr(2));
            for (int i = 0; i < fusioncore::STATE_DIM; ++i) ss >> restored.x[i];
          } else if (line.substr(0, 2) == "P=") {
            std::istringstream ss(line.substr(2));
            for (int r = 0; r < fusioncore::STATE_DIM; ++r)
              for (int c = 0; c < fusioncore::STATE_DIM; ++c)
                ss >> restored.P(r, c);
          }
        }
        fc_->init(restored, t);
        response->success = true;
        response->message = "Loaded from " + checkpoint_path_;
        RCLCPP_INFO(get_logger(), "State checkpoint loaded from %s at t=%.3f",
          checkpoint_path_.c_str(), t);
      });

    // Sensor wait: populate the expected set based on configured sources.
    if (wait_for_all_sensors_) {
      sensors_expected_.clear();
      sensors_received_.clear();
      sensor_wait_done_ = false;
      sensors_expected_.insert("IMU");
      sensors_expected_.insert("Encoder");
      if (reference_use_first_fix_)        sensors_expected_.insert("GNSS");
      if (!imu2_topic_.empty())            sensors_expected_.insert("IMU2");
      if (!encoder2_topic_.empty())        sensors_expected_.insert("Encoder2");
      if (!vslam_topic_.empty())           sensors_expected_.insert("VSLAM");
      if (!gnss_vel_topic_.empty())        sensors_expected_.insert("GPSVel");
      if (!radar_vel_topic_.empty())       sensors_expected_.insert("RadarVel");
      if (!heading_topic_.empty() ||
          !azimuth_topic_.empty())         sensors_expected_.insert("Heading");
      if (!gnss2_topic_.empty())           sensors_expected_.insert("GNSS2");
      activate_time_ = this->now().seconds();
      RCLCPP_INFO(get_logger(), "Waiting for %zu sensor(s) before starting filter.",
        sensors_expected_.size());
    }

    RCLCPP_INFO(get_logger(), "FusionCore active. Listening for sensors.");
    return CallbackReturn::SUCCESS;
  }

  // ─── Lifecycle: Deactivate ─────────────────────────────────────────────────

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    imu_sub_.reset();
    imu2_sub_.reset();
    encoder_sub_.reset();
    encoder2_sub_.reset();
    vslam_sub_.reset();
    vslam_origin_set_          = false;
    vslam_consecutive_rejects_ = 0;
    gnss_vel_sub_.reset();
    radar_vel_sub_.reset();
    gnss_sub_.reset();
    gps_fix_sub_.reset();
    gnss2_sub_.reset();
    gnss_heading_sub_.reset();
    azimuth_sub_.reset();
    mag_sub_.reset();
    publish_timer_.reset();
    diag_timer_.reset();
    reset_srv_.reset();
    from_ll_srv_.reset();
    save_checkpoint_srv_.reset();
    load_checkpoint_srv_.reset();
    sensors_expected_.clear();
    sensors_received_.clear();
    sensor_wait_done_ = false;
    odom_pub_.reset();
    pose_pub_.reset();
    diag_pub_.reset();
    gnss_status_pub_.reset();
    filter_health_pub_.reset();
    deinit_proj();
    RCLCPP_INFO(get_logger(), "FusionCore deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    autostart_timer_.reset();
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

    // Use configured IMU frame instead of hardcoded "imu_link"
    std::string imu_tf_frame = imu_frame_override_.empty() ? "imu_link" : imu_frame_override_;

    // Check common sensor transforms.
    // Note: base_frame_ → odom_frame_ is intentionally NOT checked here.
    // FusionCore itself publishes that TF, so it cannot exist before the filter
    // starts: checking it would always produce a misleading MISSING warning.
    std::vector<std::pair<std::string,std::string>> to_check = {
      {imu_tf_frame, base_frame_},
    };

    for (const auto& [from, to] : to_check) {
      if (check_transform(from, to)) {
        RCLCPP_INFO(get_logger(), "  [OK]      %s -> %s", from.c_str(), to.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "  [MISSING] %s -> %s  Fix: ros2 run tf2_ros static_transform_publisher --frame-id %s --child-frame-id %s",
          from.c_str(), to.c_str(), to.c_str(), from.c_str());
        all_ok = false;
      }
    }

    // Check GNSS frame if primary lever arm is configured
    if (!gnss_lever_arm_.is_zero()) {
      if (check_transform("gnss_link", base_frame_)) {
        RCLCPP_INFO(get_logger(), "  [OK]      gnss_link -> %s", base_frame_.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
          "  [MISSING] gnss_link -> %s  Fix: ros2 run tf2_ros static_transform_publisher --x %.3f --y %.3f --z %.3f --frame-id %s --child-frame-id gnss_link",
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
          "  [MISSING] gnss2_link -> %s  Fix: ros2 run tf2_ros static_transform_publisher --x %.3f --y %.3f --z %.3f --frame-id %s --child-frame-id gnss2_link",
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

  // Helper: mark a sensor as received for the sensor-wait feature.
  void mark_sensor_received(const std::string& name) {
    if (wait_for_all_sensors_ && sensors_expected_.count(name))
      sensors_received_.insert(name);
  }

  // Helper: format a set of sensor names as "A, B, C" for log messages.
  static std::string format_sensor_set(const std::set<std::string>& s) {
    std::string out;
    for (const auto& n : s) { if (!out.empty()) out += ", "; out += n; }
    return out;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Lazy init: initialize the filter on the first IMU message using the
    // message timestamp. This avoids a large dead prediction step when
    // use_sim_time:true and /clock hasn't started before on_activate().
    last_imu_time_ = t;
    mark_sensor_received("IMU");

    if (pending_init_) {
      // Sensor wait gate: hold initialization until all expected sensors checked in.
      if (wait_for_all_sensors_ && !sensor_wait_done_) {
        if (sensors_received_ != sensors_expected_) {
          double elapsed = this->now().seconds() - activate_time_;
          if (elapsed < sensor_wait_timeout_) {
            std::set<std::string> missing;
            for (const auto& s : sensors_expected_)
              if (!sensors_received_.count(s)) missing.insert(s);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
              "Waiting for sensors (%.1fs / %.1fs): missing [%s]",
              elapsed, sensor_wait_timeout_, format_sensor_set(missing).c_str());
            return;
          }
          std::set<std::string> missing;
          for (const auto& s : sensors_expected_)
            if (!sensors_received_.count(s)) missing.insert(s);
          RCLCPP_WARN(get_logger(),
            "Sensor wait timed out after %.1fs. Missing: [%s]. Starting anyway.",
            sensor_wait_timeout_, format_sensor_set(missing).c_str());
        } else {
          RCLCPP_INFO(get_logger(),
            "All %zu configured sensors ready. Starting filter.",
            sensors_expected_.size());
        }
        sensor_wait_done_ = true;
      }

      if (init_window_duration_ <= 0.0) {
        fusioncore::State initial;
        initial.P = fusioncore::StateMatrix::Identity() * 0.1;
        initial.P(0,0) = 1000.0;
        initial.P(1,1) = 1000.0;
        initial.P(2,2) = 1000.0;
        fc_->init(initial, t);
        pending_init_ = false;
        RCLCPP_INFO(get_logger(), "Filter initialized at t=%.3f (first IMU)", t);
      } else {
        // Static bias window: collect IMU samples before starting the filter.
        if (!init_window_collecting_) {
          init_window_collecting_ = true;
          // Use message timestamp when valid (non-zero): makes the window deterministic
          // during bag replay with use_sim_time:true. Fall back to wall clock only for
          // drivers that publish zero-stamped messages (the original bug fix path).
          init_window_start_is_msg_time_ = (t > 0.0);
          init_window_start_ = init_window_start_is_msg_time_
            ? t : this->now().seconds();
          init_window_aborted_    = false;
          init_win_n_             = 0;
          init_win_wx_ = init_win_wy_ = init_win_wz_ = 0.0;
          init_win_ax_ = init_win_ay_ = init_win_az_ = 0.0;
          init_win_qw_ = init_win_qx_ = init_win_qy_ = init_win_qz_ = 0.0;
          init_win_orient_n_ = 0;
          RCLCPP_INFO(get_logger(),
            "Collecting %.1fs bias window before init...", init_window_duration_);
        }

        // Accumulate gyro and accel
        init_win_wx_ += msg->angular_velocity.x;
        init_win_wy_ += msg->angular_velocity.y;
        init_win_wz_ += msg->angular_velocity.z;
        init_win_ax_ += msg->linear_acceleration.x;
        init_win_ay_ += msg->linear_acceleration.y;
        init_win_az_ += msg->linear_acceleration.z;
        ++init_win_n_;

        // Accumulate orientation if the driver provides one.
        //
        // This has to agree with fuse_imu_orientation_if_valid(). Per the
        // sensor_msgs/Imu spec only a NEGATIVE covariance means "no orientation
        // data"; all-zeros means "covariance unknown", and plenty of drivers
        // publish exactly that alongside a perfectly good quaternion.
        //
        // Requiring a positive covariance here silently discarded orientation
        // from 9-axis IMUs whose driver leaves the covariance at zero. That
        // matters more than it sounds: without an orientation the accel bias
        // below is never initialised, so whatever slice of gravity the IMU's
        // mounting tilt produces stays in the acceleration channel, and a
        // constant acceleration error double-integrates straight into position.
        const auto& ocov = msg->orientation_covariance;
        const auto& q    = msg->orientation;
        const double q_norm_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        // A driver with nothing to report may leave the quaternion all-zero,
        // which is not a rotation. Anything normalised passes.
        const bool has_orient = (ocov[0] >= 0.0) && (q_norm_sq > 0.5);
        if (has_orient) {
          init_win_qw_ += q.w;
          init_win_qx_ += q.x;
          init_win_qy_ += q.y;
          init_win_qz_ += q.z;
          ++init_win_orient_n_;
        }

        // Window complete? Use same time source that was chosen at window start.
        double window_elapsed = init_window_start_is_msg_time_
          ? (t - init_window_start_)
          : (this->now().seconds() - init_window_start_);
        if (window_elapsed >= init_window_duration_) {
          fusioncore::State initial;
          initial.P = fusioncore::StateMatrix::Identity() * 0.1;
          initial.P(0,0) = 1000.0;
          initial.P(1,1) = 1000.0;
          initial.P(2,2) = 1000.0;

          if (!init_window_aborted_ && init_win_n_ > 0) {
            double n = static_cast<double>(init_win_n_);
            initial.x[fusioncore::B_GX] = init_win_wx_ / n;
            initial.x[fusioncore::B_GY] = init_win_wy_ / n;
            initial.x[fusioncore::B_GZ] = init_win_wz_ / n;

            if (init_win_orient_n_ > 0) {
              double on = static_cast<double>(init_win_orient_n_);
              double qw = init_win_qw_ / on, qx = init_win_qx_ / on;
              double qy = init_win_qy_ / on, qz = init_win_qz_ / on;
              double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
              qw /= norm; qx /= norm; qy /= norm; qz /= norm;
              const double g = 9.80665;
              double gx = 2.0*(qx*qz - qy*qw)*g;
              double gy = 2.0*(qy*qz + qx*qw)*g;
              double gz = (1.0 - 2.0*(qx*qx + qy*qy))*g;
              initial.x[fusioncore::B_AX] = init_win_ax_ / n - gx;
              initial.x[fusioncore::B_AY] = init_win_ay_ / n - gy;
              initial.x[fusioncore::B_AZ] = init_win_az_ / n - gz;
              RCLCPP_INFO(get_logger(),
                "Bias window done: gyro=[%.4f,%.4f,%.4f] accel=[%.4f,%.4f,%.4f] rad/s, m/s²",
                initial.x[fusioncore::B_GX], initial.x[fusioncore::B_GY], initial.x[fusioncore::B_GZ],
                initial.x[fusioncore::B_AX], initial.x[fusioncore::B_AY], initial.x[fusioncore::B_AZ]);
            } else {
              // No usable orientation, so the accel bias stays at zero and the
              // filter starts level. That is fine for a 6-axis IMU mounted flat
              // (the UKF observes gravity and converges), but if the IMU is
              // tilted the uncompensated gravity component behaves like a
              // constant acceleration and integrates into position. Warn rather
              // than log quietly: on a 9-axis IMU this almost always means the
              // driver is not publishing what FusionCore expects.
              RCLCPP_WARN(get_logger(),
                "Bias window done (gyro only, no orientation): gyro=[%.4f,%.4f,%.4f]. "
                "Accel bias left at zero. If your IMU does report orientation, check that "
                "orientation_covariance[0] is not negative and the quaternion is non-zero, "
                "otherwise a tilted IMU mount will leak gravity into position.",
                initial.x[fusioncore::B_GX], initial.x[fusioncore::B_GY], initial.x[fusioncore::B_GZ]);
            }
          } else {
            RCLCPP_WARN(get_logger(),
              "Bias window aborted (robot moved). Starting with zero bias.");
          }

          fc_->init(initial, t);
          pending_init_         = false;
          init_window_collecting_ = false;
          RCLCPP_INFO(get_logger(), "Filter initialized at t=%.3f", t);
        }
        return;  // Don't process this IMU message through the filter yet
      }
    }

    if (!fc_->is_initialized()) return;

    std::string imu_frame = imu_frame_override_.empty()
      ? (msg->header.frame_id.empty() ? "imu_link" : msg->header.frame_id)
      : imu_frame_override_;

    // On the first IMU message, confirm the resolved frame matches what
    // validate_transforms assumed. If they differ, the TF lookup will fail
    // silently and orientation corrections will be skipped.
    if (imu_frame_resolved_.empty()) {
      imu_frame_resolved_ = imu_frame;
      std::string validated_frame = imu_frame_override_.empty() ? "imu_link" : imu_frame_override_;
      if (imu_frame_resolved_ != validated_frame) {
        RCLCPP_WARN(get_logger(),
          "IMU frame mismatch: TF validation checked '%s' but first message has frame_id '%s'. "
          "Set imu.frame_id: \"%s\" in your config to fix the startup validation warning.",
          validated_frame.c_str(), imu_frame_resolved_.c_str(), imu_frame_resolved_.c_str());
      } else {
        RCLCPP_DEBUG(get_logger(), "IMU TF frame confirmed: %s", imu_frame_resolved_.c_str());
      }
    }

    // One-shot auto-resolve of the IMU lever arm from TF. Only runs when
    // the user did NOT set imu.lever_arm_x/y/z explicitly (all zero).
    // Picks up the translation from base_frame -> imu_frame published by
    // robot_state_publisher from the URDF.
    if (!imu_lever_arm_explicit_ && !imu_lever_arm_tf_resolved_ &&
        imu_frame != base_frame_) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          base_frame_, imu_frame, tf2::TimePointZero,
          tf2::durationFromSec(0.2));
        fusioncore::sensors::ImuLeverArm la;
        la.x = tf.transform.translation.x;
        la.y = tf.transform.translation.y;
        la.z = tf.transform.translation.z;
        if (!la.is_zero()) {
          fc_->set_imu_lever_arm(la);
          RCLCPP_INFO(get_logger(),
            "IMU lever arm auto-resolved from TF %s -> %s: x=%.3f y=%.3f z=%.3f m",
            base_frame_.c_str(), imu_frame.c_str(), la.x, la.y, la.z);
        } else {
          RCLCPP_INFO(get_logger(),
            "IMU lever arm auto-resolved to zero (IMU is at base_frame origin)");
        }
        imu_lever_arm_tf_resolved_ = true;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "IMU lever arm auto-resolve failed (%s -> %s): %s. "
          "Leaving lever arm at zero; set imu.lever_arm_x/y/z explicitly to override.",
          base_frame_.c_str(), imu_frame.c_str(), ex.what());
      }
    }

    if (imu_frame == base_frame_) {
      double ax = msg->linear_acceleration.x;
      double ay = msg->linear_acceleration.y;
      double az = msg->linear_acceleration.z;
      if (imu_remove_gravity_ && fc_->is_initialized()) {
        // IMU driver already removed gravity → add specific force back so the
        // filter measurement model (which expects specific force) is consistent.
        tf2::Vector3 g_base = gravity_in_body_frame();
        ax += g_base.x(); ay += g_base.y(); az += g_base.z();
      }
      fc_->update_imu(t,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z,
        ax, ay, az);
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
        " --frame-id %s --child-frame-id %s",
        imu_frame.c_str(), base_frame_.c_str(), ex.what(),
        base_frame_.c_str(), imu_frame.c_str());
      double ax = msg->linear_acceleration.x;
      double ay = msg->linear_acceleration.y;
      double az = msg->linear_acceleration.z;
      if (imu_remove_gravity_ && fc_->is_initialized()) {
        tf2::Vector3 g_base = gravity_in_body_frame();
        ax += g_base.x(); ay += g_base.y(); az += g_base.z();
      }
      fc_->update_imu(t,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z,
        ax, ay, az);
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

    if (imu_remove_gravity_ && fc_->is_initialized()) {
      tf2::Vector3 g_base = gravity_in_body_frame();
      a_base += g_base;
    }

    fc_->update_imu(t,
      w_base.x(), w_base.y(), w_base.z(),
      a_base.x(), a_base.y(), a_base.z());
    // Fix 11: pass the rotation quaternion so orientation is also transformed
    fuse_imu_orientation_if_valid(t, msg, q);
  }

  // Second IMU callback. Mirrors imu_callback but skips filter initialization
  // (only the primary IMU drives init). Treats the second sensor as an
  // independent measurement of the same state; both update_imu() calls are
  // valid because they fuse independent noise realizations.
  void imu2_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    mark_sensor_received("IMU2");
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    std::string imu_frame = imu2_frame_override_.empty()
      ? (msg->header.frame_id.empty() ? "imu_link" : msg->header.frame_id)
      : imu2_frame_override_;

    if (imu_frame == base_frame_) {
      double ax = msg->linear_acceleration.x;
      double ay = msg->linear_acceleration.y;
      double az = msg->linear_acceleration.z;
      if (imu2_remove_gravity_) {
        tf2::Vector3 g_base = gravity_in_body_frame();
        ax += g_base.x(); ay += g_base.y(); az += g_base.z();
      }
      fc_->update_imu(t,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        ax, ay, az);
      fuse_imu_orientation_if_valid(t, msg, std::nullopt);
      return;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, imu_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot transform IMU2 from %s to %s: %s"
        " -- Fix: ros2 run tf2_ros static_transform_publisher"
        " --frame-id %s --child-frame-id %s",
        imu_frame.c_str(), base_frame_.c_str(), ex.what(),
        base_frame_.c_str(), imu_frame.c_str());
      double ax = msg->linear_acceleration.x;
      double ay = msg->linear_acceleration.y;
      double az = msg->linear_acceleration.z;
      if (imu2_remove_gravity_) {
        tf2::Vector3 g_base = gravity_in_body_frame();
        ax += g_base.x(); ay += g_base.y(); az += g_base.z();
      }
      fc_->update_imu(t,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        ax, ay, az);
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

    if (imu2_remove_gravity_) {
      tf2::Vector3 g_base = gravity_in_body_frame();
      a_base += g_base;
    }

    fc_->update_imu(t,
      w_base.x(), w_base.y(), w_base.z(),
      a_base.x(), a_base.y(), a_base.z());
    fuse_imu_orientation_if_valid(t, msg, q);
  }

  // Returns the specific-force gravity contribution in body frame.
  // For an upright ENU robot this is [0, 0, +9.81].
  // Use: add this to a "true acceleration" reading to recover specific force,
  // which is what update_imu() expects.
  tf2::Vector3 gravity_in_body_frame()
  {
    const fusioncore::State& s = fc_->get_state();
    // tf2::Quaternion(x,y,z,w): note: NOT (w,x,y,z)
    tf2::Quaternion q_body(s.x[fusioncore::QX], s.x[fusioncore::QY],
                           s.x[fusioncore::QZ], s.x[fusioncore::QW]);
    // In ENU world frame the apparent gravity in a stationary IMU = [0, 0, +9.80665].
    // Rotate from world to body using the inverse quaternion (q maps body→world).
    tf2::Vector3 g_world(0.0, 0.0, 9.80665);
    return tf2::quatRotate(q_body.inverse(), g_world);
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

    // All zeros means "unknown covariance": Gazebo's default IMU plugin.
    // Don't skip: fuse using fallback covariance so Gazebo robots work without extra config.
    // update_imu_orientation() already falls back to config defaults when cov is zero.

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
    mark_sensor_received("Encoder");
    // If collecting the bias window, abort it if the robot moves.
    if (init_window_collecting_) {
      double speed = std::abs(msg->twist.twist.linear.x);
      double wz    = std::abs(msg->twist.twist.angular.z);
      if (speed > zupt_velocity_threshold_ || wz > zupt_angular_threshold_) {
        init_window_aborted_ = true;
      }
    }

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

    // Auto-detect holonomic (mecanum/omnidirectional) robots.
    // The outlier gate cannot distinguish "large VY innovation from a wrong NHC"
    // from "genuine encoder spike": it rejects valid lateral motion and leaves
    // the filter uncorrected during every sideways move.
    // Detection: watch the raw encoder VY. If it is consistently non-zero,
    // the robot genuinely moves laterally and the VY=0 NHC must not apply.
    // This runs before update_encoder so var_vy is correct before the gate fires.
    if (nhc_auto_detect_) {
      if (std::abs(vy) > kNhcDetectVyThreshold_) {
        ++nhc_nonzero_vy_count_;
        if (!nhc_holonomic_detected_ && nhc_nonzero_vy_count_ >= kNhcDetectN_) {
          nhc_holonomic_detected_ = true;
          RCLCPP_INFO(get_logger(),
            "NHC auto-detect: lateral motion detected (VY=%.3f m/s). "
            "Disabling VY=0 constraint. Robot identified as holonomic.", vy);
        }
      } else {
        nhc_nonzero_vy_count_ = 0;
      }
    }

    // When holonomic and no message covariance for VY: inject VX noise as the
    // VY variance so the encoder update passes the outlier gate. Without this,
    // a 0.5 m/s lateral move with nhc_vy_sigma=0.05 is a 10-sigma deviation:
    // the gate rejects the whole update and position goes uncorrected.
    if (nhc_holonomic_detected_ && var_vy <= 0.0) {
      var_vy = nhc_vy_auto_noise_ * nhc_vy_auto_noise_;
    }

    fc_->update_encoder(t, vx, vy, wz, var_vx, var_vy, var_wz);

    // Non-holonomic ground constraint: wheeled robots cannot move vertically.
    // Fuses VZ=0 as a pseudo-measurement to prevent altitude drift.
    fc_->update_ground_constraint(t);

    // Zero-velocity update (ZUPT): when the robot is stationary, assert
    // [VX=0, VY=0, WZ=0] with tight noise to suppress IMU drift.
    // Use encoder measurements for detection, NOT the filter's WZ state.
    // The filter's WZ state is inflated by process noise (q_angular_vel = 0.1
    // per step, unscaled by dt) and can exceed the threshold even when the
    // robot is stationary, causing ZUPT to stop firing and yaw to drift.
    if (zupt_enabled_) {
      double speed = std::sqrt(vx*vx + vy*vy);
      if (speed < zupt_velocity_threshold_ && std::abs(wz) < zupt_angular_threshold_) {
        fc_->update_zupt(t, zupt_noise_sigma_);
      }
    }
  }

  // ─── Second encoder-twist callback ────────────────────────────────────────
  // Handles a supplementary twist source (e.g. KISS-ICP LiDAR odometry).
  // Uses the same update_encoder path as the primary wheel encoder.
  // Does NOT drive ground-constraint or ZUPT updates: those remain anchored
  // to the primary wheel encoder so their detection thresholds and rates
  // stay unchanged when encoder2 is enabled.

  void encoder2_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mark_sensor_received("Encoder2");
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Extract per-axis variances from the Odometry twist covariance (6x6, row-major).
    // Indices: vx=0, vy=7, wz=35 (diagonal elements for linear.x, linear.y, angular.z).
    // Fall back to encoder2.vel_noise / encoder2.yaw_noise when the message
    // reports zero or negative variance (e.g. KISS-ICP, RealSense T265).
    const auto& cov = msg->twist.covariance;
    double var_vx = (cov[0]  > 0.0) ? cov[0]  : enc2_vel_noise_ * enc2_vel_noise_;
    double var_vy = (cov[7]  > 0.0) ? cov[7]  : enc2_vel_noise_ * enc2_vel_noise_;
    double var_wz = (cov[35] > 0.0) ? cov[35] : enc2_yaw_noise_ * enc2_yaw_noise_;

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double wz = msg->twist.twist.angular.z;

    fc_->update_encoder(t, vx, vy, wz, var_vx, var_vy, var_wz);
  }

  // ─── VSLAM pose callback ──────────────────────────────────────────────────
  // Accepts nav_msgs/Odometry. Uses pose component only; twist is ignored.
  // Covariance is extracted from pose.covariance (6x6 row-major):
  //   [0,7,14] = position variance (x,y,z), [21,28,35] = orientation variance (r,p,y).
  // Falls back to config noise when covariance is zero or negative.
  //
  // Frame alignment: VSLAM has its own map frame origin (always starts at 0,0,0).
  // The filter's odom frame may have a different origin if the robot moved before
  // VSLAM initialized, or if VSLAM reinitializes after tracking loss.
  // We track a 3D offset (odom_origin - vslam_origin) and apply it to every
  // VSLAM measurement so the filter sees poses in odom frame coordinates.
  // After vslam_reinit_n_ consecutive gate rejections we re-anchor, which handles
  // ORB-SLAM3 reinitializations that produce large discontinuous pose jumps.

  void vslam_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mark_sensor_received("VSLAM");
    if (!fc_->is_initialized()) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // Raw pose in VSLAM map frame
    fusioncore::sensors::VslamPose pose;
    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_z = msg->pose.pose.position.z;

    const auto& q = msg->pose.pose.orientation;
    double raw_roll, raw_pitch, raw_yaw;
    fusioncore::quat_to_euler(q.w, q.x, q.y, q.z, raw_roll, raw_pitch, raw_yaw);

    // Anchor VSLAM map origin to filter's current odom position on first call.
    // With init.wait_for_all_sensors: true and a stationary startup, both frames
    // start at (0,0,0) and the offset is zero. The anchor still handles the general
    // case where VSLAM initializes after the robot has already moved.
    if (!vslam_origin_set_) {
      const auto& s = fc_->get_state();
      vslam_offset_x_ = s.x[fusioncore::X] - raw_x;
      vslam_offset_y_ = s.x[fusioncore::Y] - raw_y;
      vslam_offset_z_ = s.x[fusioncore::Z] - raw_z;
      vslam_origin_set_ = true;
      RCLCPP_INFO(get_logger(),
        "VSLAM: map origin anchored. offset=(%.3f, %.3f, %.3f)",
        vslam_offset_x_, vslam_offset_y_, vslam_offset_z_);
    }

    // Apply offset: translate pose from VSLAM map frame into filter odom frame
    pose.x = raw_x + vslam_offset_x_;
    pose.y = raw_y + vslam_offset_y_;
    pose.z = raw_z + vslam_offset_z_;
    pose.roll  = raw_roll;
    pose.pitch = raw_pitch;
    pose.yaw   = raw_yaw;

    // Extract covariance from pose.covariance (6x6, row-major, [x,y,z,rx,ry,rz]).
    // Diagonal indices: x=0, y=7, z=14, roll=21, pitch=28, yaw=35.
    const auto& cov = msg->pose.covariance;
    constexpr double kMinVarPos    = 1e-4;
    constexpr double kMinVarOrient = 1e-6;

    const double var_x   = cov[0];
    const double var_y   = cov[7];
    const double var_z   = cov[14];
    const double var_r   = cov[21];
    const double var_p   = cov[28];
    const double var_yaw = cov[35];

    if (var_x > 0.0 && var_y > 0.0 && var_z > 0.0) {
      pose.has_position_cov  = true;
      pose.position_cov(0,0) = std::max(var_x, kMinVarPos);
      pose.position_cov(1,1) = std::max(var_y, kMinVarPos);
      pose.position_cov(2,2) = std::max(var_z, kMinVarPos);
    }

    if (var_r > 0.0 && var_p > 0.0 && var_yaw > 0.0) {
      pose.has_orientation_cov  = true;
      pose.orientation_cov(0,0) = std::max(var_r,   kMinVarOrient);
      pose.orientation_cov(1,1) = std::max(var_p,   kMinVarOrient);
      pose.orientation_cov(2,2) = std::max(var_yaw, kMinVarOrient);
    }

    const bool accepted = fc_->update_pose(t, pose);

    if (accepted) {
      vslam_consecutive_rejects_ = 0;
    } else {
      ++vslam_consecutive_rejects_;
      // After vslam_reinit_n_ consecutive gate rejections, VSLAM has almost
      // certainly reinitialized to a new map. Re-anchor to the filter's current
      // position so subsequent measurements are accepted in the new map frame.
      if (vslam_consecutive_rejects_ >= vslam_reinit_n_) {
        const auto& s = fc_->get_state();
        vslam_offset_x_ = s.x[fusioncore::X] - raw_x;
        vslam_offset_y_ = s.x[fusioncore::Y] - raw_y;
        vslam_offset_z_ = s.x[fusioncore::Z] - raw_z;
        vslam_consecutive_rejects_ = 0;
        RCLCPP_WARN(get_logger(),
          "VSLAM: %d consecutive rejections: reinitialization detected. "
          "Re-anchoring map origin. new offset=(%.3f, %.3f, %.3f)",
          vslam_reinit_n_, vslam_offset_x_, vslam_offset_y_, vslam_offset_z_);
      }
    }
  }

  // ─── Radar Doppler velocity callback ─────────────────────────────────────
  // Fuses ego-velocity from a 4D imaging radar as an independent measurement.
  // Velocity is expected in robot body frame: linear.x=forward, linear.y=lateral.
  // A bridge node handles raw Doppler point cloud -> ego-velocity extraction.
  // Works indoors and in all weather (rain, fog, dust) where GPS is unreliable.
  // Angular rate from radar is not reliable; WZ is suppressed via large variance.

  void radar_vel_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mark_sensor_received("RadarVel");
    if (!fc_->is_initialized()) return;

    const double t  = rclcpp::Time(msg->header.stamp).seconds();
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;

    const auto& cov = msg->twist.covariance;
    const double var_vx = (cov[0] > 0.0) ? cov[0] : (radar_vel_noise_ * radar_vel_noise_);
    const double var_vy = (cov[7] > 0.0) ? cov[7] : (radar_vel_noise_ * radar_vel_noise_);

    fc_->update_encoder(t, vx, vy, 0.0, var_vx, var_vy, 1e6);
  }

  // ─── GPS velocity callback ────────────────────────────────────────────────
  // Fuses horizontal GPS velocity as an independent measurement.
  // Message twist is expected in ENU (world) frame: linear.x=east, linear.y=north.
  // Rotated to body frame using the current filter quaternion before fusing,
  // so the innovation is consistent with the wheel encoder measurement model.
  // Angular rate is not available from GPS; WZ is passed with large variance
  // (1e6) so the Kalman gain for WZ is effectively zero.

  void gnss_vel_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mark_sensor_received("GPSVel");
    if (!fc_->is_initialized()) return;

    const double t  = rclcpp::Time(msg->header.stamp).seconds();
    const double ve = msg->twist.twist.linear.x;   // east
    const double vn = msg->twist.twist.linear.y;   // north
    const double vu = msg->twist.twist.linear.z;   // up

    // Rotate ENU -> body:  v_body = R(q)^T * v_world
    const auto& s = fc_->get_state();
    double R[3][3];
    fusioncore::quat_to_rotation_matrix(s.quat_w(), s.quat_x(), s.quat_y(), s.quat_z(), R);
    const double vx = R[0][0]*ve + R[1][0]*vn + R[2][0]*vu;
    const double vy = R[0][1]*ve + R[1][1]*vn + R[2][1]*vu;

    const auto& cov = msg->twist.covariance;
    const double var_vx = (cov[0] > 0.0) ? cov[0] : -1.0;
    const double var_vy = (cov[7] > 0.0) ? cov[7] : -1.0;

    fc_->update_encoder(t, vx, vy, 0.0, var_vx, var_vy, 1e6);
  }

  // ─── GNSS position callback ────────────────────────────────────────────────

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg, int source_id = 0)
  {
    if (source_id == 0) mark_sensor_received("GNSS");
    else                mark_sensor_received("GNSS2");
    if (!fc_->is_initialized()) return;

    if (msg->status.status < 0) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    // One-shot auto-resolve of the GNSS lever arm from TF, primary receiver
    // only. Uses msg->header.frame_id (typically "gps" or "gnss_link")
    // looked up against base_frame_. Only runs when the user did not set
    // gnss.lever_arm_x/y/z explicitly.
    if (source_id == 0 && !gnss_lever_arm_explicit_ && !gnss_lever_arm_tf_resolved_) {
      if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
        try {
          auto tf = tf_buffer_->lookupTransform(
            base_frame_, msg->header.frame_id, tf2::TimePointZero,
            tf2::durationFromSec(0.2));
          gnss_lever_arm_.x = tf.transform.translation.x;
          gnss_lever_arm_.y = tf.transform.translation.y;
          gnss_lever_arm_.z = tf.transform.translation.z;
          if (!gnss_lever_arm_.is_zero()) {
            RCLCPP_INFO(get_logger(),
              "GNSS lever arm auto-resolved from TF %s -> %s: x=%.3f y=%.3f z=%.3f m",
              base_frame_.c_str(), msg->header.frame_id.c_str(),
              gnss_lever_arm_.x, gnss_lever_arm_.y, gnss_lever_arm_.z);
          } else {
            RCLCPP_INFO(get_logger(),
              "GNSS lever arm auto-resolved to zero (antenna at base_frame origin)");
          }
          gnss_lever_arm_tf_resolved_ = true;
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "GNSS lever arm auto-resolve failed (%s -> %s): %s. "
            "Leaving lever arm at zero; set gnss.lever_arm_x/y/z explicitly to override.",
            base_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
        }
      } else {
        // Empty frame_id or same as base: nothing to resolve, mark done.
        gnss_lever_arm_tf_resolved_ = true;
      }
    }

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
    // Covariance floor protects against Mahalanobis self-rejection on RTK
    // Fixed: ublox_dgnss reports σxy ~3 mm when carr_soln = FIXED, and any
    // wheel/IMU drift >~1 cm between fixes then fails the chi² outlier gate
    // (16.27 at 3 DoF). Floor σxy = 2 cm, σz = 5 cm so small integration
    // drift stays inside the gate while still benefitting from RTK precision.
    constexpr double kMinVarXY = 4e-4;    // σ = 0.02 m
    constexpr double kMinVarZ  = 2.5e-3;  // σ = 0.05 m
    if (msg->position_covariance_type == 3) {
      // Full 3x3 covariance available: use it directly including off-diagonals
      Eigen::Matrix3d cov;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          cov(i, j) = msg->position_covariance[i*3 + j];

      // Validate diagonal is positive
      if (cov(0,0) > 0.0 && cov(1,1) > 0.0 && cov(2,2) > 0.0) {
        if (cov(0,0) < kMinVarXY) cov(0,0) = kMinVarXY;
        if (cov(1,1) < kMinVarXY) cov(1,1) = kMinVarXY;
        if (cov(2,2) < kMinVarZ)  cov(2,2) = kMinVarZ;
        fix.has_full_covariance = true;
        fix.full_covariance = cov;
        fix.hdop = std::sqrt((cov(0,0) + cov(1,1)) / 2.0);  // for validity check
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
      if (var_xy < kMinVarXY) var_xy = kMinVarXY;
      double var_z  = msg->position_covariance[8];
      if (var_z < kMinVarZ) var_z = kMinVarZ;
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
    const auto& dbg = fc_->get_gnss_debug();

    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GNSS fix rejected: %s (hdop=%.2f, d2=%.1f, threshold=%.1f)",
        gnss_reason_str(dbg.reason).c_str(),
        fix.hdop,
        dbg.mahalanobis_sq,
        dbg.chi2_threshold);
    }

    publish_gnss_status(rclcpp::Time(msg->header.stamp));

    auto fc_status = fc_->get_status();
    if (!fc_status.heading_validated) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Heading not yet validated: lever arm inactive. "
        "Distance traveled: %.1fm (need %.1fm), or provide dual antenna / IMU orientation.",
        fc_status.distance_traveled,
        5.0);
    }
  }

  // ─── GPSFix callback ──────────────────────────────────────────────────────
  // Handles gps_msgs/GPSFix when gnss.use_gps_fix: true.
  // Advantages over NavSatFix:
  //   - RTK_FLOAT status (status 20) is expressible; NavSatFix can only reach RTK_FIXED.
  //   - Receiver-native hdop/vdop fields for direct DOP-scaled noise.
  //   - satellites_used for the quality gate.
  //   - err_horz/err_vert (95% CI bounds) as a fallback covariance source.

  void gps_fix_callback(const gps_msgs::msg::GPSFix::SharedPtr msg, int source_id = 0)
  {
    if (source_id == 0) mark_sensor_received("GNSS");
    else                mark_sensor_received("GNSS2");
    if (!fc_->is_initialized()) return;

    if (msg->status.status < 0) return;

    double t = rclcpp::Time(msg->header.stamp).seconds();

    fusioncore::sensors::LLAPoint lla;
    lla.lat_rad = msg->latitude  * M_PI / 180.0;
    lla.lon_rad = msg->longitude * M_PI / 180.0;
    lla.alt_m   = msg->altitude;

    fusioncore::sensors::ECEFPoint ecef;
    gnss_to_output(lla, ecef);

    if (!gnss_ref_set_) {
      gnss_ref_lla_  = lla;
      gnss_ref_ecef_ = ecef;
      gnss_ref_set_  = true;
      RCLCPP_INFO(get_logger(), "GNSS reference set (GPSFix): lat=%.6f lon=%.6f",
        msg->latitude, msg->longitude);
    }

    {
      double dx = ecef.x - gnss_ref_ecef_.x;
      double dy = ecef.y - gnss_ref_ecef_.y;
      double dz = ecef.z - gnss_ref_ecef_.z;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) > 10000.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "GPSFix dropped: more than 10km from reference (hardware glitch)");
        return;
      }
    }

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
    fix.source_id = source_id;
    fix.lever_arm = (source_id == 0) ? gnss_lever_arm_ : gnss_lever_arm2_;

    // gps_msgs/GPSStatus constants:
    //   STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
    //   STATUS_DGPS_FIX=18, STATUS_RTK_FIX=19, STATUS_RTK_FLOAT=20
    using S = gps_msgs::msg::GPSStatus;
    switch (msg->status.status) {
      case S::STATUS_RTK_FIX:
        fix.fix_type = fusioncore::sensors::GnssFixType::RTK_FIXED; break;
      case S::STATUS_RTK_FLOAT:
        fix.fix_type = fusioncore::sensors::GnssFixType::RTK_FLOAT; break;
      case S::STATUS_GBAS_FIX:
        fix.fix_type = fusioncore::sensors::GnssFixType::RTK_FIXED; break;
      case S::STATUS_DGPS_FIX:
      case S::STATUS_SBAS_FIX:
        fix.fix_type = fusioncore::sensors::GnssFixType::DGPS_FIX; break;
      default:
        fix.fix_type = fusioncore::sensors::GnssFixType::GPS_FIX; break;
    }

    // satellites_used is directly available in GPSFix (NavSatFix has no equivalent).
    fix.satellites = (msg->status.satellites_used > 0)
      ? static_cast<int>(msg->status.satellites_used) : 4;

    // Covariance priority:
    //   1. Full 3x3 from position_covariance_type==3 (most accurate)
    //   2. Diagonal from position_covariance_type>=1
    //   3. err_horz/err_vert (95% CI from receiver): construct a diagonal covariance
    //   4. Receiver hdop/vdop: actual DOP values, scale with base_noise in the core
    //   5. Defaults

    constexpr double kMinVarXY = 4e-4;   // sigma = 0.02 m
    constexpr double kMinVarZ  = 2.5e-3; // sigma = 0.05 m

    if (msg->position_covariance_type == gps_msgs::msg::GPSFix::COVARIANCE_TYPE_KNOWN) {
      Eigen::Matrix3d cov;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          cov(i, j) = msg->position_covariance[i*3 + j];
      if (cov(0,0) > 0.0 && cov(1,1) > 0.0 && cov(2,2) > 0.0) {
        if (cov(0,0) < kMinVarXY) cov(0,0) = kMinVarXY;
        if (cov(1,1) < kMinVarXY) cov(1,1) = kMinVarXY;
        if (cov(2,2) < kMinVarZ)  cov(2,2) = kMinVarZ;
        fix.has_full_covariance = true;
        fix.full_covariance = cov;
        fix.hdop = std::sqrt((cov(0,0) + cov(1,1)) / 2.0);
        fix.vdop = std::sqrt(cov(2,2));
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
      }
    } else if (msg->position_covariance_type >= gps_msgs::msg::GPSFix::COVARIANCE_TYPE_APPROXIMATED) {
      double var_xy = (msg->position_covariance[0] + msg->position_covariance[4]) / 2.0;
      if (var_xy < kMinVarXY) var_xy = kMinVarXY;
      double var_z = msg->position_covariance[8];
      if (var_z < kMinVarZ) var_z = kMinVarZ;
      if (var_xy > 0.0 && var_z > 0.0) {
        fix.hdop = std::sqrt(var_xy);
        fix.vdop = std::sqrt(var_z);
      } else {
        fix.hdop = 1.5;
        fix.vdop = 2.0;
      }
    } else if (msg->err_horz > 0.0 && msg->err_vert > 0.0) {
      // err_horz/err_vert are 95% CI bounds in meters. Convert to 1-sigma variance.
      double sigma_xy = msg->err_horz / 1.96;
      double sigma_z  = msg->err_vert / 1.96;
      double var_xy = sigma_xy * sigma_xy;
      double var_z  = sigma_z  * sigma_z;
      if (var_xy < kMinVarXY) var_xy = kMinVarXY;
      if (var_z  < kMinVarZ)  var_z  = kMinVarZ;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      cov(0,0) = var_xy;
      cov(1,1) = var_xy;
      cov(2,2) = var_z;
      fix.has_full_covariance = true;
      fix.full_covariance = cov;
      fix.hdop = std::sqrt(var_xy);
      fix.vdop = std::sqrt(var_z);
    } else if (msg->hdop > 0.0 && msg->vdop > 0.0) {
      // Receiver-native dimensionless DOP: used directly by the core noise model
      // (sigma_xy = base_noise_xy * hdop, sigma_z = base_noise_z * vdop).
      fix.hdop = msg->hdop;
      fix.vdop = msg->vdop;
    } else {
      fix.hdop = 1.5;
      fix.vdop = 2.0;
    }

    bool accepted = fc_->update_gnss(t, fix);
    const auto& dbg = fc_->get_gnss_debug();

    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "GPSFix rejected: %s (hdop=%.2f, d2=%.1f, threshold=%.1f)",
        gnss_reason_str(dbg.reason).c_str(),
        fix.hdop,
        dbg.mahalanobis_sq,
        dbg.chi2_threshold);
    }

    publish_gnss_status(rclcpp::Time(msg->header.stamp));

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
    mark_sensor_received("Heading");
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

    // Some drivers set covariance[0] = -1 to signal "no roll/pitch data" (yaw-only message).
    // This callback only ever reads cov[8] (yaw variance), so reject only when
    // yaw data is also absent. Rejects all-zero and all-unknown messages; accepts
    // messages where cov[0] = -1 but cov[8] carries a valid yaw variance.
    if (msg->orientation_covariance[0] < 0.0 && msg->orientation_covariance[8] <= 0.0) {
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
    mark_sensor_received("Heading");
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

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    if (!fc_->is_initialized()) return;
    double t = rclcpp::Time(msg->header.stamp).seconds();
    bool accepted = fc_->update_magnetometer(
      t,
      msg->magnetic_field.x,
      msg->magnetic_field.y,
      msg->magnetic_field.z);
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Magnetometer heading update rejected (chi2 gate)");
    }
  }

  // ─── Observability helpers ────────────────────────────────────────────────

  // Converts a GnssRejectionReason enum to the string stored in the message.
  static std::string gnss_reason_str(fusioncore::GnssRejectionReason r)
  {
    switch (r) {
      case fusioncore::GnssRejectionReason::ACCEPTED:        return "ACCEPTED";
      case fusioncore::GnssRejectionReason::FIX_TYPE_LOW:    return "FIX_TYPE_LOW";
      case fusioncore::GnssRejectionReason::HDOP_HIGH:       return "HDOP_HIGH";
      case fusioncore::GnssRejectionReason::VDOP_HIGH:       return "VDOP_HIGH";
      case fusioncore::GnssRejectionReason::MIN_SATS:        return "MIN_SATS";
      case fusioncore::GnssRejectionReason::CHI2_FAILED:     return "CHI2_FAILED";
      case fusioncore::GnssRejectionReason::DELAY_TOO_LARGE: return "DELAY_TOO_LARGE";
      default:                                                return "NOT_PROCESSED";
    }
  }

  // Publishes /fusion/debug/gnss_status from the debug struct the core just populated.
  // Called from gnss_callback and gps_fix_callback immediately after update_gnss().
  void publish_gnss_status(const rclcpp::Time& stamp)
  {
    if (!gnss_status_pub_) return;
    const auto& d = fc_->get_gnss_debug();

    fusioncore_ros::msg::GnssStatus msg;
    msg.header.stamp     = stamp;
    msg.header.frame_id  = odom_frame_;
    msg.accepted         = d.accepted;
    msg.rejection_reason = gnss_reason_str(d.reason);
    msg.mahalanobis_sq   = d.mahalanobis_sq;
    msg.chi2_threshold   = d.chi2_threshold;
    msg.hdop             = d.hdop;
    msg.vdop             = d.vdop;
    msg.satellites       = d.satellites;
    msg.fix_type         = d.fix_type;
    msg.in_coast_mode    = d.in_coast_mode;
    msg.consecutive_rejects = d.consecutive_rejects;
    msg.position_sigma_x = d.position_sigma_x;
    msg.position_sigma_y = d.position_sigma_y;
    msg.lever_arm_used   = d.lever_arm_used;
    msg.heading_sigma_deg = d.heading_sigma_deg;

    gnss_status_pub_->publish(msg);
  }

  // Extracts heading 1-sigma in degrees from the filter covariance via quaternion Jacobian.
  double compute_heading_sigma_deg(const fusioncore::State& s) const
  {
    const double qw = s.x[fusioncore::QW];
    const double qx = s.x[fusioncore::QX];
    const double qy = s.x[fusioncore::QY];
    const double qz = s.x[fusioncore::QZ];

    const double t3 = 2.0 * (qw*qz + qx*qy);
    const double t4 = 1.0 - 2.0 * (qy*qy + qz*qz);
    const double safe_denom_yaw = std::max(t3*t3 + t4*t4, 1e-12);

    // d(yaw)/d(qw, qx, qy, qz): row 2 of the full quaternion-to-Euler Jacobian
    Eigen::Matrix<double, 1, 4> J_yaw;
    J_yaw(0,0) = 2.0*qz*t4 / safe_denom_yaw;
    J_yaw(0,1) = 2.0*qy*t4 / safe_denom_yaw;
    J_yaw(0,2) = (2.0*qx*t4 + 4.0*qy*t3) / safe_denom_yaw;
    J_yaw(0,3) = (2.0*qw*t4 + 4.0*qz*t3) / safe_denom_yaw;

    static constexpr int qi[4] = {
      fusioncore::QW, fusioncore::QX, fusioncore::QY, fusioncore::QZ
    };
    Eigen::Matrix4d P_quat;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        P_quat(i,j) = s.P(qi[i], qi[j]);

    double yaw_var = (J_yaw * P_quat * J_yaw.transpose())(0, 0);
    return std::sqrt(std::max(yaw_var, 0.0)) * 180.0 / M_PI;
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
    odom.pose.pose.position.z = force_2d_ ? 0.0 : s.x[fusioncore::Z];

    odom.pose.pose.orientation.x = s.x[fusioncore::QX];
    odom.pose.pose.orientation.y = s.x[fusioncore::QY];
    odom.pose.pose.orientation.z = s.x[fusioncore::QZ];
    odom.pose.pose.orientation.w = s.x[fusioncore::QW];

    odom.twist.twist.linear.x  = s.x[fusioncore::VX];
    odom.twist.twist.linear.y  = s.x[fusioncore::VY];
    odom.twist.twist.linear.z  = force_2d_ ? 0.0 : s.x[fusioncore::VZ];
    odom.twist.twist.angular.x = s.x[fusioncore::WX];
    odom.twist.twist.angular.y = s.x[fusioncore::WY];
    odom.twist.twist.angular.z = s.x[fusioncore::WZ];

    // Publish UKF covariance so Nav2 and other consumers see real uncertainty.
    // pose.covariance is 6x6 row-major for [x, y, z, roll, pitch, yaw].
    // twist.covariance is 6x6 row-major for [vx, vy, vz, wx, wy, wz].
    const fusioncore::StateMatrix& P = s.P;

    // Pose covariance: ROS convention is [x, y, z, roll, pitch, yaw].
    // The UKF tracks orientation as a quaternion (qw, qx, qy, qz), so we must
    // propagate quaternion covariance through the quaternion-to-Euler Jacobian:
    //
    //   C_euler = J * P_quat * J^T      (3x3 Euler covariance)
    //   C_pos_euler = P_pos_quat * J^T  (3x3 position-Euler cross-covariance)
    //
    // where J = d(roll,pitch,yaw)/d(qw,qx,qy,qz) is the 3x4 analytical Jacobian
    // evaluated at the current quaternion. Without this step, Nav2 would read
    // quaternion component variance instead of yaw variance (wrong by ~4x for
    // small angles, increasingly wrong as orientation changes).
    {
      const double qw = s.x[fusioncore::QW];
      const double qx = s.x[fusioncore::QX];
      const double qy = s.x[fusioncore::QY];
      const double qz = s.x[fusioncore::QZ];

      // Intermediate terms for the three Euler angle formulas.
      const double t0 = 2.0 * (qw*qx + qy*qz);          // roll numerator
      const double t1 = 1.0 - 2.0 * (qx*qx + qy*qy);    // roll denominator
      const double t2 = std::clamp(2.0 * (qw*qy - qz*qx), -1.0, 1.0); // pitch sin
      const double t3 = 2.0 * (qw*qz + qx*qy);           // yaw numerator
      const double t4 = 1.0 - 2.0 * (qy*qy + qz*qz);    // yaw denominator

      // Gimbal lock protection: all three denominators go to zero when pitch = +/-90 deg.
      // At pitch = +/-90 deg, t0=t1=0 (roll undefined) and t3=t4=0 (yaw undefined) and
      // sqrt(1-t2^2)=0 (pitch Jacobian singular). Clamp all three to 1e-12 to produce
      // large-but-finite covariance instead of NaN. The unit quaternion constraint
      // guarantees none of these denominators can be negative.
      const double safe_denom_roll  = std::max(t0*t0 + t1*t1, 1e-12);
      const double safe_denom_yaw   = std::max(t3*t3 + t4*t4, 1e-12);
      const double safe_pitch       = std::max(std::sqrt(1.0 - t2*t2), 1e-12);

      // 3x4 Jacobian: rows = [roll, pitch, yaw], cols = [qw, qx, qy, qz].
      Eigen::Matrix<double, 3, 4> J;

      // d(roll)/d(qw, qx, qy, qz)  via d/d* atan2(t0, t1)
      J(0,0) = 2.0*qx*t1 / safe_denom_roll;
      J(0,1) = (2.0*qw*t1 + 4.0*qx*t0) / safe_denom_roll;
      J(0,2) = (2.0*qz*t1 + 4.0*qy*t0) / safe_denom_roll;
      J(0,3) = 2.0*qy*t1 / safe_denom_roll;

      // d(pitch)/d(qw, qx, qy, qz)  via d/d* asin(t2)
      J(1,0) =  2.0*qy / safe_pitch;
      J(1,1) = -2.0*qz / safe_pitch;
      J(1,2) =  2.0*qw / safe_pitch;
      J(1,3) = -2.0*qx / safe_pitch;

      // d(yaw)/d(qw, qx, qy, qz)  via d/d* atan2(t3, t4)
      J(2,0) = 2.0*qz*t4 / safe_denom_yaw;
      J(2,1) = 2.0*qy*t4 / safe_denom_yaw;
      J(2,2) = (2.0*qx*t4 + 4.0*qy*t3) / safe_denom_yaw;
      J(2,3) = (2.0*qw*t4 + 4.0*qz*t3) / safe_denom_yaw;

      // 4x4 quaternion covariance sub-block from P (order: qw, qx, qy, qz).
      static constexpr int qi[4] = {
        fusioncore::QW, fusioncore::QX, fusioncore::QY, fusioncore::QZ
      };
      Eigen::Matrix4d P_quat;
      for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
          P_quat(i,j) = P(qi[i], qi[j]);

      // 3x4 position-quaternion cross-covariance (rows=XYZ, cols=qw,qx,qy,qz).
      static constexpr int pi[3] = {
        fusioncore::X, fusioncore::Y, fusioncore::Z
      };
      Eigen::Matrix<double, 3, 4> P_pos_quat;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
          P_pos_quat(i,j) = P(pi[i], qi[j]);

      // Propagate through Jacobian.
      const Eigen::Matrix3d C_euler     = J * P_quat * J.transpose();
      const Eigen::Matrix3d C_pos_euler = P_pos_quat * J.transpose();

      // Fill 6x6 pose covariance (row-major, [x,y,z,roll,pitch,yaw]).
      // top-left 3x3: position covariance
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          odom.pose.covariance[i*6 + j] = P(pi[i], pi[j]);
      // top-right 3x3: position-Euler cross-covariance
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          odom.pose.covariance[i*6 + (3+j)] = C_pos_euler(i, j);
      // bottom-left 3x3: Euler-position cross-covariance (symmetric)
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          odom.pose.covariance[(3+i)*6 + j] = C_pos_euler(j, i);
      // bottom-right 3x3: Euler angle covariance
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          odom.pose.covariance[(3+i)*6 + (3+j)] = C_euler(i, j);
    }

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
    tf.transform.translation.z = force_2d_ ? 0.0 : s.x[fusioncore::Z];
    tf.transform.rotation.x = s.x[fusioncore::QX];
    tf.transform.rotation.y = s.x[fusioncore::QY];
    tf.transform.rotation.z = s.x[fusioncore::QZ];
    tf.transform.rotation.w = s.x[fusioncore::QW];

    if (publish_tf_) tf_broadcaster_->sendTransform(tf);
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

    // VSLAM (only shown when configured)
    if (!vslam_topic_.empty()) {
      diag_array.status.push_back(make_status("VSLAM",
        health_to_level(status.vslam_health),
        health_to_str(status.vslam_health),
        {{"outlier_count", std::to_string(status.vslam_outliers)}}));
    }

    // Magnetometer (only shown when configured)
    if (mag_enabled_) {
      diag_array.status.push_back(make_status("Magnetometer",
        health_to_level(status.mag_health),
        health_to_str(status.mag_health),
        {{"outlier_count", std::to_string(status.mag_outliers)}}));
    }

    // Filter
    auto heading_src_str = [](fusioncore::HeadingSource src) -> std::string {
      switch (src) {
        case fusioncore::HeadingSource::NONE:            return "NONE: lever arm inactive";
        case fusioncore::HeadingSource::DUAL_ANTENNA:    return "DUAL_ANTENNA";
        case fusioncore::HeadingSource::IMU_ORIENTATION: return "IMU_ORIENTATION (9-axis)";
        case fusioncore::HeadingSource::GPS_TRACK:       return "GPS_TRACK";
        case fusioncore::HeadingSource::MAGNETOMETER:    return "MAGNETOMETER";
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

    // FilterHealth: plottable topic with innovation norms and position uncertainty.
    // Consumed directly by Foxglove without a custom panel: every field is a float64.
    if (filter_health_pub_) {
      const fusioncore::State& s = fc_->get_state();

      fusioncore_ros::msg::FilterHealth fh;
      fh.header.stamp    = stamp;
      fh.header.frame_id = odom_frame_;

      fh.gnss_innovation_norm    = status.gnss_innovation_norm;
      fh.imu_innovation_norm     = status.imu_innovation_norm;
      fh.encoder_innovation_norm = status.encoder_innovation_norm;

      fh.position_sigma_x = status.position_sigma_x;
      fh.position_sigma_y = status.position_sigma_y;
      fh.position_sigma_z = status.position_sigma_z;

      fh.heading_sigma_deg = compute_heading_sigma_deg(s);

      fh.heading_validated = status.heading_validated;
      fh.heading_source    = heading_src_str(status.heading_source);

      fh.gnss_in_coast           = status.gnss_in_coast;
      fh.gnss_consecutive_rejects = status.gnss_consecutive_rejects;

      fh.distance_traveled_m = status.distance_traveled;

      fh.gnss_outlier_count    = status.gnss_outliers;
      fh.imu_outlier_count     = status.imu_outliers;
      fh.encoder_outlier_count = status.enc_outliers;
      fh.mag_outlier_count     = status.mag_outliers;

      filter_health_pub_->publish(fh);
    }
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
  //
  // proj_normalize_for_visualization forces the map-plotting axis order for
  // EPSG:4326: the first axis is LONGITUDE, the second is LATITUDE (because
  // map renderers expect x=east, y=north). Values must be in degrees.
  //
  // Previous code passed (lat, lon) in slots 0/1, which caused PROJ to
  // interpret the latitude value as longitude (and vice versa): giving an
  // ECEF roughly 6500 km off for mid-latitude sites. With
  // reference.use_first_fix=true both sides of the round-trip were wrong in
  // the same way so the bug was masked; with an external fixed ECEF
  // reference (reference.use_first_fix=false), every live fix was rejected
  // as thousands of km from the reference.
  //
  // Note: output_to_gnss uses `r.lpzt.phi` / `r.lpzt.lam` which are semantic
  // (phi=latitude, lam=longitude) regardless of axis order, so no change is
  // needed there.
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
      lla.lon_rad * 180.0 / M_PI,   // slot 0 = longitude (visualization order)
      lla.lat_rad * 180.0 / M_PI,   // slot 1 = latitude
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
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          imu2_sub_;
  rclcpp::Subscription<compass_msgs::msg::Azimuth>::SharedPtr     azimuth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          gnss_heading_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        encoder_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        encoder2_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        vslam_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        gnss_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        radar_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    gnss_sub_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr          gps_fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                gnss2_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                       odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr         diag_pub_;
  rclcpp::Publisher<fusioncore_ros::msg::GnssStatus>::SharedPtr               gnss_status_pub_;
  rclcpp::Publisher<fusioncore_ros::msg::FilterHealth>::SharedPtr             filter_health_pub_;
  rclcpp::TimerBase::SharedPtr                                                publish_timer_;
  rclcpp::TimerBase::SharedPtr                                                diag_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                          reset_srv_;
  rclcpp::Service<fusioncore_ros::srv::FromLL>::SharedPtr                     from_ll_srv_;

  std::string base_frame_;
  std::string odom_frame_;
  double      publish_rate_;
  bool        force_2d_    = false;
  bool        publish_tf_  = true;
  bool        use_gps_fix_  = false;
  std::string heading_topic_;
  std::string gnss_fix_topic_;
  std::string gnss2_topic_;
  std::string azimuth_topic_;
  std::string mag_topic_;
  bool        mag_enabled_ = false;
  std::string encoder_topic_;
  std::string encoder2_topic_;
  double      enc2_vel_noise_ = 0.05;
  double      enc2_yaw_noise_ = 0.02;
  std::string vslam_topic_;
  std::string vslam_frame_override_;
  // VSLAM map-to-odom frame offset: applied to every VSLAM measurement.
  // Set on first measurement; re-computed on reinitialization detection.
  bool   vslam_origin_set_         = false;
  double vslam_offset_x_           = 0.0;
  double vslam_offset_y_           = 0.0;
  double vslam_offset_z_           = 0.0;
  int    vslam_consecutive_rejects_ = 0;
  int    vslam_reinit_n_           = 10;
  std::string gnss_vel_topic_;
  std::string radar_vel_topic_;
  double      radar_vel_noise_ = 0.1;

  bool        pending_init_        = false;

  // Static bias initialization window
  double init_window_duration_         = 0.0;
  bool   init_window_collecting_       = false;
  bool   init_window_aborted_          = false;
  double init_window_start_            = 0.0;
  bool   init_window_start_is_msg_time_ = false;  // true: msg timestamps; false: wall clock
  int    init_win_n_                   = 0;
  double init_win_wx_ = 0.0, init_win_wy_ = 0.0, init_win_wz_ = 0.0;
  double init_win_ax_ = 0.0, init_win_ay_ = 0.0, init_win_az_ = 0.0;
  double init_win_qw_ = 0.0, init_win_qx_ = 0.0, init_win_qy_ = 0.0, init_win_qz_ = 0.0;
  int    init_win_orient_n_      = 0;
  bool        gnss_ref_set_        = false;
  bool        imu_remove_gravity_  = false;
  std::string imu_topic_;
  std::string imu_frame_override_;
  std::string imu_frame_resolved_;
  std::string imu2_topic_;
  std::string imu2_frame_override_;
  bool        imu2_remove_gravity_ = false;
  double      last_imu_time_       = 0.0;   // timestamp of most recent IMU message
  fusioncore::sensors::LLAPoint  gnss_ref_lla_;
  fusioncore::sensors::ECEFPoint gnss_ref_ecef_;

  fusioncore::sensors::GnssFixType  min_fix_type_   = fusioncore::sensors::GnssFixType::GPS_FIX;
  fusioncore::sensors::GnssLeverArm gnss_lever_arm_;    // primary receiver
  fusioncore::sensors::GnssLeverArm gnss_lever_arm2_;   // secondary receiver (fix2_topic)

  // Auto-resolve flags: true means a non-zero value was given in the YAML
  // and we should NOT overwrite it from TF. Default (all zero) triggers
  // one-shot TF resolution on the first matching message.
  bool imu_lever_arm_explicit_    = false;
  bool gnss_lever_arm_explicit_   = false;
  bool imu_lever_arm_tf_resolved_  = false;
  bool gnss_lever_arm_tf_resolved_ = false;

  // ZUPT parameters
  bool   zupt_enabled_            = true;
  double zupt_velocity_threshold_ = 0.05;
  double zupt_angular_threshold_  = 0.05;
  double zupt_noise_sigma_        = 0.01;

  // NHC holonomic auto-detection
  // |VY| must exceed this to count as real lateral motion (filters vibration/noise)
  static constexpr double kNhcDetectVyThreshold_ = 0.02;  // m/s
  // Consecutive messages above threshold before holonomic flag is set (~0.1s at 50Hz)
  static constexpr int    kNhcDetectN_            = 5;
  bool   nhc_auto_detect_        = true;
  bool   nhc_holonomic_detected_ = false;
  int    nhc_nonzero_vy_count_   = 0;
  double nhc_vy_auto_noise_      = 0.05;  // VX noise cached at configure time

  // Callback groups: sensor callbacks are mutually exclusive (protect UKF state);
  // publish timer runs in its own group so it never waits on a sensor callback.
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr publish_cb_group_;
  std::mutex fc_mutex_;

  // Sensor wait (#28)
  bool                     wait_for_all_sensors_ = false;
  double                   sensor_wait_timeout_  = 10.0;
  double                   activate_time_        = 0.0;
  bool                     sensor_wait_done_     = false;
  std::set<std::string>    sensors_expected_;
  std::set<std::string>    sensors_received_;

  // Autostart: self-transition configure -> activate without external lifecycle management
  bool autostart_ = true;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  // Deterministic replay checkpoint (#27)
  std::string checkpoint_path_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_checkpoint_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_checkpoint_srv_;

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
