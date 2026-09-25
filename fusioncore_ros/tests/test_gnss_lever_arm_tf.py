# Both GNSS receivers resolve their own antenna lever arm from TF. Issue #4.
#
# Before this, the auto-resolve was hardcoded to source_id == 0 inside the
# NavSatFix callback. Two consequences, both reported by a dual-antenna user:
# the second antenna's offset had to be typed into the YAML even though the URDF
# already described exactly where it was, and anyone reading gps_msgs/GPSFix got
# no auto-resolve at all.
#
# This test would FAIL on that code: the secondary assertion never fires, because
# nothing ever looked the second frame up.
#
# The lever arms are observed through the node's own log lines rather than a
# topic, because a resolved lever arm is not published anywhere. That makes the
# log line the contract, so it is asserted with the numbers in it.

import os
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', str(42 + os.getpid() % 40))

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

# Two antennas on one robot, front and rear, deliberately asymmetric so a test
# that mixed them up would read the wrong numbers rather than passing anyway.
FRONT = ('gps_front', 1.00, 0.20, 0.50)
REAR = ('gps_rear', -0.80, -0.15, 0.55)

REF_LAT, REF_LON, REF_ALT = 43.258878, -79.913153, 100.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    def static_tf(child, x, y, z):
        return launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(x), str(y), str(z), '0', '0', '0', 'base_link', child],
            output='log',
        )

    node = launch_ros.actions.Node(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        output='screen',
        parameters=[{
            'init.wait_for_all_sensors': False,
            'reference.use_first_fix': True,
            'base_frame': 'base_link',
            'gnss.fix_topic': '/gnss/fix',
            'gnss.fix2_topic': '/gnss/fix2',
        }],
    )
    return launch.LaunchDescription([
        static_tf(*FRONT),
        static_tf(*REAR),
        node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGnssLeverArmAutoResolve(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gnss_lever_arm_tf')
        cli = cls.node.create_client(ChangeState, '/fusioncore/change_state')
        assert cli.wait_for_service(timeout_sec=30.0), 'lifecycle service never appeared'
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(cls.node, fut, timeout_sec=30.0)
        assert fut.result() is not None, 'configure transition timed out'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_fixes(self):
        """Feed each receiver a fix carrying its own antenna frame_id.

        The IMU stream is not decoration. gnss_callback returns early while
        fc_->is_initialized() is false, so a GPS-only version of this test never
        reaches the lever-arm resolve at all and both assertions time out for a
        reason that has nothing to do with the feature.

        QoS matters too: the node subscribes with SensorDataQoS (BEST_EFFORT).
        """
        imu_pub = self.node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        pubs = [
            (self.node.create_publisher(
                NavSatFix, '/gnss/fix', qos_profile_sensor_data), FRONT[0]),
            (self.node.create_publisher(
                NavSatFix, '/gnss/fix2', qos_profile_sensor_data), REAR[0]),
        ]
        time.sleep(2.0)
        for i in range(600):
            now = self.node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.81
            # Every element must be a float, not a bare int. Humble's message
            # assertion checks the type of each value and rejects 0; Jazzy lets
            # it through, so this passes locally and fails CI on humble only.
            imu.orientation_covariance = [
                -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu.angular_velocity_covariance = [
                1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
            imu.linear_acceleration_covariance = [
                1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
            imu_pub.publish(imu)
            if i % 10 == 0:
                for pub, frame in pubs:
                    msg = NavSatFix()
                    msg.header.stamp = now
                    msg.header.frame_id = frame
                    msg.status.status = NavSatStatus.STATUS_FIX
                    msg.latitude, msg.longitude, msg.altitude = REF_LAT, REF_LON, REF_ALT
                    msg.position_covariance = [
                        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0]
                    msg.position_covariance_type = \
                        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.001)
            time.sleep(0.01)

    def test_both_antennas_resolve_their_own_arm(self, proc_output):
        """Each receiver resolves independently, from its own frame."""
        self._publish_fixes()
        proc_output.assertWaitFor(
            'GNSS lever arm (primary) auto-resolved from TF base_link -> gps_front: '
            'x=1.000 y=0.200 z=0.500 m',
            timeout=60,
        )
        # The regression this feature exists for. On the old code the secondary
        # was never looked up at all, so this is the assertion that fails there.
        proc_output.assertWaitFor(
            'GNSS lever arm (secondary) auto-resolved from TF base_link -> gps_rear: '
            'x=-0.800 y=-0.150 z=0.550 m',
            timeout=60,
        )
