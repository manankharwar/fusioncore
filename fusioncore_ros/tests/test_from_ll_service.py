# The /fromLL service contract, locked down against the regression in issue #73.
#
# The bug: /fromLL was advertised as fusioncore_ros/srv/FromLL, which has fields
# identical to robot_localization/srv/FromLL. ROS 2 matches services on name AND
# type, and nav2_waypoint_follower has the robot_localization type compiled into
# its header, so its client waited forever for a service it could not see. It
# never errored, it just hung, and it shipped that way from v0.2.1 to 0.3.4.
#
# Nothing caught it because a manual `ros2 service call` passes the type by hand
# and therefore always works. Only a client holding the type exposes the problem.
# So this test asserts the advertised TYPE STRING, not just that /fromLL exists.

import math
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from robot_localization.srv import FromLL

# A known origin, pinned at configure time so the test never depends on a GPS fix.
REF_LAT, REF_LON, REF_ALT = 43.258878, -79.913153, 100.0
REF_ECEF = (814818.4192, -4580454.6512, 4348559.7245)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    node = launch_ros.actions.Node(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        output='screen',
        parameters=[{
            'init.wait_for_all_sensors': False,
            'reference.use_first_fix': False,
            'reference.x': REF_ECEF[0],
            'reference.y': REF_ECEF[1],
            'reference.z': REF_ECEF[2],
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestFromLLService(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_from_ll')
        cls._configure_fusioncore()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _configure_fusioncore(cls):
        """Drive the lifecycle node to active so it advertises its services."""
        cli = cls.node.create_client(ChangeState, '/fusioncore/change_state')
        assert cli.wait_for_service(timeout_sec=30.0), 'lifecycle service never appeared'
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(cls.node, fut, timeout_sec=30.0)
        assert fut.result() is not None, 'configure transition timed out'
        # The node autostarts into active shortly after configuring.

    def _spin_until(self, predicate, timeout=30.0):
        end = time_now() + timeout
        while time_now() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            value = predicate()
            if value:
                return value
        return None

    def test_advertised_with_the_type_nav2_expects(self):
        """/fromLL must be robot_localization/srv/FromLL, not a look-alike."""
        def find():
            for name, types in self.node.get_service_names_and_types():
                if name == '/fromLL':
                    return types
            return None

        types = self._spin_until(find)
        self.assertIsNotNone(types, '/fromLL was never advertised')
        self.assertIn(
            'robot_localization/srv/FromLL', types,
            'nav2_waypoint_follower binds robot_localization/srv/FromLL. Advertising '
            'any other type, even one with identical fields, makes the service '
            'invisible to it and followGpsWaypoints hangs forever. See issue #73.')

    def test_converts_lat_lon_to_the_local_enu_frame(self):
        """A client holding the type gets correct answers, not just a connection."""
        cli = self.node.create_client(FromLL, '/fromLL')
        self.assertTrue(cli.wait_for_service(timeout_sec=30.0),
                        'no robot_localization-typed client could bind to /fromLL')

        def convert(lat, lon, alt=REF_ALT):
            req = FromLL.Request()
            req.ll_point.latitude = lat
            req.ll_point.longitude = lon
            req.ll_point.altitude = alt
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30.0)
            self.assertIsNotNone(fut.result(), 'fromLL call timed out')
            return fut.result().map_point

        # The origin itself maps to zero.
        p = convert(REF_LAT, REF_LON)
        self.assertAlmostEqual(p.x, 0.0, delta=0.05)
        self.assertAlmostEqual(p.y, 0.0, delta=0.05)

        # 0.0009 deg of latitude is about 100 m north, and must land on +y.
        north = convert(REF_LAT + 0.0009, REF_LON)
        self.assertAlmostEqual(north.y, 0.0009 * 111320.0, delta=1.0)
        self.assertAlmostEqual(north.x, 0.0, delta=0.5)

        # The same step of longitude shrinks by cos(latitude), and lands on +x.
        east = convert(REF_LAT, REF_LON + 0.0009)
        expected_east = 0.0009 * 111320.0 * math.cos(math.radians(REF_LAT))
        self.assertAlmostEqual(east.x, expected_east, delta=1.0)
        self.assertAlmostEqual(east.y, 0.0, delta=0.5)


def time_now():
    import time
    return time.monotonic()
