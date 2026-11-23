"""Contract test for safety_stop (REQ-SCOUT-009) fake scans in, cmd_vel out."""

import math
import time
import unittest

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PARAMS = {'stop_distance': 0.15, 'laser_to_front': 0.10,
          'sector_half_width_deg': 30.0, 'scan_timeout': 0.3}

N = 720
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / (N - 1)
FRONT = 359   # -0.25 deg; beam 360 is +0.25 deg
BACK = 0   # -pi (beam 719 is +pi, the same direction)


@pytest.mark.launch_test
def generate_test_description():
    """Launch safety_stop alone; the test plays scan source, teleop and bridge."""
    node = launch_ros.actions.Node(
        package='scout_localization',
        executable='safety_stop',
        parameters=[PARAMS], output='screen')
    return (launch.LaunchDescription([node,
                                      launch_testing.actions.ReadyToTest()]),
            {'safety_stop': node})


def scan_ranges(front=2.0, back=2.0):
    """Return N ranges: 2 m everywhere, `front` and `back` on 7 beams."""
    ranges = [2.0] * N
    for k in range(-3, 4):
        ranges[(FRONT + k) % N] = front
        ranges[(BACK + k) % N] = back
    return ranges


class TestSafetyStop(unittest.TestCase):
    """REQ-SCOUT-009: the test is the scan source, the teleop and the bridge."""

    @classmethod
    def setUpClass(cls):
        """Create the tester node, its publishers and the /cmd_vel subscription."""
        rclpy.init()
        cls.node = rclpy.create_node('safety_stop_tester')
        cls.scan_pub = cls.node.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)
        cls.cmd_pub = cls.node.create_publisher(Twist, 'cmd_vel_in', 10)
        cls.received = []
        cls.node.create_subscription(Twist, 'cmd_vel', cls.received.append, 10)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:          # let discovery happen
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        """Destroy the tester node."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def make_scan(self, ranges):
        """Build a LaserScan with the C1's geometry around `ranges`."""
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_laser'
        msg.angle_min = ANGLE_MIN
        msg.angle_max = math.pi
        msg.angle_increment = ANGLE_INCREMENT
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 16.0
        msg.ranges = [float(r) for r in ranges]
        return msg

    def pump(self, seconds, ranges, vx, wz=0.0):
        """Publish scan + cmd_vel_in at 10 Hz for `seconds`; return the last cmd_vel."""
        self.received.clear()
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        end = time.monotonic() + seconds
        next_pub = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                if ranges is not None:  # None = the LiDAR has gone quiet
                    self.scan_pub.publish(self.make_scan(ranges))
                self.cmd_pub.publish(cmd)
                next_pub = time.monotonic() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.received,
                        'no message on /cmd_vel: is safety_stop publishing?')
        return self.received[-1]

    def test_0_no_scan_yet_stops(self):
        """Stop everything before the first scan has arrived."""
        out = self.pump(1.5, None, 0.2, 0.3)
        self.assertEqual(out.linear.x, 0.0)
        self.assertEqual(out.angular.z, 0.0)

    def test_1_clear_passes_command(self):
        """Pass the command unchanged when nothing is within 0.25 m."""
        out = self.pump(1.5, scan_ranges(), 0.2, 0.3)
        self.assertAlmostEqual(out.linear.x, 0.2, places=6)
        self.assertAlmostEqual(out.angular.z, 0.3, places=6)

    def test_2_obstacle_blocks_forward(self):
        """Zero the forward speed at 0.20 m, but keep the turn."""
        out = self.pump(1.0, scan_ranges(front=0.20), 0.2, 0.3)
        self.assertEqual(out.linear.x, 0.0)
        self.assertAlmostEqual(out.angular.z, 0.3, places=6)

    def test_3_obstacle_allows_reverse(self):
        """Let the robot back away from the obstacle."""
        out = self.pump(1.0, scan_ranges(front=0.20), -0.1)
        self.assertAlmostEqual(out.linear.x, -0.1, places=6)

    def test_4_obstacle_behind_is_ignored(self):
        """Ignore an obstacle behind the robot."""
        out = self.pump(1.0, scan_ranges(back=0.10), 0.2)
        self.assertAlmostEqual(out.linear.x, 0.2, places=6)

    def test_5_invalid_beams_are_not_obstacles(self):
        """Ignore below-range_min, NaN and inf beams in front."""
        ranges = scan_ranges()
        ranges[FRONT] = 0.02
        ranges[FRONT + 1] = math.nan
        ranges[FRONT - 1] = math.inf
        out = self.pump(1.0, ranges, 0.2)
        self.assertAlmostEqual(out.linear.x, 0.2, places=6)

    def test_6_stale_scan_stops_everything(self):
        """Stop fully (also reverse and turning) when the scan goes stale."""
        self.pump(1.0, scan_ranges(), 0.2)        # fresh scans first
        out = self.pump(1.0, None, -0.1, 0.3)
        self.assertEqual(out.linear.x, 0.0)
        self.assertEqual(out.angular.z, 0.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """safety_stop exits cleanly when the launch stops it."""

    def test_exit_code(self, proc_info):
        """Accept exit code 0, or a stop by SIGINT / SIGTERM."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
