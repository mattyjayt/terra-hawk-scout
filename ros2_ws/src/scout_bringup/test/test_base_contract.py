"""Automated SIL contract test."""
import json, os, subprocess, sys, time, unittest

from geometry_msgs.msg import Twist, Vector3

import launch, launch_ros.actions, launch_testing.actions, launch_testing.asserts

import pytest, rclpy
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

REPO = os.environ.get('SCOUT_REPO', os.path.expanduser('~/Projects/terra-hawk-scout'))
FAKE = os.path.join(REPO, 'tools', 'fake_r4.py')
PORT, LOG = '/tmp/fake_r4_test', '/tmp/fake_r4_test.jsonl'
PKG = {'py': 'scout_base_py', 'cpp': 'scout_base_cpp'}[os.environ.get('SCOUT_BRIDGE_IMPL', 'py')]
PARAMS = {'port': PORT, 'track_width': 0.15, 'max_wheel_speed': 0.5, 'pwm_min': 60}


def start_fake(*extra):
    """Start a fake subprocess."""
    return subprocess.Popen([sys.executable, FAKE, '--link', PORT, '--log', LOG, '--boot-s', '0.3', *extra])


@pytest.mark.launch_test
def generate_test_description():
    """Generate a test description."""
    bridge = launch_ros.actions.Node(package=PKG, executable='base_bridge',
                                     parameters=[PARAMS], output='screen')
    return (launch.LaunchDescription(
        [bridge, launch_testing.actions.ReadyToTest()]), {'bridge': bridge})


class TestBaseContract(unittest.TestCase):
    """Unit Test Class for Base Contract."""

    @classmethod
    def setUpClass(cls):
        """Define setup class method."""
        if os.path.exists(LOG):
            os.unlink(LOG)
        cls.fake = start_fake()
        rclpy.init()
        cls.node = rclpy.create_node('contract_tester')

    @classmethod
    def tearDownClass(cls):
        """Define tear down class method."""
        cls.fake.terminate()
        cls.fake.wait()
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        """Define spin for method."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def fake_log(self):
        """Define a fake log."""
        with open(LOG) as f:
            return [json.loads(line) for line in f]

    def test_tc022_imu_rate_and_shape(self):
        """Define test case tc022."""
        msgs = []
        sub = self.node.create_subscription(Imu, 'imu/data_raw',
                                            msgs.append,
                                            qos_profile_sensor_data)
        self.spin_for(4.0)            # bridge reconnects (≤ 1 s) + fake boots (0.3 s)
        msgs.clear()
        t0 = time.time()
        self.spin_for(2.0)
        rate = len(msgs) / (time.time() - t0)
        self.node.destroy_subscription(sub)
        self.assertGreaterEqual(rate, 18.0)
        m = msgs[-1]
        self.assertEqual(m.header.frame_id, 'imu_link')
        self.assertEqual(m.orientation_covariance[0], -1.0)
        a = m.linear_acceleration
        self.assertAlmostEqual((a.x**2 + a.y**2 + a.z**2) ** 0.5,
                               9.81, delta=0.3)

    def test_tc021b_stop_after_cmd_timeout(self):
        """Silence on cmd_vel -> one S within 0.3 s of the last command."""
        pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.spin_for(1.0)                          # let discovery connect pub -> bridge
        for _ in range(10):                         # act: 0.5 s of driving at 20 Hz
            pub.publish(Twist(linear=Vector3(x=0.2)))
            self.spin_for(0.05)
        t_last = time.time()
        self.spin_for(0.6)                          # wait: silence
        rx = [e for e in self.fake_log() if 'rx' in e and e['t'] > t_last]
        stops = [e for e in rx if e['rx'] == 'S']
        self.assertTrue(stops, 'no S after cmd_vel stopped')       # check
        self.assertLess(stops[0]['t'] - t_last, 0.3)
        self.node.destroy_publisher(pub)


    # ---------- helpers for the cases below ----------
    def wait_for(self, predicate, timeout):
        """Spin until predicate() is true; return seconds taken or None."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return time.time() - t0
        return None

    def rx_since(self, t):
        """Lines the fake R4 received after time t (wall clock)."""
        return [e['rx'] for e in self.fake_log() if 'rx' in e and e['t'] > t]

    def restart_fake(self, *extra):
        """Replace the running fake R4 (e.g. with --freeze-imu)."""
        self.fake.terminate()
        self.fake.wait()
        self.fake = start_fake(*extra)
        type(self).fake = self.fake      # so tearDownClass stops the new one

    def wait_link_up(self, timeout=5.0):
        """Block until the bridge publishes IMU data (R4 connected, booted)."""
        got = []
        sub = self.node.create_subscription(Imu, 'imu/data_raw', got.append,
                                            qos_profile_sensor_data)
        took = self.wait_for(lambda: got, timeout)
        self.node.destroy_subscription(sub)
        self.assertIsNotNone(took, 'bridge never connected to the fake R4')

    def latest_diag(self):
        """Subscribe to diagnostics; return a list that fills with statuses."""
        got = []
        sub = self.node.create_subscription(
            DiagnosticArray, 'diagnostics',
            lambda m: got.extend(m.status), 10)
        return got, sub

    # ---------- TC-SCOUT-021a/b: drive and command timeout ----------
    def test_tc021a_drive_rate(self):
        """cmd_vel at 20 Hz -> M lines at ~20 Hz."""
        pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.spin_for(1.0)                     # discovery
        t0 = time.time()
        for _ in range(20):                    # 1 s of driving
            pub.publish(Twist(linear=Vector3(x=0.2)))
            self.spin_for(0.05)
        t1 = time.time()
        ms = [e for e in self.fake_log()
              if 'rx' in e and t0 < e['t'] <= t1 and e['rx'].startswith('M ')]
        self.node.destroy_publisher(pub)
        self.spin_for(0.5)                     # let the bridge stop again
        self.assertGreaterEqual(len(ms), 17)
        self.assertLessEqual(len(ms), 23)

    def test_tc021b_stop_after_cmd_timeout(self):
        """Silence on cmd_vel -> one S within 0.3 s of the last command."""
        pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.spin_for(1.0)
        for _ in range(10):
            pub.publish(Twist(linear=Vector3(x=0.2)))
            self.spin_for(0.05)
        t_last = time.time()
        self.spin_for(0.6)
        stops = [e for e in self.fake_log()
                 if 'rx' in e and e['t'] > t_last and e['rx'] == 'S']
        self.node.destroy_publisher(pub)
        self.assertTrue(stops, 'no S after cmd_vel stopped')
        self.assertLess(stops[0]['t'] - t_last, 0.3)

    # ---------- TC-SCOUT-023: pan ----------
    def test_tc023_pan(self):
        """pan/cmd 0.5 rad -> 'V 119' on the wire, joint_states ~0.506 rad."""
        pub = self.node.create_publisher(Float64, 'pan/cmd', 10)
        joints = []
        sub = self.node.create_subscription(JointState, 'joint_states',
                                            joints.append, 10)
        self.spin_for(1.0)
        t0 = time.time()
        pub.publish(Float64(data=0.5))
        took = self.wait_for(lambda: 'V 119' in self.rx_since(t0), 2.0)
        self.assertIsNotNone(took, "fake never received 'V 119'")
        joints.clear()
        self.wait_for(lambda: joints, 2.0)
        self.assertAlmostEqual(joints[-1].position[0], 0.506, delta=0.02)
        pub.publish(Float64(data=0.0))         # leave the pan centred
        self.spin_for(0.3)
        self.node.destroy_publisher(pub)
        self.node.destroy_subscription(sub)

    # ---------- /base/stop service ----------
    def test_base_stop_service(self):
        """/base/stop -> success, 'S ok'."""
        self.wait_link_up()      # runs first (alphabetical): link may be down
        cli = self.node.create_client(Trigger, 'base/stop')
        self.assertTrue(cli.wait_for_service(timeout_sec=3.0))
        fut = cli.call_async(Trigger.Request())
        self.wait_for(fut.done, 3.0)
        self.node.destroy_client(cli)
        self.assertTrue(fut.done(), 'service call timed out')
        self.assertTrue(fut.result().success, fut.result().message)
        self.assertEqual(fut.result().message, 'S ok')

    # ---------- TC-SCOUT-024: link loss and recovery ----------
    def test_tc024_link_loss_and_recovery(self):
        """Kill the fake -> diagnostics ERROR; restart -> OK again."""
        got, sub = self.latest_diag()
        self.fake.terminate()
        self.fake.wait()
        took = self.wait_for(
            lambda: got and got[-1].level == DiagnosticStatus.ERROR, 3.0)
        self.assertIsNotNone(took, 'no ERROR after link loss')
        self.assertLess(took, 2.0)             # 1 Hz diagnostics + detection
        self.assertEqual(got[-1].message, 'link down')
        self.fake = start_fake()
        type(self).fake = self.fake
        took = self.wait_for(
            lambda: got and got[-1].level == DiagnosticStatus.OK, 5.0)
        self.node.destroy_subscription(sub)
        self.assertIsNotNone(took, 'no OK after the R4 came back')

    # ---------- TC-SCOUT-024b: frozen IMU ----------
    def test_tc024b_frozen_imu(self):
        """A frozen IMU -> ERROR 'IMU frozen' and imu_frozen 'True'."""
        got, sub = self.latest_diag()
        self.restart_fake('--freeze-imu')
        took = self.wait_for(
            lambda: got and got[-1].message == 'IMU frozen', 6.0)
        self.assertIsNotNone(took, 'frozen IMU never reported')
        st = got[-1]
        self.assertEqual(st.level, DiagnosticStatus.ERROR)
        values = {kv.key: kv.value for kv in st.values}
        self.assertEqual(values['imu_frozen'], 'True')
        self.restart_fake()                    # back to a live IMU
        self.wait_for(lambda: got and got[-1].level == DiagnosticStatus.OK,
                      5.0)
        self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Checks that run after the bridge has been stopped."""

    def test_exit_code(self, proc_info):
        """The bridge exits cleanly (0, SIGINT or SIGTERM)."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
