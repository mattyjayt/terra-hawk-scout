"""Base Bridge Node for Scout Base ROS2 package."""
import collections
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from scout_base_py.conversions import (dps_to_radps, g_to_mps2,
                                       RateMeter, StaleDetector)
from scout_base_py.kinematics import (pan_rad_to_servo_deg,
                                      servo_deg_to_pan_rad, twist_to_pwm)
from scout_base_py.protocol import (Ack, format_drive, format_servo, ImuSample,
                                    parse_line, WatchdogStop)

from sensor_msgs.msg import BatteryState, Imu, JointState

import serial

from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from std_srvs.srv._trigger import Trigger_Request as Request
from std_srvs.srv._trigger import Trigger_Response as Response


class BaseBridge(Node):
    """Base Bridge Node for Scout Base ROS2 package."""

    def __init__(self):
        """Initialize the Base Bridge Node."""
        super().__init__('base_bridge')
        self.get_logger().info('Base Bridge Node has been started.')

        # Declare Parameters
        self.declare_parameter('port', '/dev/scout_base')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.0)
        self.declare_parameter('max_wheel_speed', 0.0)
        self.declare_parameter('pwm_min', 0)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('send_rate', 20.0)
        self.declare_parameter('gyro_var', 0.00001)
        self.declare_parameter('accel_var', 0.002)
        self.declare_parameter('imu_stale_count', 10)
        self.declare_parameter('pan_sign', 1)
        self.declare_parameter('pan_min', -1.5708)
        self.declare_parameter('pan_max', 1.5708)
        self.declare_parameter('cell_count', 2)
        self.declare_parameter('battery_present_v', 2.0)
        self.declare_parameter('frame_id_imu', 'imu_link')
        self.declare_parameter('frame_id_base', 'base_link')

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value
        self._track_width = self.get_parameter('track_width').value
        self._max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self._pwm_min = self.get_parameter('pwm_min').value
        self._max_linear = self.get_parameter('max_linear').value
        self._max_angular = self.get_parameter('max_angular').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._send_rate = self.get_parameter('send_rate').value
        self._gyro_var = self.get_parameter('gyro_var').value
        self._accel_var = self.get_parameter('accel_var').value
        self._pan_sign = self.get_parameter('pan_sign').value
        self._cell_count = self.get_parameter('cell_count').value
        self._battery_present_v = self.get_parameter('battery_present_v').value
        self._batt_samples = collections.deque(maxlen=20)
        self._stale = StaleDetector(
            self.get_parameter('imu_stale_count').value)
        self._imu_frozen = False
        self._imu_rate = RateMeter(1.0)
        self._last_sample_time = None
        self._battery_timer = self.create_timer(1.0, self._publish_battery)
        self._frame_id_imu = self.get_parameter('frame_id_imu').value
        self._frame_id_base = self.get_parameter('frame_id_base').value
        self._pan_min = self.get_parameter('pan_min').value
        self._pan_max = self.get_parameter('pan_max').value
        self._last_servo_deg = None
        self._last_servo_time = 0.0
        self._ack_event = threading.Event()
        self._last_ack = ''

        self.create_service(Trigger, 'base/stop', self._srv_stop)
        self.create_service(Trigger, 'imu/calibrate', self._srv_calibrate)
        self.create_service(Trigger, 'imu/zero_yaw', self._srv_zero_yaw)

        for name in ('track_width', 'max_wheel_speed', 'pwm_min'):
            if self.get_parameter(name).value <= 0:
                raise ValueError(
                    f'Parameter {name} must be set (> 0), see config/base.yaml'
                )

        self._ser = None
        self._write_lock = threading.Lock()
        self._running = True
        self._cmd = (0.0, 0.0)
        self._cmd_time = None
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._stop_sent = True
        self._send_timer = self.create_timer(
            1.0 / self.get_parameter('send_rate').value,
            self._send_tick)
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)

        # Create Publishers
        self.imu_publisher = self.create_publisher(
            Imu,
            'imu/data_raw',
            qos_profile_sensor_data
        )
        self.battery_publisher = self.create_publisher(
            BatteryState,
            'battery',
            10
        )
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            'diagnostics',
            10
        )
        self.base_debug_yaw_publisher = self.create_publisher(
            Float64,
            'base/debug/yaw',
            qos_profile_sensor_data
        )

        # Create Subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            callback=self.cmd_vel_callback,
            qos_profile=10
        )
        self.pan_cmd_subscriber = self.create_subscription(
            Float64,
            'pan/cmd',
            callback=self.pan_cmd_callback,
            qos_profile=10
        )

        self._reader.start()

    def _open_port(self) -> bool:
        """Open the serial port. Return True on success, False if it failed."""
        if self._ser is None:
            try:
                self._ser = serial.Serial(
                    self.get_parameter('port').value,
                    self.get_parameter('baud').value,
                    timeout=0.1
                )
                self._ser.reset_input_buffer()
                self.get_logger().info(
                    f"Opened Port: {self.get_parameter('port').value}"
                )
                self.get_logger().info(
                    f"At Baud Rate: {self.get_parameter('baud').value}"
                )
                self._write_line('S\n')
                return True
            except (serial.SerialException, OSError):
                return False

    def _close_port(self) -> None:
        """Close the port if open and forget it."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _write_line(self, text: str) -> bool:
        """Send one line to the R4. Return True if it was sent."""
        with self._write_lock:
            if self._ser is None:
                return False
            else:
                try:
                    self._ser.write(text.encode())
                    return True
                except Exception:
                    self._close_port()
                    return False
        return False

    def _reader_loop(self) -> None:
        """Run in the reader thread: (re)connect, read lines, parse them."""
        waiting_logged = False

        while self._running:
            if self._ser is None:
                if not self._open_port():
                    if not waiting_logged:
                        self.get_logger().info(
                            f"Waiting for port:{
                                self.get_parameter('port').value}")
                        waiting_logged = True
                    time.sleep(1.0)
                    continue
                waiting_logged = False
            else:
                try:
                    raw = self._ser.readline()
                    if not raw:
                        continue
                    raw = raw.strip()
                    self._handle_line(parse_line(raw.decode(errors='replace')))
                except (serial.SerialException, OSError):
                    self.get_logger().info('R4 link lost')
                    self._close_port()
                    continue

    def _send_tick(self) -> None:
        """
        Run 20x per second: send M l r while a command is fresh.

        else S once.
        """
        stale = (self._cmd_time is None
                 or (time.monotonic() - self._cmd_time) > self._cmd_timeout)

        if stale:
            if not self._stop_sent:
                self._write_line('S\n')
                self._stop_sent = True
            return

        v, w = self._cmd
        v = max(-self.get_parameter('max_linear').value,
                min(self.get_parameter('max_linear').value, v))

        w = max(-self.get_parameter('max_angular').value,
                min(self.get_parameter('max_angular').value, w))

        left, right = twist_to_pwm(v, w,
                                   self.get_parameter('track_width').value,
                                   self.get_parameter('max_wheel_speed').value,
                                   self.get_parameter('pwm_min').value
                                   )
        self._write_line(format_drive(left, right))
        self._stop_sent = False

    def _handle_line(self, msg) -> None:
        """Send one parsed line to the handler for its type."""
        if isinstance(msg, ImuSample):
            self._on_imu(msg)
        elif isinstance(msg, Ack):
            self._on_ack(msg)
        elif isinstance(msg, WatchdogStop):
            self._on_watchdog()
        # None (garbled/unknown line): ignored for now, counted in 4.6

    def _on_imu(self, s: ImuSample) -> None:
        stamp = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = self._frame_id_imu
        m.orientation.w = 1.0
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.x = dps_to_radps(s.gyro_x)
        m.angular_velocity.y = dps_to_radps(s.gyro_y)
        m.angular_velocity.z = dps_to_radps(s.gyro_z)
        m.linear_acceleration.x = g_to_mps2(s.accel_x)
        m.linear_acceleration.y = g_to_mps2(s.accel_y)
        m.linear_acceleration.z = g_to_mps2(s.accel_z)
        g = self._gyro_var
        x = self._accel_var
        m.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        m.linear_acceleration_covariance = [
            x, 0.0, 0.0, 0.0, x, 0.0, 0.0, 0.0, x]
        self.imu_publisher.publish(m)

        j = JointState()
        j.header.stamp = stamp
        j.name = ['pan_joint']
        j.position = [servo_deg_to_pan_rad(s.servo_deg, self._pan_sign)]
        self.joint_state_publisher.publish(j)

        self.base_debug_yaw_publisher.publish(
            Float64(data=math.radians(s.yaw_deg)))

        self._batt_samples.append(s.batt_voltage)
        self._imu_frozen = self._stale.update(
            (s.accel_x, s.accel_y, s.accel_z,
             s.gyro_x, s.gyro_y, s.gyro_z))
        now = time.monotonic()
        self._imu_rate.tick(now)
        self._last_sample_time = now

    def _on_ack(self, ack: Ack) -> None:
        self._last_ack = ack.text
        self._ack_event.set()

    def _on_watchdog(self) -> None:
        pass

    def _publish_battery(self) -> None:
        """Publish the mean battery voltage of the last ~1 s."""
        if not self._batt_samples:
            return
        v = sum(self._batt_samples) / len(self._batt_samples)
        nan = float('nan')
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = self._frame_id_base
        b.voltage = v
        b.present = v > self._battery_present_v
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.current = b.charge = b.capacity = b.design_capacity = nan
        b.percentage = b.temperature = nan
        b.cell_voltage = [v / self._cell_count] * self._cell_count
        b.location = 'base'
        self.battery_publisher.publish(b)

    def cmd_vel_callback(self, msg):
        """Listen to incoming cmd_vel topic."""
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_time = time.monotonic()

    def pan_cmd_callback(self, msg) -> None:
        """Listen to incoming pan_cmd topic."""
        deg = pan_rad_to_servo_deg(msg.data, self._pan_sign, self._pan_min,
                                   self._pan_max)
        if (self._last_servo_deg is not None and
                abs(deg - self._last_servo_deg) < 1):
            return

        if time.monotonic() - self._last_servo_time < 0.05:
            return

        if self._write_line(format_servo(deg)):
            self._last_servo_deg = deg
            self._last_servo_time = time.monotonic()

    def _send_and_wait(self, line: str,
                       expect: str, timeout: float) -> tuple[bool, str]:
        if self._ser is None:
            return (False, 'link down')

        self._ack_event.clear()
        if not self._write_line(line):
            return (False, 'write failed')

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._ack_event.wait(deadline - time.monotonic()):
                break
            if self._last_ack == expect:
                return (True, expect)
            else:
                self._ack_event.clear()

        return (False, f'no "{expect}" within {timeout} s')

    def _srv_stop(self, request: Request, response: Response) -> Response:
        self._cmd_time = None
        (response.success, response.message) = self._send_and_wait(
            'S\n', 'S ok', 0.5)
        return response

    def _srv_calibrate(self, request: Request, response: Response) -> Response:
        (response.success, response.message) = self._send_and_wait(
            'C\n', 'C ok', 3.0)
        return response

    def _srv_zero_yaw(self, request: Request, response: Response) -> Response:
        (response.success, response.message) = self._send_and_wait(
            'Z\n', 'Z ok', 0.5)
        return response


def main(args=None):
    """Start Base Bridge Node."""
    rclpy.init(args=args)
    base_bridge = BaseBridge()
    rclpy.spin(base_bridge)
    base_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
