"""Base Bridge Node for Scout Base ROS2 package."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import threading
import serial
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState, JointState
from diagnostic_msgs.msg import DiagnosticArray

from scout_base_py.protocol import format_drive, parse_line, ImuSample
from scout_base_py.kinematics import twist_to_pwm


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

        for name in ('track_width', 'max_wheel_speed', 'pwm_min'):
            if self.get_parameter(name).value <= 0:
                raise ValueError(
                    f'Parameter {name} must be set (> 0), see config/base.yaml'
                )

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
        i_count = 0
        last_report = time.monotonic()

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
                    msg = parse_line(raw.decode(errors='replace'))
                    if isinstance(msg, ImuSample):
                        i_count += 1
                    if time.monotonic() - last_report >= 1.0:
                        self.get_logger().info(f'I lines: {i_count}')
                        i_count = 0
                        last_report = time.monotonic()
                except (serial.SerialException, OSError):
                    self.get_logger().info('R4 link lost')
                    self._close_port()
                    continue

    def _send_tick(self) -> None:
        """Run 20x per second: send M l r while a command is fresh.

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

    def cmd_vel_callback(self, msg):
        """Listen to incoming cmd_vel topic."""
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_time = time.monotonic()

    def pan_cmd_callback(self, msg):
        """Listen to incoming pan_cmd topic."""
        self.get_logger().info('I heard: "%s' % msg.data)


def main(args=None):
    """Start Base Bridge Node."""
    rclpy.init(args=args)
    base_bridge = BaseBridge()
    rclpy.spin(base_bridge)
    base_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
