"""pan_tracker: keep the selected person centred.

with the pan servo (visual servoing + gyro FF).
"""
import bisect
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scout_perception.tracking import (
    bearing_error, fx_from_hfov, next_pan, PanParams, select_target,
    step_toward, Target,
    yaw_rate_in_base)
from sensor_msgs.msg import CameraInfo, Imu, JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


class PanTracker(Node):
    """PanTracker Node."""

    def __init__(self):
        """Initialize the Pan Tracker Node."""
        super().__init__('pan_tracker')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('hfov_deg', 66.0)
        self.declare_parameter('kp', 3.0)
        self.declare_parameter('deadband_deg', 2.0)
        self.declare_parameter('max_rate', 3.5)
        self.declare_parameter('pan_min', -1.5708)
        self.declare_parameter('pan_max', 1.5708)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('lost_hold_s', 1.0)
        self.declare_parameter('return_rate', 0.3)
        self.declare_parameter('use_gyro_ff', False)
        self.declare_parameter('base_frame', 'base_link')
        width = self.get_parameter('image_width').value
        self._p = PanParams(
            kp=self.get_parameter('kp').value,
            deadband=math.radians(self.get_parameter('deadband_deg').value),
            max_rate=self.get_parameter('max_rate').value,
            pan_min=self.get_parameter('pan_min').value,
            pan_max=self.get_parameter('pan_max').value,
            use_gyro_ff=self.get_parameter('use_gyro_ff').value)
        self._fx = fx_from_hfov(width, math.radians(
            self.get_parameter('hfov_deg').value))
        self._cx0 = width / 2.0
        self._lost_hold_s = self.get_parameter('lost_hold_s').value
        self._return_rate = self.get_parameter('return_rate').value
        self._base_frame = self.get_parameter('base_frame').value
        self._pan_cmd = None
        self._pan_meas = None        # latest measured pan angle (joint_states)
        self._pan_hist = []          # (stamp_s, pan) from joint_states, last ~2 s
        self._target_id = None
        self._pan_target = None
        self._err = None
        self._last_seen = None
        self._gyro_z = 0.0
        self._imu_q = None
        self._last_tick = time.monotonic()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(Float64, 'pan/cmd', 10)
        self.create_subscription(Detection2DArray, 'detections',
                                 self._on_detections,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, 'joint_states',
                                 self._on_joint_states, 10)
        self.create_subscription(Imu, 'imu/data_raw', self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/camera_info',
                                 self._on_camera_info,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value,
                          self._tick)
        self.get_logger().info(f'fx {self._fx:.1f} px, gyro ff {
            self._p.use_gyro_ff}')

    def _on_detections(self, msg: Detection2DArray) -> None:
        targets = []
        for detection in msg.detections:
            targets.append(Target(detection.id,
                                  detection.bbox.center.position.x,
                                  detection.bbox.center.position.y,
                                  detection.results[0].hypothesis.score))
        t = select_target(targets, self._target_id)
        if t is None:
            self._err = None
            return

        self._target_id = t.track_id
        self._err = bearing_error(t.cx, self._cx0, self._fx)
        self._last_seen = time.monotonic()
        if self._pan_meas is None:
            return
        if abs(self._err) < self._p.deadband:
            # 2.1: close enough to centre; aim at where we already are
            if self._pan_target is None:
                self._pan_target = self._pan_cmd
            return
        # 2.2: aim from where the camera pointed when this image was taken
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pan_then = self._pan_at(stamp)
        self._pan_target = min(max(
            pan_then - self._err, self._p.pan_min), self._p.pan_max)

    def _pan_at(self, t: float) -> float:
        """Return the measured pan angle at time t (s), else the latest."""
        if not self._pan_hist or t <= 0.0:
            return self._pan_meas
        i = bisect.bisect_right([h[0] for h in self._pan_hist], t) - 1
        return self._pan_hist[max(i, 0)][1]

    def _on_joint_states(self, msg: JointState) -> None:
        if 'pan_joint' not in msg.name:
            return
        self._pan_meas = msg.position[msg.name.index('pan_joint')]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        self._pan_hist.append((t, self._pan_meas))
        while self._pan_hist and self._pan_hist[0][0] < t - 2.0:
            self._pan_hist.pop(0)
        if self._pan_cmd is None:
            self._pan_cmd = self._pan_meas     # seed the command once

    def _on_imu(self, msg: Imu) -> None:
        if self._imu_q is None:
            try:
                tf = self._tf_buffer.lookup_transform(self._base_frame,
                                                      msg.header.frame_id,
                                                      Time())
            except TransformException:
                return
            r = tf.transform.rotation
            self._imu_q = (r.x, r.y, r.z, r.w)

        w = msg.angular_velocity
        self._gyro_z = yaw_rate_in_base(self._imu_q,
                                        (w.x, w.y, w.z))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0:
            self._fx = msg.k[0]
            self._cx0 = msg.k[2]

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self._pan_cmd is None:
            return
        age = now - self._last_seen if (
            self._last_seen is not None) else float('inf')
        if age < self._lost_hold_s and self._pan_target is not None:
            if self._p.use_gyro_ff:
                self._pan_target -= self._gyro_z * dt
            self._pan_cmd = step_toward(
                self._pan_cmd, self._pan_target, self._p.max_rate * dt)
        else:
            self._target_id = None
            self._pan_target = None
            self._pan_cmd = step_toward(
                self._pan_cmd, 0.0, self._return_rate * dt)

        self._pub.publish(Float64(data=self._pan_cmd))


def main(args=None):
    """Start the Pan Tracker Node."""
    rclpy.init(args=args)
    node = PanTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
