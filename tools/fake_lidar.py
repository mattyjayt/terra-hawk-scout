#!/usr/bin/env python3
# Copyright 2026 Jaime Tellie
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Fake 2D LiDAR for software-in-the-loop tests of the Scout.

A tiny 2D world (a rectangular room plus box obstacles) and a robot that
moves when it receives ``/cmd_vel``. Every scan period the node ray-casts
the world from the laser pose and publishes ``/scan`` exactly like the
RPLIDAR C1 driver would (frame ``base_laser``, 360 beams, inf = no echo).

Interface (Labs 9-11 rely on it; change it only with the labs):

- sub ``/cmd_vel`` (geometry_msgs/Twist): robot velocity command. The pose
  is integrated at 50 Hz; no command for ``--cmd-timeout`` s means stop.
- pub ``/scan`` (sensor_msgs/LaserScan, SENSOR QoS = best effort), frame
  ``base_laser``, ``--rate`` Hz (10), ``--beams`` beams (360) over
  [-pi, pi), range_min 0.05 m, range_max 12.0 m, gaussian noise 0.01 m.
- pub ``/ground_truth/odom`` (nav_msgs/Odometry, frame ``world``, child
  ``base_link``) with ``--publish-ground-truth``. With the default start
  pose (0, 0, 0) the ``world`` frame coincides with ``odom`` and ``map``
  at t = 0, so poses can be compared directly.
- pub ``/imu/fake`` (sensor_msgs/Imu, frame ``imu_link``, SENSOR QoS,
  ``--imu-rate`` Hz) with ``--publish-imu``: gyro z = the simulated true yaw
  rate + noise + bias; accel = gravity on +z. Point the EKF's ``imu0`` at
  it for SIL (``tools/fake_r4.py``'s gyro does not follow this world).
- pub ``/odom/cmd`` (nav_msgs/Odometry, frame ``odom``) with
  ``--publish-cmd-odom``: dead reckoning from the *commanded* velocity,
  i.e. what open-loop odometry would believe. No TF.
- static TF ``base_link -> base_laser`` with ``--publish-laser-tf`` (only
  when no robot_state_publisher / URDF runs).

The world is never drawn; ``--world FILE`` reads a YAML file::

    room: [4.0, 3.0]            # size x, y in m, centred on the origin
    boxes:                      # [centre_x, centre_y, size_x, size_y] in m
      - [1.0, 0.8, 0.4, 0.3]
    start: [0.0, 0.0, 0.0]      # optional robot start x, y, yaw

Only numpy (and PyYAML for ``--world``, which ROS 2 always ships) is needed.
"""

import argparse
import math
import sys
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

DEFAULT_ROOM = (4.0, 3.0)
# [centre_x, centre_y, size_x, size_y]: asymmetric on purpose, so that a
# scan matcher can't confuse the room with its mirror image.
DEFAULT_BOXES = (
    (1.6, 0.9, 0.40, 0.30),     # cabinet, front-left corner
    (-1.1, -0.75, 0.30, 0.50),  # box, rear-right
    (0.5, -0.8, 0.12, 0.12),    # table leg, front-right
    (-1.3, 0.95, 0.50, 0.25),   # shelf, rear-left
    (0.5, 0.5, 0.15, 0.15),     # plant pot, inside the 1 m test square
)
# The 1 m square (0,0) -> (1,0) -> (1,1) -> (0,1) keeps >= 0.3 m from every box.
GRAVITY = 9.80665


def yaw_to_quaternion(yaw):
    """
    Return (x, y, z, w) of a rotation by ``yaw`` about +z.

    Args:
    ----
    yaw: heading in rad.

    Returns
    -------
    A 4-tuple quaternion.

    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle(a):
    """Wrap an angle in rad to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def world_segments(room, boxes):
    """
    Build the wall segments of the world.

    Args:
    ----
    room: (size_x, size_y) of the room, centred on the origin.
    boxes: iterable of (centre_x, centre_y, size_x, size_y).

    Returns
    -------
    Two (M, 2) arrays: segment start points and end points.

    """
    rects = [(0.0, 0.0, room[0], room[1])] + [tuple(b) for b in boxes]
    starts, ends = [], []
    for cx, cy, sx, sy in rects:
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            starts.append(corners[i])
            ends.append(corners[(i + 1) % 4])
    return np.array(starts, dtype=float), np.array(ends, dtype=float)


def ray_cast(origins, angles, seg_a, seg_b, max_range):
    """
    Return the distance along each ray to the nearest segment.

    Args:
    ----
    origins: (N, 2) ray start points (one per beam) in the world frame.
    angles: (N,) ray directions in rad, world frame.
    seg_a: (M, 2) segment start points.
    seg_b: (M, 2) segment end points.
    max_range: distance returned when a ray hits nothing.

    Returns
    -------
    (N,) array of distances in m (``inf`` where nothing is hit).

    """
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)[:, None, :]
    o = origins[:, None, :]
    e = (seg_b - seg_a)[None, :, :]
    w = seg_a[None, :, :] - o
    # Solve o + t*d = a + u*e with 2D cross products.
    denom = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[..., 0] * e[..., 1] - w[..., 1] * e[..., 0]) / denom
        u = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    dist = t.min(axis=1)
    dist[dist > max_range] = np.inf
    return dist


def collides(x, y, radius, room, boxes):
    """
    Return True if a disc of ``radius`` at (x, y) overlaps a wall or box.

    Args:
    ----
    x: disc centre x in m (world frame).
    y: disc centre y in m (world frame).
    radius: disc radius in m.
    room: (size_x, size_y), centred on the origin.
    boxes: iterable of (centre_x, centre_y, size_x, size_y).

    Returns
    -------
    True on overlap.

    """
    if abs(x) > room[0] / 2.0 - radius or abs(y) > room[1] / 2.0 - radius:
        return True
    for cx, cy, sx, sy in boxes:
        dx = max(abs(x - cx) - sx / 2.0, 0.0)
        dy = max(abs(y - cy) - sy / 2.0, 0.0)
        if dx * dx + dy * dy < radius * radius:
            return True
    return False


class FakeLidar(Node):
    """Simulate the Scout's motion and its 2D LiDAR in a box world."""

    def __init__(self, args):
        """Create publishers, the subscriber and the timers."""
        super().__init__('fake_lidar')
        self.args = args
        self.rng = np.random.default_rng(args.seed)
        self.room, self.boxes, start = self._load_world(args)
        self.seg_a, self.seg_b = world_segments(self.room, self.boxes)

        # True robot state (world frame) and the open-loop estimate.
        self.x, self.y, self.yaw = start
        self.v, self.w = 0.0, 0.0                   # true velocities
        self.cx, self.cy, self.cyaw = 0.0, 0.0, 0.0  # commanded dead reckoning
        self.cmd_v, self.cmd_w = 0.0, 0.0
        self.cmd_time = None
        self.last_step = time.monotonic()
        self.history = []                           # (t, x, y, yaw) for --distort
        self.bumps = 0
        self.turn_slip = args.slip if args.turn_slip is None else args.turn_slip
        self.gyro_bias = args.gyro_bias

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan',
                                              qos_profile_sensor_data)
        self.gt_pub = None
        if args.publish_ground_truth:
            self.gt_pub = self.create_publisher(Odometry, 'ground_truth/odom', 10)
        self.cmd_odom_pub = None
        if args.publish_cmd_odom:
            self.cmd_odom_pub = self.create_publisher(Odometry, 'odom/cmd', 10)
        self.imu_pub = None
        if args.publish_imu:
            self.imu_pub = self.create_publisher(Imu, 'imu/fake',
                                                 qos_profile_sensor_data)
            self.create_timer(1.0 / args.imu_rate, self._publish_imu)
        if args.publish_laser_tf:
            self._send_laser_tf()

        self.create_timer(1.0 / 50.0, self._step)
        self.create_timer(1.0 / args.rate, self._publish_scan)
        self.get_logger().info(
            f'fake_lidar: room {self.room[0]:.2f} x {self.room[1]:.2f} m, '
            f'{len(self.boxes)} boxes, start ({start[0]:.2f}, {start[1]:.2f}, '
            f'{start[2]:.2f}), slip {args.slip:.2f}/{self.turn_slip:.2f}, '
            f'{args.beams} beams at {args.rate:.1f} Hz')

    # ----- setup -----
    @staticmethod
    def _load_world(args):
        """Return (room, boxes, start) from defaults, the YAML file and args."""
        room, boxes = DEFAULT_ROOM, list(DEFAULT_BOXES)
        start = tuple(args.start)
        if args.world:
            import yaml  # noqa: I001 - ROS 2 always ships PyYAML
            with open(args.world, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            room = tuple(float(v) for v in data.get('room', room))
            boxes = [tuple(float(v) for v in b) for b in data.get('boxes', boxes)]
            if 'start' in data:
                start = tuple(float(v) for v in data['start'])
        if args.room:
            room = tuple(args.room)
        if args.no_boxes:
            boxes = []
        if collides(start[0], start[1], args.robot_radius, room, boxes):
            raise SystemExit(f'start pose {start} is inside a wall or box')
        return room, boxes, start

    def _send_laser_tf(self):
        """Publish base_link -> base_laser once on /tf_static."""
        self.tf_static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.args.frame_id
        t.transform.translation.x = self.args.laser_x
        t.transform.translation.y = self.args.laser_y
        t.transform.translation.z = self.args.laser_z
        qx, qy, qz, qw = yaw_to_quaternion(self.args.laser_yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_static.sendTransform(t)

    # ----- motion -----
    def _on_cmd_vel(self, msg):
        """Store the newest velocity command and its arrival time."""
        self.cmd_v, self.cmd_w = msg.linear.x, msg.angular.z
        self.cmd_time = time.monotonic()

    def _step(self):
        """Integrate the true pose and the commanded dead reckoning (50 Hz)."""
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now
        fresh = self.cmd_time is not None and now - self.cmd_time < self.args.cmd_timeout
        cmd_v, cmd_w = (self.cmd_v, self.cmd_w) if fresh else (0.0, 0.0)

        noise = self.args.vel_noise
        v = cmd_v * (1.0 - self.args.slip)
        w = cmd_w * (1.0 - self.turn_slip)
        if noise > 0.0 and (cmd_v or cmd_w):
            v *= 1.0 + self.rng.normal(0.0, noise)
            w *= 1.0 + self.rng.normal(0.0, noise)

        yaw_mid = self.yaw + 0.5 * w * dt
        nx = self.x + v * math.cos(yaw_mid) * dt
        ny = self.y + v * math.sin(yaw_mid) * dt
        if collides(nx, ny, self.args.robot_radius, self.room, self.boxes):
            v = 0.0
            self.bumps += 1
            if self.bumps % 50 == 1:
                self.get_logger().warn('bumped into an obstacle: not moving forward')
        else:
            self.x, self.y = nx, ny
        self.yaw = wrap_angle(self.yaw + w * dt)
        self.v, self.w = v, w

        cyaw_mid = self.cyaw + 0.5 * cmd_w * dt
        self.cx += cmd_v * math.cos(cyaw_mid) * dt
        self.cy += cmd_v * math.sin(cyaw_mid) * dt
        self.cyaw = wrap_angle(self.cyaw + cmd_w * dt)

        self.history.append((now, self.x, self.y, self.yaw))
        if len(self.history) > 50:
            del self.history[0]

        stamp = self.get_clock().now().to_msg()
        if self.gt_pub is not None:
            self.gt_pub.publish(self._odom(stamp, 'world', self.x, self.y, self.yaw,
                                           self.v, self.w))
        if self.cmd_odom_pub is not None:
            self.cmd_odom_pub.publish(self._odom(stamp, 'odom', self.cx, self.cy,
                                                 self.cyaw, cmd_v, cmd_w))

    @staticmethod
    def _odom(stamp, frame, x, y, yaw, v, w):
        """Build a nav_msgs/Odometry message for a planar pose and twist."""
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = qx, qy
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        m.twist.twist.linear.x = v
        m.twist.twist.angular.z = w
        return m

    # ----- sensors -----
    def _beam_poses(self, n, scan_time):
        """
        Return per-beam laser origins (N, 2) and headings (N,) in the world.

        Without ``--distort`` every beam starts from the current pose. With
        it, beam i starts from the robot pose at scan start + i * dt, like a
        spinning sensor on a moving robot (motion distortion).
        """
        a = self.args
        if a.distort and len(self.history) > 1:
            t_now = self.history[-1][0]
            ts = t_now - scan_time + np.arange(n) * (scan_time / n)
            hist = np.array(self.history)
            xs = np.interp(ts, hist[:, 0], hist[:, 1])
            ys = np.interp(ts, hist[:, 0], hist[:, 2])
            yaws = np.interp(ts, hist[:, 0], np.unwrap(hist[:, 3]))
        else:
            xs = np.full(n, self.x)
            ys = np.full(n, self.y)
            yaws = np.full(n, self.yaw)
        c, s = np.cos(yaws), np.sin(yaws)
        ox = xs + c * a.laser_x - s * a.laser_y
        oy = ys + s * a.laser_x + c * a.laser_y
        return np.stack([ox, oy], axis=1), yaws + a.laser_yaw

    def _publish_scan(self):
        """Ray-cast the world from the laser pose and publish /scan."""
        a = self.args
        n = a.beams
        inc = 2.0 * math.pi / n
        scan_time = 1.0 / a.rate
        origins, headings = self._beam_poses(n, scan_time)
        beam_angles = -math.pi + inc * np.arange(n)
        ranges = ray_cast(origins, headings + beam_angles, self.seg_a, self.seg_b,
                          a.range_max)
        finite = np.isfinite(ranges)
        if a.range_noise > 0.0:
            ranges[finite] += self.rng.normal(0.0, a.range_noise, finite.sum())
        ranges[finite & (ranges < a.range_min)] = np.inf
        if a.dropout > 0.0:
            ranges[self.rng.random(n) < a.dropout] = np.inf

        m = LaserScan()
        stamp = self.get_clock().now()
        if a.distort:
            stamp = stamp - Duration(seconds=scan_time)
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = a.frame_id
        m.angle_min = -math.pi
        m.angle_increment = inc
        m.angle_max = -math.pi + inc * (n - 1)
        m.time_increment = scan_time / n
        m.scan_time = scan_time
        m.range_min = a.range_min
        m.range_max = a.range_max
        m.ranges = ranges.astype(np.float32).tolist()
        m.intensities = np.where(np.isfinite(ranges), 47.0, 0.0).astype(np.float32).tolist()
        self.scan_pub.publish(m)

    def _publish_imu(self):
        """Publish a gyro that follows the simulated yaw rate."""
        a = self.args
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        m.orientation.w = 1.0
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = (self.w + self.gyro_bias
                                + float(self.rng.normal(0.0, a.gyro_noise)))
        m.angular_velocity.x = float(self.rng.normal(0.0, a.gyro_noise))
        m.angular_velocity.y = float(self.rng.normal(0.0, a.gyro_noise))
        g = a.gyro_noise ** 2 if a.gyro_noise > 0.0 else 8.7e-6
        m.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        m.linear_acceleration.z = GRAVITY
        m.linear_acceleration_covariance = [2e-3, 0.0, 0.0, 0.0, 2e-3, 0.0, 0.0, 0.0, 2e-3]
        self.imu_pub.publish(m)


def parse_args(argv):
    """Parse the command line (ROS arguments already removed)."""
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--world', help='YAML world file (room, boxes, start)')
    p.add_argument('--room', type=float, nargs=2, metavar=('X', 'Y'),
                   help='room size in m (default 4.0 3.0)')
    p.add_argument('--no-boxes', action='store_true', help='empty room')
    p.add_argument('--start', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                   metavar=('X', 'Y', 'YAW'), help='start pose in the world (m, rad)')
    p.add_argument('--laser-x', type=float, default=0.0, help='base_link -> base_laser x (m)')
    p.add_argument('--laser-y', type=float, default=0.0, help='base_link -> base_laser y (m)')
    p.add_argument('--laser-z', type=float, default=0.12, help='base_link -> base_laser z (m)')
    p.add_argument('--laser-yaw', type=float, default=0.0, help='laser yaw offset (rad)')
    p.add_argument('--frame-id', default='base_laser', help='LaserScan frame_id')
    p.add_argument('--beams', type=int, default=360, help='beams per scan')
    p.add_argument('--rate', type=float, default=10.0, help='scan rate (Hz)')
    p.add_argument('--range-min', type=float, default=0.05, help='m')
    p.add_argument('--range-max', type=float, default=12.0, help='m')
    p.add_argument('--range-noise', type=float, default=0.01, help='range sigma (m)')
    p.add_argument('--dropout', type=float, default=0.0,
                   help='fraction of beams randomly returned as inf (glass, black)')
    p.add_argument('--distort', action='store_true',
                   help='cast each beam from the pose at its own time (motion distortion)')
    p.add_argument('--slip', type=float, default=0.0,
                   help='true speed = (1 - slip) * commanded speed')
    p.add_argument('--turn-slip', type=float, default=None,
                   help='slip of the yaw rate (default: same as --slip)')
    p.add_argument('--vel-noise', type=float, default=0.0,
                   help='relative velocity noise sigma (e.g. 0.05)')
    p.add_argument('--cmd-timeout', type=float, default=0.25,
                   help='stop when /cmd_vel is older than this (s)')
    p.add_argument('--robot-radius', type=float, default=0.12, help='collision radius (m)')
    p.add_argument('--publish-ground-truth', action='store_true',
                   help='publish /ground_truth/odom (frame world)')
    p.add_argument('--publish-imu', action='store_true', help='publish /imu/fake')
    p.add_argument('--imu-rate', type=float, default=20.0, help='Hz')
    p.add_argument('--gyro-noise', type=float, default=0.003, help='gyro sigma (rad/s)')
    p.add_argument('--gyro-bias', type=float, default=0.0, help='gyro z bias (rad/s)')
    p.add_argument('--publish-cmd-odom', action='store_true',
                   help='publish /odom/cmd (open-loop dead reckoning)')
    p.add_argument('--publish-laser-tf', action='store_true',
                   help='publish base_link -> base_laser on /tf_static')
    p.add_argument('--seed', type=int, default=None, help='random seed')
    return p.parse_args(argv)


def main(argv=None):
    """Run the fake LiDAR until Ctrl+C."""
    argv = sys.argv if argv is None else argv
    args = parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv)
    node = FakeLidar(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
