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
SIL stand-in for Lab 10's localization (rf2o + EKF): perfect odometry.

Subscribes to fake_lidar's /ground_truth/odom (frame world) and republishes it
as /odometry/filtered (frame odom) plus the TF odom -> base_link. With the
default start pose the world frame equals odom at t = 0. Use it only when the
Lab 10 localization launch isn't available; never on the robot.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GtToTf(Node):
    """Relay ground truth to odometry/filtered and TF."""

    def __init__(self):
        super().__init__('gt_to_tf')
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_odom(self, m):
        """Republish one ground-truth message in the odom frame."""
        m.header.frame_id = 'odom'
        self.pub.publish(m)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    """Spin until Ctrl+C."""
    rclpy.init()
    node = GtToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
