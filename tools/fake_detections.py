"""Fake person detector: publish vision_msgs/Detection2DArray.

    on detections (SIL for pan_tracker).

The person's box centre moves left and right in a sine, so you can see whether
pan/cmd turns the right way without a camera or a model.
Shape follows IF-ROS-TOPICS T13.

    python3 tools/fake_detections.py
    python3 tools/fake_detections.py --amplitude 0 --cx 520
    python3 tools/fake_detections.py --stranger
    python3 tools/fake_detections.py --gone-after 5
"""
import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import (Detection2D,
                             Detection2DArray, ObjectHypothesisWithPose)


def make_det(det_id, cx, cy, w, h, score):
    """Make a detection message."""
    d = Detection2D()
    d.id = det_id
    d.bbox.center.position.x = float(cx)
    d.bbox.center.position.y = float(cy)
    d.bbox.center.theta = 0.0
    d.bbox.size_x = float(w)
    d.bbox.size_y = float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = 'person'
    hyp.hypothesis.score = float(score)
    d.results.append(hyp)
    return d


class FakeDetections(Node):
    """Fake Detection Node."""

    def __init__(self, args):
        """Start Fake Detection Node."""
        super().__init__('fake_detections')
        self.args = args
        self.pub = self.create_publisher(
            Detection2DArray, 'detections', qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            f'cx = {args.cx} + {args.amplitude} sin(2 pi t / {
                args.period}) at {args.rate} Hz')

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        if self.args.gone_after <= 0 or t < self.args.gone_after:
            cx = self.args.cx + self.args.amplitude * math.sin(
                2.0 * math.pi * t / self.args.period)
            msg.detections.append(make_det('1', cx, 240.0, 120.0, 300.0, 0.80))
            if self.args.stranger:
                msg.detections.append(make_det('2', 100.0, 240.0,
                                               120.0, 300.0, 0.95))
        self.pub.publish(msg)


def main():
    """Start Main for the Fake Detections."""
    ap = argparse.ArgumentParser(description='Publish fake person detections')
    ap.add_argument('--rate', type=float, default=10.0,
                    help='Hz (default 10, like the real detector)')
    ap.add_argument('--cx', type=float, default=320.0,
                    help='centre of the sine, px')
    ap.add_argument('--amplitude', type=float, default=200.0, help='px')
    ap.add_argument('--period', type=float, default=8.0, help='s')
    ap.add_argument('--stranger', action='store_true',
                    help='add a 2nd person (id 2, score 0.95)')
    ap.add_argument('--gone-after', type=float, default=0.0,
                    help='s; 0 = never')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeDetections(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
