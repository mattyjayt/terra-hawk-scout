#!/usr/bin/env python3
"""Fake camera: publish a JPEG or a looping video as.

    camera/image_raw (SIL without the Pi camera).

Stands in for camera_ros until the real driver works (Lab 8.1).
It publishes the same
topics, frame and QoS as IF-ROS-TOPICS T10 + T12:
  camera/image_raw    sensor_msgs/msg/Image       (SENSOR QoS,
  frame camera_optical_frame)
  camera/camera_info  sensor_msgs/msg/CameraInfo  (pinhole K from the
  horizontal FOV)

    python3 tools/fake_camera.py --image /tmp/bus.jpg
    python3 tools/fake_camera.py --video ~/clips/walk.mp4 --rate 15
    python3 tools/fake_camera.py --image /tmp/bus.jpg --encoding rgb8
    python3 tools/fake_camera.py --image /tmp/bus.jpg --shift-px 150

--shift-px moves the picture sideways in a slow sine (period --shift-period s),
so a
person in a still photo walks across the frame. Pixels that slide in are black.
Do not commit images or videos of people (privacy note in Lab 8).
"""
import argparse
import math
import sys
import time

import cv2

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image


class FakeCamera(Node):
    """Fake Camera Node."""

    def __init__(self, args):
        """Start Fake Camera."""
        super().__init__('fake_camera')
        self.args = args
        self.w, self.h = args.width, args.height
        self.cap = None
        self.still = None
        if args.video:
            self.cap = cv2.VideoCapture(args.video)
            if not self.cap.isOpened():
                sys.exit(f'cannot open video {args.video}')
        else:
            img = cv2.imread(args.image)                # OpenCV loads BGR
            if img is None:
                sys.exit(f'cannot read image {args.image}')
            self.still = self._fit(img)
        fx = (self.w / 2.0) / math.tan(math.radians(args.hfov_deg) / 2.0)
        self.info = CameraInfo()
        self.info.header.frame_id = args.frame_id
        self.info.width, self.info.height = self.w, self.h
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [0.0] * 5
        self.info.k = [fx, 0.0, self.w / 2.0, 0.0, fx, self.h / 2.0,
                       0.0, 0.0, 1.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [fx, 0.0, self.w / 2.0, 0.0, 0.0, fx, self.h / 2.0,
                       0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_img = self.create_publisher(Image, 'camera/image_raw',
                                             qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info',
                                              qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.count = 0
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            f'publishing {self.w}x{self.h} {
                args.encoding} at {args.rate} Hz, fx={fx:.1f} px, '
            f'frame_id={args.frame_id}')

    def _fit(self, img):
        return cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)

    def _next_frame(self):
        if self.still is not None:
            return self.still
        ok, img = self.cap.read()
        if not ok:                                      # end of video: loop
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, img = self.cap.read()
            if not ok:
                return None
        return self._fit(img)

    def _shift(self, img):
        a = self.args.shift_px
        if a == 0:
            return img
        t = time.monotonic() - self.t0
        dx = int(round(
            a * math.sin(2.0 * math.pi * t / self.args.shift_period)))
        m = np.float32([[1, 0, dx], [0, 1, 0]])
        return cv2.warpAffine(img, m, (self.w, self.h))

    def _tick(self):
        bgr = self._next_frame()
        if bgr is None:
            return
        bgr = self._shift(bgr)
        if self.args.encoding == 'rgb8':
            pixels = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        else:
            pixels = bgr
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.frame_id
        msg.height, msg.width = self.h, self.w
        msg.encoding = self.args.encoding
        msg.is_bigendian = 0
        msg.step = self.w * 3
        msg.data = np.ascontiguousarray(pixels).tobytes()
        self.pub_img.publish(msg)
        self.info.header.stamp = stamp
        self.pub_info.publish(self.info)
        self.count += 1


def main():
    """Start fake camera testing."""
    ap = argparse.ArgumentParser(
        description='Publish a JPEG or video as camera/image_raw')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--image', help='still image (jpg/png)')
    src.add_argument('--video', help='video file, looped')
    ap.add_argument('--rate', type=float, default=15.0, help='Hz (default 15)')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--encoding', choices=['bgr8', 'rgb8'], default='bgr8')
    ap.add_argument('--frame-id', default='camera_optical_frame')
    ap.add_argument('--hfov-deg', type=float,
                    default=66.0, help='Camera Module 3 standard: 66')
    ap.add_argument('--shift-px', type=int,
                    default=0, help='sine amplitude of a sideways slide')
    ap.add_argument('--shift-period',
                    type=float, default=8.0, help='s per slide cycle')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeCamera(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
